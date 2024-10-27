#!/usr/bin/env python3
import rospy
import json
import os
import numpy as np
from enum import Enum
import time

import pymtmap
from concurrent.futures import ThreadPoolExecutor, as_completed

from std_msgs.msg import String
from race_demo.msg import BillStatus
from race_demo.msg import BindCargo
from race_demo.msg import CarPhysicalStatus
from race_demo.msg import CarRouteInfo
from race_demo.msg import DroneMsg
from race_demo.msg import DronePhysicalStatus
from race_demo.msg import DroneWayPoint
from race_demo.msg import DroneWayPointInfo
from race_demo.msg import DynamicPosition
from race_demo.msg import EulerAngle
from race_demo.msg import EventMsg
from race_demo.msg import PanoramicInfo
from race_demo.msg import Position
from race_demo.msg import UnBindInfo
from race_demo.msg import UserCmdRequest
from race_demo.msg import UserCmdResponse
from race_demo.msg import UserPhysicalStatus
from race_demo.msg import Voxel
from race_demo.srv import QueryVoxel, QueryVoxelRequest

from utils import dijkstra, astar, is_direct_path, astar_for_magv

class WorkState(Enum):
    START = 1
    TEST_MAP_QUERY = 2
    MOVE_CAR_GO_TO_LOADING_POINT = 3
    MOVE_DRONE_ON_CAR = 4
    MOVE_CARGO_IN_DRONE = 5
    MOVE_CAR_TO_LEAVING_POINT = 6
    RELEASE_DRONE_OUT = 7
    RELEASE_CARGO = 8
    RELEASE_DRONE_RETURN = 9
    MOVE_CAR_BACK_TO_LOADING_POINT = 10
    DRONE_BATTERY_REPLACEMENT = 11
    DRONE_RETRIEVE = 12
    FINISHED = 13
    MOVE_CAR_TO_DRONE_KEY_POINT = 14  # 添加新的状态

def check_path(pair, occ_map):
    start, end = pair
    if is_direct_path(start, end, occ_map):
        start_str = f"{start[0]}_{start[1]}"
        end_str = f"{end[0]}_{end[1]}"
        distance = np.hypot(end[0] - start[0], end[1] - start[1])
        return (start_str, end_str, distance)
    return None

# demo定义的状态流转
class DemoPipeline:
    def __init__(self):
        # 初始化ROS全局变量
        self.state = WorkState.START
        rospy.init_node('race_demo')
        # 初始化发布器
        self.cmd_pub = rospy.Publisher('/cmd_exec', UserCmdRequest, queue_size=10000)
        # 初始化订阅器
        self.cmd_sub = rospy.Subscriber('/user_cmd_response', UserCmdResponse, self.cmdResponseCallback)
        self.info_sub = rospy.Subscriber('/panoramic_info', PanoramicInfo, self.panoramic_info_callback, queue_size=10)
        self.map_client = rospy.ServiceProxy('query_voxel', QueryVoxel)
        self.need_init = False  # 设置是否需要初始化地图

        # 读取配置文件和信息，可以保留对这个格式文件的读取，但是不能假设config明文可见
        with open('/config/config.json', 'r') as file:
            self.config = json.load(file)
        self.drone_infos = self.config['taskParam']['droneParamList']
        self.car_infos = self.config['taskParam']['magvParamList']
        self.loading_cargo_point = self.config['taskParam']['loadingCargoPoint']
        self.map_boundary = self.config['taskParam']['mapBoundaryInfo']
        self.waybill_infos = self.config['taskParam']['waybillParamList'] 
        self.unloading_cargo_stations = self.config['taskParam']['unloadingCargoStationList']
        self.drone_sn_list = [drone['droneSn'] for drone in self.drone_infos]
        self.car_sn_list = [car['magvSn'] for car in self.car_infos]
        self.peer_id = self.config['peerId']
        self.task_guid = self.config['taskParam']['guid']
        self.car_physical_status = None
        self.drone_physical_status = None
        self.bills_status = None
        self.score = None
        self.events = None
        self.cmd_response_type = None
        self.cmd_description = None

        # 为每个无人车和无人机创建状态机
        self.car_state_dict = {}
        self.drone_state_dict = {}
        self.waybill_dict = {}
        # 维护已分配订单的集合
        self.assigned_orders = set()

        # 定义高度层和空域划分
        self.current_altitude_levels_for_cars = [-115, -115, -85, -105, -85, -65]
        self.full_altitude_levels = [-115, -105, -95, -85, -75, -65]
        self.occ_map_dict = {}  # 存储不同高度层的障碍物地图
        self.fast_path_dict = {}  # 存储不同高度层的快速通道
        self.car_drone_key_positions = {}  # 记录每个车辆的关键放飞位置（替代之前demo使用的初始位置）

        # 防止车辆相撞，添加几个可以直达的放飞点
        self.given_car_drone_key_point = {
            "SIM-MAGV-0001": [187,431],
            "SIM-MAGV-0002": [187,439],
            "SIM-MAGV-0003": [181,445],
            "SIM-MAGV-0004": [194,431],
            "SIM-MAGV-0005": [190,449],
            "SIM-MAGV-0006": [199,445],
        }

        self.given_car_unloading_point = {
            "SIM-MAGV-0001": [146,186],
            "SIM-MAGV-0002": [528,172],
            "SIM-MAGV-0003": [564,394],
            "SIM-MAGV-0004": [430,184],
            "SIM-MAGV-0005": [490,390],
            "SIM-MAGV-0006": [508,514],
        }

        # 初始化每个车辆的状态字典
        for car_sn in self.car_sn_list:
            self.car_state_dict[car_sn] = {
                'state': WorkState.START,
                'path': [],
                'current_waypoint_index': 0,
                'next_state': None,
            }

        # 初始化每个无人机的状态字典
        for drone_sn in self.drone_sn_list:
            self.drone_state_dict[drone_sn] = {
                'state': WorkState.START,
            }

        # 固定障碍物
        self.fixed_obstacles = [
            {'name': 'A', 'x_min': 186, 'x_max': 196, 'y_min': 432, 'y_max': 434},
            {'name': 'B', 'x_min': 184, 'x_max': 188, 'y_min': 440, 'y_max': 444},
            {'name': 'C', 'x_min': 194, 'x_max': 198, 'y_min': 440, 'y_max': 444},
        ]

        # 添加边界障碍物
        self.boundary_obstacle = {'x_min': 180, 'x_max': 200, 'y_min': 420, 'y_max': 450}

    # 仿真回调函数
    def panoramic_info_callback(self, panoramic_info):
        self.car_physical_status = panoramic_info.cars
        self.drone_physical_status = panoramic_info.drones
        self.bills_status = panoramic_info.bills
        self.score = panoramic_info.score
        self.events = panoramic_info.events

    def cmdResponseCallback(self, msg):
        self.cmd_response_type = msg.type
        self.cmd_description = msg.description

    # 系统初始化(按需)
    def sys_init(self):
        # 初始化地图和路径
        if self.need_init:
            self.init_occ_map()
            self.init_fast_paths()
            self.state = WorkState.FINISHED
            print("地图和路径初始化完成")
        else:
            # 从本地文件加载
            if os.path.exists('/root/mtuav-competition-2024/occ_map_dict.json'):
                with open('/root/mtuav-competition-2024/occ_map_dict.json', 'r') as f:
                    self.occ_map_dict = json.load(f)
            if os.path.exists('/root/mtuav-competition-2024/fast_path_dict.json'):
                with open('/root/mtuav-competition-2024/fast_path_dict.json', 'r') as f:
                    self.fast_path_dict = json.load(f)
            print("地图和路径初始化导入成功")

    # 构建障碍物地图
    def init_occ_map(self):
        print("开始构建障碍物地图...")

        # 创建 Map 实例，加载地图文件
        map_file_path = "/home/sdk_for_user/map_client_sdk/for_py/voxel_map_final.bin"  # 请根据实际路径修改
        map_instance = pymtmap.Map(map_file_path)

        # 检查地图是否有效
        if not map_instance.IsValid():
            print("地图无效，无法构建障碍物地图。")
            return

        x_min = int(max(self.map_boundary['bottomLeft']['x'], map_instance.min_x()))
        x_max = int(min(self.map_boundary['bottomRight']['x'], map_instance.max_x()))
        y_min = int(max(self.map_boundary['bottomLeft']['y'], map_instance.min_y()))
        y_max = int(min(self.map_boundary['topLeft']['y'], map_instance.max_y()))
        x_step = 1  # 采样步长，可根据需要调整
        y_step = 1

        for z in self.full_altitude_levels:
            occ_map = []
            print(f"构建高度 {z} 的障碍物地图...")
            # 准备所有需要查询的点
            points = [(x, y) for x in range(x_min, x_max + 1, x_step)
                            for y in range(y_min, y_max + 1, y_step)]
            # 定义处理单个点的函数
            def process_point(point):
                x, y = point
                voxel = map_instance.Query(x, y, z)
                if voxel.semantic == 255:
                    return None  # 超出地图范围
                if voxel.distance < 3.0001:
                    return (x, y)
                if voxel.cur_height_to_ground < 2.0001:
                    return (x, y)
                return None

            # 使用线程池并行处理
            with ThreadPoolExecutor(max_workers=64) as executor:
                futures = [executor.submit(process_point, point) for point in points]
                for future in as_completed(futures):
                    result = future.result()
                    if result is not None:
                        occ_map.append(result)
            self.occ_map_dict[str(z)] = occ_map
        # 保存到本地文件
        with open('/root/mtuav-competition-2024/occ_map_dict.json', 'w') as f:
            json.dump(self.occ_map_dict, f)
        print("完成构建障碍物地图...")

    # 构建快速通道
    def init_fast_paths(self):
        print("开始构建快速通道...")
        x_min = int(self.map_boundary['bottomLeft']['x'])
        x_max = int(self.map_boundary['bottomRight']['x'])
        y_min = int(self.map_boundary['bottomLeft']['y'])
        y_max = int(self.map_boundary['topLeft']['y'])

        for z in self.full_altitude_levels:
            occ_map = set(map(tuple, self.occ_map_dict[str(z)]))  # 转换为集合，元素为 (x, y) 元组
            paths = {}
            print(f"构建高度 {z} 的快速通道...")

            # 定义关键点集
            key_points = []

            # 添加装载点
            loading_point = (int(self.loading_cargo_point['x']), int(self.loading_cargo_point['y']))
            if loading_point not in occ_map:
                key_points.append(loading_point)

            # 添加卸货点
            for station in self.unloading_cargo_stations:
                pos = station['position']
                point = (int(pos['x']), int(pos['y']))
                if point not in occ_map:
                    key_points.append(point)

            # 添加每个车辆接收收发飞机的关键点
            for car_id, coords in self.given_car_drone_key_point.items():
                point = tuple(coords)  # 将列表转换为元组 (x, y)
                if point not in occ_map:
                    key_points.append(point)

            # 添加地图边界上的采样点（每隔一定距离采样一次）
            boundary_sampling_step = 3  # 调整采样距离
            for x in range(x_min, x_max + 1, boundary_sampling_step):
                for y in [y_min, y_max]:
                    point = (x, y)
                    if point not in occ_map:
                        key_points.append(point)
            for y in range(y_min, y_max + 1, boundary_sampling_step):
                for x in [x_min, x_max]:
                    point = (x, y)
                    if point not in occ_map:
                        key_points.append(point)

            # 移除重复的关键点
            key_points = list(set(key_points))

            print(f"高度 {z} 的关键点数量：{len(key_points)}")

            # 构建关键点之间的图，只包含有直线路径的边
            graph = {}
            from itertools import combinations

            key_point_pairs = combinations(key_points, 2)

            # 使用线程池并行处理
            with ThreadPoolExecutor(max_workers=64) as executor:
                futures = {executor.submit(check_path, pair, occ_map): pair for pair in key_point_pairs}
                for future in as_completed(futures):
                    result = future.result()
                    if result is not None:
                        start_str, end_str, distance = result
                        if start_str not in graph:
                            graph[start_str] = []
                        if end_str not in graph:
                            graph[end_str] = []
                        graph[start_str].append((end_str, distance))
                        graph[end_str].append((start_str, distance))  # 无向图

            # 将 key_points 转换为字符串形式
            key_points_str = [f"{p[0]}_{p[1]}" for p in key_points]

            self.fast_path_dict[str(z)] = {
                'key_points': key_points_str,
                'graph': graph
            }
            print(f"高度 {z} 的关键点之间的直线路径数量：{sum(len(v) for v in graph.values()) // 2}")

        # 保存到本地文件
        with open('fast_path_dict.json', 'w') as f:
            json.dump(self.fast_path_dict, f)
        print("完成构建快速通道...")

    # 检测位置到达的函数
    def des_pos_reached(self, des_pos, cur_pos, threshold):
        des = np.array([des_pos.x, des_pos.y, des_pos.z])
        cur = np.array([cur_pos.x, cur_pos.y, cur_pos.z])
        return np.linalg.norm(des - cur) < threshold

    # 获取其他车辆的占用区域，包括动态障碍物
    def get_other_car_occupied_points(self, car_sn):
        occupied_points = set()
        for other_car_sn, other_data in self.car_state_dict.items():
            if other_car_sn == car_sn:
                continue  # 跳过自己
            # 仅考虑其他车辆的当前位置到下一个目标点的路径
            waypoint_index = other_data['current_waypoint_index']
            other_path = other_data['path']
            if len(other_path) > 0 and waypoint_index < len(other_path):
                # 车辆正在移动
                current_car_physical_status = next((car for car in self.car_physical_status if car.sn == other_car_sn), None)
                if current_car_physical_status is None:
                    continue
                other_car_pos = current_car_physical_status.pos.position
                next_waypoint = other_path[waypoint_index]
                path_points = self.get_points_along_path(other_car_pos, [next_waypoint])
                for x, y in path_points:
                    for dx in range(-3, 4):
                        for dy in range(-3, 4):
                            occupied_points.add((x+dx, y+dy))
            else:
                # 车辆未移动，获取其当前位置，偏移3
                current_car_physical_status = next((car for car in self.car_physical_status if car.sn == other_car_sn), None)
                if current_car_physical_status is None:
                    continue
                x = int(current_car_physical_status.pos.position.x)
                y = int(current_car_physical_status.pos.position.y)
                for dx in range(-3, 4):
                    for dy in range(-3, 4):
                        occupied_points.add((x+dx, y+dy))
        return occupied_points

    # 获取路径上的所有点
    def get_points_along_path(self, start_pos, path):
        points = []
        last_pos = (int(start_pos.x), int(start_pos.y))
        for pos in path:
            current_pos = (int(pos.x), int(pos.y))
            x1, y1 = last_pos
            x2, y2 = current_pos
            if x1 == x2:
                # 垂直移动
                y_range = range(min(y1, y2), max(y1, y2) + 1)
                for y in y_range:
                    points.append((x1, y))
            elif y1 == y2:
                # 水平移动
                x_range = range(min(x1, x2), max(x1, x2) + 1)
                for x in x_range:
                    points.append((x, y1))
            else:
                # 不应该出现非水平或垂直的移动
                print(f"警告：检测到非水平或垂直的移动，从 {last_pos} 到 {current_pos}")
            last_pos = current_pos
        return points

    # 生成水平和垂直的路径，使用 A* 算法
    def plan_path_avoiding_obstacles(self, car_sn, start_pos, end_pos):
        start = (int(start_pos.x), int(start_pos.y))
        end = (int(end_pos.x), int(end_pos.y))

        # 构建障碍物地图
        occ_map = set()
        # 添加固定障碍物
        for obstacle in self.fixed_obstacles:
            for x in range(obstacle['x_min'], obstacle['x_max'] + 1):
                for y in range(obstacle['y_min'], obstacle['y_max'] + 1):
                    occ_map.add((x, y))
        # 添加其他车辆的占用区域
        other_cars_occupied_points = self.get_other_car_occupied_points(car_sn)
        occ_map.update(other_cars_occupied_points)

        # 定义地图范围
        x_range = (min(start[0], end[0]) - 50, max(start[0], end[0]) + 50)
        y_range = (min(start[1], end[1]) - 50, max(start[1], end[1]) + 50)

        # 使用 A* 算法进行路径规划
        path = astar_for_magv(start, end, occ_map, x_range, y_range, self.boundary_obstacle)

        if path is None:
            print(f"车辆 {car_sn} 无法找到从 {start} 到 {end} 的可行路径")
            return None

        # 优化路径，合并连续的水平或垂直移动
        optimized_path = []
        current_direction = None

        for i in range(len(path)):
            x, y = path[i]
            if i == 0:
                optimized_path.append(Position(x=x, y=y, z=start_pos.z))
                if len(path) > 1:
                    next_x, next_y = path[i + 1]
                    delta_x = next_x - x
                    delta_y = next_y - y
                    if delta_x != 0:
                        current_direction = ('x', delta_x / abs(delta_x))
                    elif delta_y != 0:
                        current_direction = ('y', delta_y / abs(delta_y))
                continue

            prev_x, prev_y = path[i - 1]
            delta_x = x - prev_x
            delta_y = y - prev_y

            if delta_x != 0:
                new_direction = ('x', delta_x / abs(delta_x))
            elif delta_y != 0:
                new_direction = ('y', delta_y / abs(delta_y))
            else:
                # No movement; skip
                continue

            if new_direction != current_direction:
                # Direction changed, add the previous point to the optimized path
                if optimized_path[-1].x != prev_x or optimized_path[-1].y != prev_y:
                    optimized_path.append(Position(x=prev_x, y=prev_y, z=start_pos.z))
                current_direction = new_direction

        # Ensure the last point is added
        if optimized_path[-1].x != path[-1][0] or optimized_path[-1].y != path[-1][1]:
            optimized_path.append(Position(x=path[-1][0], y=path[-1][1], z=start_pos.z))

        print(f"车辆 {car_sn} 规划的优化路径：")
        for pos in optimized_path:
            print(f"({int(pos.x)}, {int(pos.y)})")
        return optimized_path[1:]  # 去掉起始点

    # 移动车辆到下一个路点
    def move_car_to_next_waypoint(self, car_sn):
        waypoint_index = self.car_state_dict[car_sn]['current_waypoint_index']
        path = self.car_state_dict[car_sn]['path']
        if waypoint_index < len(path):
            current_car_physical_status = next((car for car in self.car_physical_status if car.sn == car_sn), None)
            if current_car_physical_status is None:
                print(f"未找到小车 {car_sn} 的状态信息，跳过")
                return
            car_pos = current_car_physical_status.pos.position
            next_pos = path[waypoint_index]
            self.move_car_with_start_and_end(car_sn, car_pos, next_pos, None)
        else:
            # 路径完成，进入下一个状态
            next_state = self.car_state_dict[car_sn]['next_state']
            self.car_state_dict[car_sn]['state'] = next_state
            self.car_state_dict[car_sn]['path'] = []
            self.car_state_dict[car_sn]['current_waypoint_index'] = 0
            self.car_state_dict[car_sn]['next_state'] = None

    # 为车辆设置路径并开始移动
    def move_car_along_path(self, car_sn, start_pos, end_pos, next_state):
        # 规划路径
        path = self.plan_path_avoiding_obstacles(car_sn, start_pos, end_pos)
        if path is None:
            print(f"车辆 {car_sn} 没有可行的路径，等待...")
            return  # 无法移动，等待下一次机会
        self.car_state_dict[car_sn]['path'] = path
        self.car_state_dict[car_sn]['current_waypoint_index'] = 0
        self.car_state_dict[car_sn]['next_state'] = next_state
        # 发送第一个移动指令
        self.move_car_to_next_waypoint(car_sn)

    # 移动地面车辆的函数
    def move_car_with_start_and_end(self, car_sn, start, end, next_state):            
        # 发送移动指令
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_CAR_EXEC_ROUTE
        msg.car_route_info.carSn = car_sn
        msg.car_route_info.way_point.append(start)
        msg.car_route_info.way_point.append(end)
        self.cmd_pub.publish(msg)

        print(f"车辆 {car_sn} 从 ({int(start.x)}, {int(start.y)}) 移动到 ({int(end.x)}, {int(end.y)})")

        cmd_pub_start_time = time.time()
        retry_times = 0
        while True:
            current_car_physical_status = next((car for car in self.car_physical_status if car.sn == car_sn), None)
            if current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_RUNNING:
                print(f"无人车{car_sn}成功收到指令并切换到状态：{current_car_physical_status.car_work_state}")
                break

            cmd_pub_current_time = time.time()
            if cmd_pub_current_time - cmd_pub_start_time > retry_times + 5:
                print("超时，重试发送")
                self.cmd_pub.publish(msg)
                retry_times += 5

        return True

    # 往车上挪机
    def move_drone_on_car(self, drone_sn, car_sn, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_DRONE_ON_CAR
        msg.binding_drone.car_sn = car_sn
        msg.binding_drone.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        
        cmd_pub_start_time = time.time()
        retry_times = 0
        while True:
            current_car_physical_status = next((car for car in self.car_physical_status if car.sn == car_sn), None)
            if current_car_physical_status.drone_sn:
                print(f"无人车{car_sn}成功收到指令并绑定了无人机：{current_car_physical_status.drone_sn}")
                break

            cmd_pub_current_time = time.time()
            if cmd_pub_current_time - cmd_pub_start_time > retry_times + 5:
                print("超时，重试发送")
                self.cmd_pub.publish(msg)
                retry_times += 5

        self.car_state_dict[car_sn]['state'] = next_state

    # 添加选择最佳订单的函数
    def select_best_order(self, drone_sn):
        current_time = int(time.time() * 1000)  # 获取当前时间戳（毫秒）
        available_orders = []
        for waybill in self.waybill_dict[drone_sn]:
            bill_status = next((bill for bill in self.bills_status if bill.index == waybill["cargoParam"]["index"]), None)
            if bill_status.status != BillStatus.NOT_STARTED:
                continue  # 已经开始或完成的订单
            orderTime = bill_status.orderTime
            timeout = bill_status.timeout
            if current_time < orderTime or current_time > timeout:
                continue  # 订单未开始或已超时，无法接单
            betterTime = bill_status.betterTime
            available_orders.append((bill_status, orderTime, betterTime, timeout))
        
        if not available_orders:
            return None  # 无可用订单

        # 优先选择在 betterTime 前的订单
        orders_before_better = [order for order in available_orders if current_time < order[2]]
        if orders_before_better:
            # 选择最早的 betterTime
            selected_order = min(orders_before_better, key=lambda x: x[2])
        else:
            # 选择 timeout 最远的，避免扣分
            selected_order = max(available_orders, key=lambda x: x[3])
        
        return selected_order[0]  # 返回选定的 waybill

    # 网飞机上挂餐
    def move_cargo_in_drone(self, drone_sn, car_sn, next_state):
        # 检查无人机是否处于 READY 状态
        drone_status = next((drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
        if drone_status is None or drone_status.drone_work_state != DronePhysicalStatus.READY:
            print(f"无人机 {drone_sn} 当前不处于 READY 状态，等待...")
            return

        # 选择最佳订单
        waybill = self.select_best_order(drone_sn)
        if waybill is None:
            print(f"无人机 {drone_sn} 无可用订单，等待...")
            return

        cargo_id = waybill.index
        self.assigned_orders.add(cargo_id)  # 记录已分配的订单

        # 打印订单信息
        orderTime = waybill.orderTime
        betterTime = waybill.betterTime
        timeout = waybill.timeout
        current_time = int(time.time() * 1000)
        print(f"无人机 {drone_sn} 接单：订单ID {cargo_id}, orderTime {orderTime}, betterTime {betterTime}, timeout {timeout}, 当前时间 {current_time}")

        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_CARGO_IN_DRONE
        msg.binding_cargo.cargo_id = cargo_id
        msg.binding_cargo.drone_sn = drone_sn
        self.cmd_pub.publish(msg)

        cmd_pub_start_time = time.time()
        retry_times = 0
        while True:
            current_drone_physical_status = next((drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
            if current_drone_physical_status.drone_work_state == DronePhysicalStatus.LOADING_CARGO:
                print(f"无人机{drone_sn}成功收到指令并开始装载货物：{cargo_id}, 进入装货中状态: {current_drone_physical_status.drone_work_state}")
                break
            
            cmd_pub_current_time = time.time()
            if cmd_pub_current_time - cmd_pub_start_time > retry_times + 5:
                print("超时，重试发送")
                self.cmd_pub.publish(msg)
                retry_times += 5

        self.car_state_dict[car_sn]['state'] = next_state

    # 移动无人机的函数，增加碰撞检测和路径优化
    def fly_one_route(self, drone_sn, car_sn, start_pos, end_pos, altitude, speed, next_state):
        # 检查无人机是否处于 READY 状态
        drone_status = next((drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
        if drone_status is None or drone_status.drone_work_state != DronePhysicalStatus.READY:
            print(f"无人机 {drone_sn} 当前不处于 READY 状态，等待...")
            return
        
        altitude_str = str(altitude)
        occ_map = set(map(tuple, self.occ_map_dict[altitude_str]))

        # 将起点和终点转换为整数坐标和字符串形式
        start_key = (int(start_pos.x), int(start_pos.y))
        end_key = (int(end_pos.x), int(end_pos.y))
        start_key_str = f"{start_key[0]}_{start_key[1]}"
        end_key_str = f"{end_key[0]}_{end_key[1]}"

        # 加载关键点集和图
        fast_path_data = self.fast_path_dict[altitude_str]
        key_points_str = fast_path_data['key_points']
        graph = fast_path_data['graph']

        key_points = [tuple(map(int, p_str.split('_'))) for p_str in key_points_str]

        # 找到距离起点最近的关键点
        nearest_start_point = min(key_points, key=lambda p: np.hypot(p[0]-start_key[0], p[1]-start_key[1]))
        nearest_start_point_str = f"{nearest_start_point[0]}_{nearest_start_point[1]}"
        # 找到距离终点最近的关键点
        nearest_end_point = min(key_points, key=lambda p: np.hypot(p[0]-end_key[0], p[1]-end_key[1]))
        nearest_end_point_str = f"{nearest_end_point[0]}_{nearest_end_point[1]}"

        # 构建完整路径
        full_path_coords = []

        # 起点到最近关键点
        if start_key != nearest_start_point:
            if is_direct_path(start_key, nearest_start_point, occ_map):
                full_path_coords.extend([start_key, nearest_start_point])
            else:
                # 使用 A* 计算短路径
                path = astar(start_key, nearest_start_point, occ_map,
                            (min(start_key[0], nearest_start_point[0]) - 50, max(start_key[0], nearest_start_point[0]) + 50),
                            (min(start_key[1], nearest_start_point[1]) - 50, max(start_key[1], nearest_start_point[1]) + 50),
                            1, 1)
                if path:
                    full_path_coords.extend(path)
                else:
                    print(f"无法找到从 {start_key} 到最近关键点 {nearest_start_point} 的路径")
                    return

        # 关键点之间的最短路径
        key_point_path_str = dijkstra(graph, nearest_start_point_str, nearest_end_point_str)
        if key_point_path_str:
            # 将字符串形式的点转换回坐标
            key_point_path = [tuple(map(int, p_str.split('_'))) for p_str in key_point_path_str]
            full_path_coords.extend(key_point_path)
        else:
            print(f"无法找到关键点之间的路径：{nearest_start_point} 到 {nearest_end_point}")
            return

        # 最近关键点到终点
        if end_key != nearest_end_point:
            if is_direct_path(nearest_end_point, end_key, occ_map):
                full_path_coords.extend([nearest_end_point, end_key])
            else:
                # 使用 A* 计算短路径
                path = astar(nearest_end_point, end_key, occ_map,
                            (min(nearest_end_point[0], end_key[0]) - 50, max(nearest_end_point[0], end_key[0]) + 50),
                            (min(nearest_end_point[1], end_key[1]) - 50, max(nearest_end_point[1], end_key[1]) + 50),
                            1, 1)
                if path:
                    full_path_coords.extend(path[1:])  # 避免重复添加关键点
                else:
                    print(f"无法找到从最近关键点 {nearest_end_point} 到终点 {end_key}")
                    return

        # 将路径转换为 DroneWayPoint
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_EXEC_ROUTE
        msg.drone_way_point_info.droneSn = drone_sn
        takeoff_point = DroneWayPoint()
        takeoff_point.type = DroneWayPoint.POINT_TAKEOFF
        takeoff_point.timeoutsec = 1000
        msg.drone_way_point_info.way_point.append(takeoff_point)

        # 添加 middle_point，尽量最小化数量
        # 去除连续重复的点
        optimized_coords = [full_path_coords[0]]
        for coord in full_path_coords[1:]:
            if coord != optimized_coords[-1]:
                optimized_coords.append(coord)
        
        optimized_coords.append(end_pos)
        route_len = len(optimized_coords)
        
        for i, coord in enumerate(optimized_coords):
            middle_point = DroneWayPoint()
            middle_point.type = DroneWayPoint.POINT_FLYING

            if i == route_len-1:
                middle_point.pos.x = end_pos.x
                middle_point.pos.y = end_pos.y
                middle_point.pos.z = end_pos.z
            else:
                middle_point.pos.x = coord[0]
                middle_point.pos.y = coord[1]
                middle_point.pos.z = altitude
            
            middle_point.v = speed
            middle_point.timeoutsec = 1000
            msg.drone_way_point_info.way_point.append(middle_point)

        land_point = DroneWayPoint()
        land_point.type = DroneWayPoint.POINT_LANDING
        land_point.timeoutsec = 1000
        msg.drone_way_point_info.way_point.append(land_point)
        self.cmd_pub.publish(msg)

        cmd_pub_start_time = time.time()
        retry_times = 0
        while True:
            current_drone_physical_status = next((drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
            if current_drone_physical_status.drone_work_state == DronePhysicalStatus.FLYING:
                print(f"无人机{drone_sn}成功收到指令并开始起飞状态：{current_drone_physical_status.drone_work_state}")
                break

            cmd_pub_current_time = time.time()
            if cmd_pub_current_time - cmd_pub_start_time > retry_times + 5:
                print("超时，重试发送")
                self.cmd_pub.publish(msg)
                retry_times += 5

        self.car_state_dict[car_sn]['state'] = next_state

    # 抛餐函数
    def release_cargo(self, cargo_id, drone_sn, car_sn, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_RELEASE_CARGO
        msg.drone_msg.drone_sn = drone_sn
        self.cmd_pub.publish(msg)

        cmd_pub_start_time = time.time()
        retry_times = 0
        while True:
            bill_status = next((bill for bill in self.bills_status if bill.index == cargo_id), None)
            if bill_status.status == BillStatus.OVER:
                print(f"货物{cargo_id}成功被送到")
                deliveryFinishTime = int(time.time() * 1000)
                orderTime = bill_status.orderTime
                betterTime = bill_status.betterTime
                timeout = bill_status.timeout
                print(f"订单ID {cargo_id}, 送达时间 {deliveryFinishTime}, orderTime {orderTime}, betterTime {betterTime}, timeout {timeout}")
                if orderTime <= deliveryFinishTime <= betterTime:
                    print("提前送达，加分")
                elif betterTime < deliveryFinishTime <= timeout:
                    print(f"按时送达，得分")
                else:
                    print(f"超时送达，扣分")
                break

            cmd_pub_current_time = time.time()
            if cmd_pub_current_time - cmd_pub_start_time > retry_times + 5:
                print("超时，重试发送")
                self.cmd_pub.publish(msg)
                retry_times += 5

        self.car_state_dict[car_sn]['state'] = next_state

    # 换电函数
    def battery_replacement(self, drone_sn, car_sn, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_BATTERY_REPLACEMENT
        msg.drone_msg.drone_sn = drone_sn
        self.cmd_pub.publish(msg)

        cmd_pub_start_time = time.time()
        retry_times = 0
        while True:
            current_drone_physical_status = next((drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
            if current_drone_physical_status.drone_work_state == DronePhysicalStatus.CHARGING_BATTERY:
                print(f"无人机{drone_sn}成功收到指令并进入正在充电状态：{current_drone_physical_status.drone_work_state}")
                break

            cmd_pub_current_time = time.time()
            if cmd_pub_current_time - cmd_pub_start_time > retry_times + 5:
                print("超时，重试发送")
                self.cmd_pub.publish(msg)
                retry_times += 5

        self.car_state_dict[car_sn]['state'] = next_state

    # 主运行函数
    def running(self):
        # 保证车辆都刷出来了再开始
        start_time = time.time()
        rospy.sleep(3.0)
        wait_flag = True
        while wait_flag:
            wait_flag = False
            for drone in self.drone_physical_status:
                if drone.drone_work_state != DronePhysicalStatus.READY:
                    wait_flag = True
            for car in self.car_physical_status:
                if car.car_work_state != CarPhysicalStatus.CAR_READY:
                    wait_flag = True
            for waybill in self.bills_status:
                if waybill.status != BillStatus.NOT_STARTED:
                    wait_flag = True
        print("所有飞机、车辆、货物都已准备就绪，调度正式开始！！！")

        car_num = len(self.car_sn_list)
        drone_num = len(self.drone_sn_list)
        unloading_station_num = len(self.unloading_cargo_stations)
        N = min(car_num, drone_num, unloading_station_num)  # 先走一个车机绑定的思路吧
        
        for i in range(N):
            car_sn = self.car_sn_list[i]
            drone_sn = self.drone_sn_list[i]
            unloading_station_sn = self.unloading_cargo_stations[i]
            self.waybill_dict[drone_sn] = []
            for waybill in self.waybill_infos:
                waybill_target_position = waybill["targetPosition"]
                if (waybill_target_position["x"] == self.given_car_unloading_point[car_sn][0] and 
                        waybill_target_position["y"] == self.given_car_unloading_point[car_sn][1]):
                    self.waybill_dict[drone_sn].append(waybill)
            
        for car in self.car_physical_status:
            self.car_drone_key_positions[car.sn] = car.pos.position
            self.car_drone_key_positions[car.sn].x = self.given_car_drone_key_point[car.sn][0]
            self.car_drone_key_positions[car.sn].y = self.given_car_drone_key_point[car.sn][1]

        self.sys_init()

        while not rospy.is_shutdown() and self.state!=WorkState.FINISHED:
            for i in range(6):
                car_sn = self.car_sn_list[i]
                drone_sn = self.drone_sn_list[i]
                # 获取车辆和无人机状态
                current_car_physical_status = next((car for car in self.car_physical_status if car.sn == car_sn), None)
                current_drone_physical_status = next((drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)

                if current_car_physical_status is None or current_drone_physical_status is None:
                    continue
                car_pos = current_car_physical_status.pos.position
                drone_pos = current_drone_physical_status.pos.position
                loading_pos = Position(
                    self.loading_cargo_point['x'],
                    self.loading_cargo_point['y'],
                    self.loading_cargo_point['z'])
                car_init_pos = car_pos  # 假设车辆初始位置为当前位置

                # 获取分配的高度层
                altitude = self.current_altitude_levels_for_cars[i % len(self.current_altitude_levels_for_cars)]
                state = self.car_state_dict[car_sn]['state']
                
                # 如果车辆正在沿路径移动，检查是否到达路点
                if len(self.car_state_dict[car_sn]['path']) > 0:
                    waypoint_index = self.car_state_dict[car_sn]['current_waypoint_index']
                    path = self.car_state_dict[car_sn]['path']
                    if waypoint_index < len(path):
                        next_pos = path[waypoint_index]
                        if self.des_pos_reached(next_pos, car_pos, 2.0):
                            # 到达当前路点，发送下一个移动指令
                            self.car_state_dict[car_sn]['current_waypoint_index'] += 1
                            self.move_car_to_next_waypoint(car_sn)
                    else:
                        # 路径完成，已经在 move_car_to_next_waypoint 中处理
                        pass
                    continue  # 继续下一个车辆

                print(f"正在处理无人车{car_sn}与无人机{drone_sn}，相应状态：{state}")
                print(f"无人车位置：{current_car_physical_status.pos.position.x},{current_car_physical_status.pos.position.y},{current_car_physical_status.pos.position.z}   状态：{current_car_physical_status.car_work_state}")
                print(f"无人机位置：{current_drone_physical_status.pos.position.x},{current_drone_physical_status.pos.position.y},{current_drone_physical_status.pos.position.z}   状态：{current_drone_physical_status.drone_work_state}")

                if state == WorkState.START:
                    self.car_state_dict[car_sn]['state'] = WorkState.MOVE_CAR_TO_DRONE_KEY_POINT
                elif state == WorkState.MOVE_CAR_TO_DRONE_KEY_POINT:
                    if current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                        car_pos = current_car_physical_status.pos.position
                        # 获取关键点位置
                        key_point_coords = self.given_car_drone_key_point[car_sn]
                        end_pos = Position(key_point_coords[0], key_point_coords[1], car_pos.z)
                        self.move_car_along_path(car_sn, car_pos, end_pos, WorkState.MOVE_CAR_GO_TO_LOADING_POINT)
                    else:
                        print(f"车辆 {car_sn} 未就绪，等待...")
                elif state == WorkState.MOVE_CAR_GO_TO_LOADING_POINT:
                    if current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                        car_pos = current_car_physical_status.pos.position
                        # 生成路径
                        loading_pos = Position(
                            self.loading_cargo_point['x'],
                            self.loading_cargo_point['y'],
                            car_pos.z)
                        self.move_car_along_path(car_sn, car_pos, loading_pos, WorkState.MOVE_DRONE_ON_CAR)
                    else:
                        print(f"车辆 {car_sn} 未就绪，等待...")
                elif state == WorkState.MOVE_DRONE_ON_CAR:
                    if (self.des_pos_reached(loading_pos, car_pos, 2.0) and
                            current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY and
                                current_drone_physical_status.drone_work_state == DronePhysicalStatus.READY):
                        self.move_drone_on_car(
                            drone_sn, car_sn, WorkState.MOVE_CARGO_IN_DRONE)
                    else:
                        print(f"车辆 {car_sn} 未就绪，等待...")
                elif state == WorkState.MOVE_CARGO_IN_DRONE:
                    if (self.des_pos_reached(loading_pos, car_pos, 2.0) and
                        current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY and
                            current_drone_physical_status.drone_work_state == DronePhysicalStatus.READY and
                                len(current_car_physical_status.drone_sn) > 0):
                        self.move_cargo_in_drone(drone_sn, car_sn, WorkState.MOVE_CAR_TO_LEAVING_POINT)
                    else:
                        print(f"车辆 {car_sn} 未就绪，等待...")
                elif state == WorkState.MOVE_CAR_TO_LEAVING_POINT:
                    if (current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY and
                            current_drone_physical_status.drone_work_state == DronePhysicalStatus.READY and
                                current_drone_physical_status.bind_cargo_id != 0):
                        # 让小车不是返回自己的出生点，而是返回关键点
                        car_init_pos = self.car_drone_key_positions[car_sn]
                        # 生成路径
                        self.move_car_along_path(car_sn, car_pos, car_init_pos, WorkState.RELEASE_DRONE_OUT)
                    else:
                        print(f"车辆 {car_sn} 未就绪，等待...")

                elif state == WorkState.RELEASE_DRONE_OUT:
                    car_init_pos = self.car_drone_key_positions[car_sn]
                    if (self.des_pos_reached(car_init_pos, car_pos, 2.0) and
                        current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY and
                            current_drone_physical_status.drone_work_state == DronePhysicalStatus.READY):
                        start_pos = Position(car_pos.x, car_pos.y, car_pos.z)
                        # 获取当前正在配送的订单
                        cargo_id = current_drone_physical_status.bind_cargo_id
                        waybill = next((wb for wb in self.waybill_infos if wb['cargoParam']['index'] == cargo_id), None)
                        if waybill is None:
                            print(f"未找到订单 {cargo_id}")
                            continue
                        end_station = waybill['targetPosition']
                        end_pos = Position(end_station['x'], end_station['y'], end_station['z']-5)
                        self.fly_one_route(
                            drone_sn, car_sn, start_pos, end_pos, altitude, 15.0, WorkState.RELEASE_CARGO)
                    else:
                        print(f"无人机或车辆未就绪，等待...")
                elif state == WorkState.RELEASE_CARGO:
                    if current_drone_physical_status.drone_work_state == DronePhysicalStatus.READY:
                        # 获取当前正在配送的订单
                        cargo_id = current_drone_physical_status.bind_cargo_id
                        waybill = next((wb for wb in self.waybill_infos if wb['cargoParam']['index'] == cargo_id), None)
                        if waybill is None:
                            print(f"未找到订单 {cargo_id}")
                            continue
                        des_pos = Position(
                            waybill['targetPosition']['x'],
                            waybill['targetPosition']['y'],
                            waybill['targetPosition']['z'])
                        if self.des_pos_reached(des_pos, drone_pos, 2.0):
                            self.release_cargo(
                                cargo_id, drone_sn, car_sn, WorkState.RELEASE_DRONE_RETURN)
                    else:
                        print(f"无人机 {drone_sn} 未就绪，等待...")
                elif state == WorkState.RELEASE_DRONE_RETURN:
                    if (current_drone_physical_status.drone_work_state == DronePhysicalStatus.READY and
                            current_drone_physical_status.bind_cargo_id == 0):
                        start_pos = Position(drone_pos.x, drone_pos.y, drone_pos.z)
                        end_pos = Position(car_pos.x, car_pos.y, car_pos.z-5)
                        self.fly_one_route(
                            drone_sn, car_sn, start_pos, end_pos, altitude, 15.0, WorkState.MOVE_CAR_BACK_TO_LOADING_POINT)
                    else:
                        print(f"无人机 {drone_sn} 未就绪，等待...")
                elif state == WorkState.MOVE_CAR_BACK_TO_LOADING_POINT:
                    if (self.des_pos_reached(car_pos, drone_pos, 2.0) and
                        current_drone_physical_status.drone_work_state == DronePhysicalStatus.READY and
                            current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                        
                        if current_drone_physical_status.remaining_capacity < 30:
                            car_next_state = WorkState.DRONE_BATTERY_REPLACEMENT
                        else:
                            car_next_state = WorkState.MOVE_CARGO_IN_DRONE
                        
                        car_pos = current_car_physical_status.pos.position
                        # 生成路径
                        loading_pos = Position(
                            self.loading_cargo_point['x'],
                            self.loading_cargo_point['y'],
                            car_pos.z)
                        self.move_car_along_path(car_sn, car_pos, loading_pos, car_next_state)
                    else:
                        print(f"无人机{drone_sn}或车辆{car_sn}未就绪，等待...")
                elif state == WorkState.DRONE_BATTERY_REPLACEMENT:
                    if (self.des_pos_reached(loading_pos, car_pos, 2.0) and
                        current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY and
                            current_drone_physical_status.drone_work_state == DronePhysicalStatus.READY):
                        self.battery_replacement(
                            drone_sn, car_sn, WorkState.MOVE_CARGO_IN_DRONE)
                    else:
                        print(f"无人机{drone_sn}或车辆{car_sn}未就绪，等待...")
                print("--------------------------------------------------------------")
                print(f"当前用时{time.time() - start_time}秒, 当前得分：{self.score}")
                print("--------------------------------------------------------------")
                rospy.sleep(1.0)

            # 自测阶段检查是否已经超过一小时，提交的时候应该注释掉
            if time.time() - start_time > 3700:
                self.state = WorkState.FINISHED
                print("\n运行时间已远远超过一小时，结束循环")

if __name__ == '__main__':
    race_demo = DemoPipeline()
    race_demo.running()
