#!/usr/bin/env python3
import rospy
import json
import os
import numpy as np
from enum import Enum

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

from utils import dijkstra,minimum_distance_between_lines,astar

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

def is_direct_path(start, end, occ_map):
    # 使用数字微分的方法检查直线路径上是否有障碍物
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    steps = int(max(abs(dx), abs(dy)))
    if steps == 0:
        return True
    x_inc = dx / steps
    y_inc = dy / steps
    for i in range(steps + 1):
        x = x1 + i * x_inc
        y = y1 + i * y_inc
        xi = int(round(x))
        yi = int(round(y))
        if (xi, yi) in occ_map:
            return False  # 路径上有障碍物
    return True  # 直线路径无障碍物

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
        self.cmd_pub = rospy.Publisher('/cmd_exec', UserCmdRequest, queue_size=10000)
        self.info_sub = rospy.Subscriber('/panoramic_info', PanoramicInfo, self.panoramic_info_callback, queue_size=10)
        self.map_client = rospy.ServiceProxy('query_voxel', QueryVoxel)
        self.need_init = True  # 设置是否需要初始化地图

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

        # 为每个无人车和无人机创建状态机
        self.state_dict = {}
        self.waybill_dict = {}
        self.waybill_index_dict = {}

        # 定义高度层和空域划分
        self.altitude_levels = [-85, -95, -105, -115, -75, -65]
        self.occ_map_dict = {}  # 存储不同高度层的障碍物地图
        self.fast_path_dict = {}  # 存储不同高度层的快速通道
        self.car_paths = {}  # 存储每辆车的规划路径
        self.car_drone_key_positions = {}  # 记录每个车辆的初始位置

        # 防止车辆相撞，添加几个可以直达的放飞点
        self.car_key_point = {
            "SIM-MAGV-0001": [181,429],
            "SIM-MAGV-0002": [181,439],
            "SIM-MAGV-0003": [181,449],
            "SIM-MAGV-0004": [199,429],
            "SIM-MAGV-0005": [199,439],
            "SIM-MAGV-0006": [199,449],
        }

    # 仿真回调函数
    # TODO: 可以用来获取实时信息
    def panoramic_info_callback(self, panoramic_info):
        self.car_physical_status = panoramic_info.cars
        self.drone_physical_status = panoramic_info.drones
        self.bills_status = panoramic_info.bills
        self.score = panoramic_info.score
        self.events = panoramic_info.events

    # 系统初始化(按需)
    def sys_init(self):
        rospy.sleep(5.0)

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
        map_file_path = "/home/sdk_for_user/map_client_sdk/for_py/voxel_map.bin"  # 请根据实际路径修改
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

        for z in self.altitude_levels:
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
                if voxel.distance < 0.8:
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

        for z in self.altitude_levels:
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

            # 添加每个车辆的出生点
            for car_sn, init_pos in self.car_drone_key_positions.items():
                point = (int(init_pos.x), int(init_pos.y))
                if point not in occ_map:
                    key_points.append(point)

            # 添加每个车辆接收收发飞机的关键点
            for car_id, coords in self.car_key_point.items():
                point = tuple(coords)  # 将列表转换为元组 (x, y)
                if point not in occ_map:
                    key_points.append(point)

            # 添加扩展config透露的关键点
            key_points.append((184,434))
            key_points.append((184,440))
            key_points.append((184,446))
            key_points.append((196,434))
            key_points.append((196,440))
            key_points.append((196,446))
            key_points.append((185,425))
            key_points.append((190,425))
            key_points.append((146,186))
            key_points.append((430,184))
            key_points.append((528,172))
            key_points.append((508,514))
            key_points.append((564,394))
            key_points.append((490,390))

            # 添加地图边界上的采样点（每隔一定距离采样一次）
            boundary_sampling_step = 1  # 调整采样距离
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
    

    # 移动地面车辆的函数，增加碰撞检测
    # 小车不能设置速度，会按照物理模型的设计尽快到达目的点
    def move_car_with_start_and_end(self, car_sn, start, end, time_est, next_state):
        # 检查车辆是否处于 READY 状态
        car_status = next((car for car in self.car_physical_status if car.sn == car_sn), None)
        if car_status is None or car_status.car_work_state != CarPhysicalStatus.CAR_READY:
            print(f"车辆 {car_sn} 当前不处于 READY 状态，等待...")
            return
        # 检查是否会与其他车辆发生路径碰撞
        for other_car_sn, other_path in self.car_paths.items():
            if other_car_sn != car_sn:
                other_start, other_end = other_path
                distance = minimum_distance_between_lines(start, end, other_start, other_end)
                if distance < 4.0:
                    print(f"车辆 {car_sn} 的路径与车辆 {other_car_sn} 的路径过近，取消移动")
                    # 不发送移动指令，直接返回
                    return
        # 检查目标位置是否与其他车辆过近
        for other_car in self.car_physical_status:
            if other_car.sn != car_sn:
                other_car_pos = other_car.pos.position
                if self.des_pos_reached(end, other_car_pos, 4.0):
                    print(f"车辆 {car_sn} 的目标位置与车辆 {other_car.sn} 的当前位置过近，取消移动")
                    return
        # 记录车辆的目标位置和路径
        self.car_paths[car_sn] = (start, end)
        # 发送移动指令
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_CAR_EXEC_ROUTE
        msg.car_route_info.carSn = car_sn
        msg.car_route_info.way_point.append(start)
        msg.car_route_info.way_point.append(end)
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state_dict[car_sn] = next_state
    
    # 往车上挪机 (如果用于一开始飞机与小车绑定的时候，则是飞机从出生点直接瞬移到小车上)
    # 后续飞机和小车不用完全绑定，送飞和接驳可以是两个不同的小车
    def move_drone_on_car(self, drone_sn, car_sn, time_est, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_DRONE_ON_CAR
        msg.binding_drone.car_sn = car_sn
        msg.binding_drone.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state_dict[car_sn] = next_state

    # 网飞机上挂餐
    def move_cargo_in_drone(self, cargo_id, drone_sn, time_est):
        # 检查无人机是否处于 READY 状态
        drone_status = next((drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
        if drone_status is None or drone_status.drone_work_state != DronePhysicalStatus.READY:
            print(f"无人机 {drone_sn} 当前不处于 READY 状态，等待...")
            return
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_CARGO_IN_DRONE
        msg.binding_cargo.cargo_id = cargo_id
        msg.binding_cargo.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        # 状态转换在调用处处理

    # 移动无人机的函数，增加碰撞检测和路径优化
    def fly_one_route(self, drone_sn, car_sn, start_pos, end_pos, altitude, speed, time_est, next_state):
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
        rospy.sleep(time_est)
        self.state_dict[car_sn] = next_state

    # 抛餐函数
    def release_cargo(self, drone_sn, car_sn, time_est, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_RELEASE_CARGO
        msg.drone_msg.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state_dict[car_sn] = next_state

    # 换电函数
    def battery_replacement(self, drone_sn, car_sn, time_est, next_state):
        # 检查无人机是否处于 READY 状态
        drone_status = next((drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
        if drone_status is None or drone_status.drone_work_state != DronePhysicalStatus.READY:
            print(f"无人机 {drone_sn} 当前不处于 READY 状态，等待...")
            return
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_BATTERY_REPLACEMENT
        msg.drone_msg.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state_dict[car_sn] = next_state

    # 回收飞机函数
    def drone_retrieve(self, drone_sn, car_sn, time_est, next_state):
        # 检查无人机是否处于 READY 状态
        drone_status = next((drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
        if drone_status is None or drone_status.drone_work_state != DronePhysicalStatus.READY:
            print(f"无人机 {drone_sn} 当前不处于 READY 状态，等待...")
            return
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_DRONE_ON_BIRTHPLACE
        msg.unbind_info.drone_sn = drone_sn
        msg.unbind_info.car_sn = car_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state_dict[car_sn] = next_state

    # 主运行函数
    def running(self):
        rospy.sleep(2.0)
        car_num = len(self.car_sn_list)
        drone_num = len(self.drone_sn_list)
        unloading_station_num = len(self.unloading_cargo_stations)
        N = min(car_num, drone_num, unloading_station_num)  # 先走一个车机绑定的思路吧
        
        for i in range(N):
            car_sn = self.car_sn_list[i]
            drone_sn = self.drone_sn_list[i]
            unloading_station_sn = self.unloading_cargo_stations[i]
            self.state_dict[car_sn] = WorkState.START
            self.state_dict[drone_sn] = WorkState.START
            self.waybill_dict[drone_sn] = []
            for waybill in self.waybill_infos:
                waybill_target_position = waybill["targetPosition"]
                if waybill_target_position["x"] == int(unloading_station_sn['position']['x']) and waybill_target_position["y"] == int(unloading_station_sn['position']['y']):
                    self.waybill_dict[drone_sn].append(waybill)
            self.waybill_index_dict[drone_sn] = 0
            
        for car in self.car_physical_status:
            self.car_drone_key_positions[car.sn] = car.pos.position
            self.car_drone_key_positions[car.sn].x = self.car_key_point[car.sn][0]
            self.car_drone_key_positions[car.sn].y = self.car_key_point[car.sn][1]
        
        self.sys_init()

        while not rospy.is_shutdown() and self.state!=WorkState.FINISHED:
            for i in range(N):
                car_sn = self.car_sn_list[i]
                drone_sn = self.drone_sn_list[i]
                waybill = self.waybill_dict[drone_sn][self.waybill_index_dict[drone_sn]]

                # 获取车辆和无人机状态
                car_physical_status = next((car for car in self.car_physical_status if car.sn == car_sn), None)
                drone_physical_status = next((drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                if car_physical_status is None or drone_physical_status is None:
                    continue
                car_pos = car_physical_status.pos.position
                drone_pos = drone_physical_status.pos.position
                loading_pos = Position(
                    self.loading_cargo_point['x'],
                    self.loading_cargo_point['y'],
                    self.loading_cargo_point['z'])
                car_init_pos = car_pos  # 假设车辆初始位置为当前位置

                # 获取分配的高度层
                altitude = self.altitude_levels[i % len(self.altitude_levels)]

                state = self.state_dict[car_sn]
                
                print(f"正在处理无人车{car_sn}与无人机{drone_sn}，相应状态：{state}")

                if state == WorkState.START:
                    self.state_dict[car_sn] = WorkState.MOVE_CAR_GO_TO_LOADING_POINT
                elif state == WorkState.MOVE_CAR_GO_TO_LOADING_POINT:
                    if car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                        self.move_car_with_start_and_end(
                            car_sn, car_pos, loading_pos, 5.0, WorkState.MOVE_DRONE_ON_CAR)
                    else:
                        print(f"车辆 {car_sn} 未就绪，等待...")
                elif state == WorkState.MOVE_DRONE_ON_CAR:
                    if (self.des_pos_reached(loading_pos, car_pos, 0.5) and
                        car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                        self.car_paths.pop(car_sn, None) # 清除车辆的目标位置和路径
                        self.move_drone_on_car(
                            drone_sn, car_sn, 3.0, WorkState.MOVE_CARGO_IN_DRONE)
                elif state == WorkState.MOVE_CARGO_IN_DRONE:
                    cargo_id = waybill['cargoParam']['index']
                    self.move_cargo_in_drone(cargo_id, drone_sn, 10.0)
                    self.state_dict[car_sn] = WorkState.MOVE_CAR_TO_LEAVING_POINT
                elif state == WorkState.MOVE_CAR_TO_LEAVING_POINT:
                    if car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                        # 让小车不是返回自己的出生点，而是返回关键点
                        car_init_pos = self.car_drone_key_positions[car_sn]
                        self.move_car_with_start_and_end(
                            car_sn, car_pos, car_init_pos, 5.0, WorkState.RELEASE_DRONE_OUT)
                    else:
                        print(f"车辆 {car_sn} 未就绪，等待...")

                elif state == WorkState.RELEASE_DRONE_OUT:
                    car_init_pos = self.car_drone_key_positions[car_sn]
                    if (self.des_pos_reached(car_init_pos, car_pos, 0.5) and
                        car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY and
                        drone_physical_status.drone_work_state == DronePhysicalStatus.READY):
                        self.car_paths.pop(car_sn, None) # 清除车辆的目标位置和路径
                        start_pos = Position(car_pos.x, car_pos.y, car_pos.z)
                        end_station = waybill['targetPosition']
                        end_pos = Position(end_station['x'], end_station['y'], end_station['z']-5)
                        self.fly_one_route(
                            drone_sn, car_sn, start_pos, end_pos, altitude, 15.0, 10, WorkState.RELEASE_CARGO)
                    else:
                        print(f"无人机或车辆未就绪，等待...")
                elif state == WorkState.RELEASE_CARGO:
                    if drone_physical_status.drone_work_state == DronePhysicalStatus.READY:
                        des_pos = Position(
                            waybill['targetPosition']['x'],
                            waybill['targetPosition']['y'],
                            waybill['targetPosition']['z'])
                        if self.des_pos_reached(des_pos, drone_pos, 2.0):
                            self.release_cargo(
                                drone_sn, car_sn, 5.0, WorkState.RELEASE_DRONE_RETURN)
                    else:
                        print(f"无人机 {drone_sn} 未就绪，等待...")
                elif state == WorkState.RELEASE_DRONE_RETURN:
                    if drone_physical_status.drone_work_state == DronePhysicalStatus.READY:
                        start_pos = Position(drone_pos.x, drone_pos.y, drone_pos.z)
                        end_pos = Position(car_pos.x, car_pos.y, car_pos.z-20)
                        self.fly_one_route(
                            drone_sn, car_sn, start_pos, end_pos, altitude, 15.0, 10, WorkState.MOVE_CAR_BACK_TO_LOADING_POINT)
                    else:
                        print(f"无人机 {drone_sn} 未就绪，等待...")
                elif state == WorkState.MOVE_CAR_BACK_TO_LOADING_POINT:
                    if (self.des_pos_reached(car_pos, drone_pos, 0.8) and
                        drone_physical_status.drone_work_state == DronePhysicalStatus.READY and
                        car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                        self.move_car_with_start_and_end(
                            car_sn, car_pos, loading_pos, 5.0, WorkState.DRONE_BATTERY_REPLACEMENT)
                    else:
                        print(f"无人机{drone_sn}或车辆{car_sn}未就绪，等待...")
                elif state == WorkState.DRONE_BATTERY_REPLACEMENT:
                    if (self.des_pos_reached(loading_pos, car_pos, 0.8) and
                        car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY and
                        drone_physical_status.drone_work_state == DronePhysicalStatus.READY):
                        self.car_paths.pop(car_sn, None) # 清除车辆的目标位置和路径
                        self.battery_replacement(
                            drone_sn, car_sn, 10.0, WorkState.DRONE_RETRIEVE)
                    else:
                        print(f"无人机或车辆未就绪，等待...")
                elif state == WorkState.DRONE_RETRIEVE:
                    if (drone_physical_status.drone_work_state == DronePhysicalStatus.READY and
                        car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                        self.drone_retrieve(
                            drone_sn, car_sn, 5.0, WorkState.MOVE_DRONE_ON_CAR)
                        # 分配下一个订单
                        self.waybill_index_dict[drone_sn] += 1
                    else:
                        print(f"无人机 {drone_sn} 未就绪，等待...")
            rospy.sleep(1.0)


if __name__ == '__main__':
    race_demo = DemoPipeline()
    race_demo.running()
