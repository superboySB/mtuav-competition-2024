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

from utils import dijkstra,minimum_distance_between_lines,astar,is_direct_path

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
        self.state_dict = {}
        self.waybill_dict = {}
        self.waybill_index_dict = {}

        # 定义高度层和空域划分
        self.current_altitude_levels_for_cars = [-115, -115, -85, -105, -85, -65]
        self.full_altitude_levels = [-115, -105, -95, -85, -75, -65]
        self.occ_map_dict = {}  # 存储不同高度层的障碍物地图
        self.fast_path_dict = {}  # 存储不同高度层的快速通道
        self.car_paths = {}  # 存储每辆车的规划路径
        self.car_drone_key_positions = {}  # 记录每个车辆的关键放飞位置（替代之前demo使用的初始位置）

        # 防止车辆相撞，添加几个可以直达的放飞点
        self.given_car_drone_key_point = {
            "SIM-MAGV-0001": [181,431],
            "SIM-MAGV-0002": [181,440],
            "SIM-MAGV-0003": [181,449],
            "SIM-MAGV-0004": [199,431],
            "SIM-MAGV-0005": [199,440],
            "SIM-MAGV-0006": [199,449],
        }

        self.given_car_unloading_point = {
            "SIM-MAGV-0001": [146,186],
            "SIM-MAGV-0002": [528,172],
            "SIM-MAGV-0003": [564,394],
            "SIM-MAGV-0004": [430,184],
            "SIM-MAGV-0005": [490,390],
            "SIM-MAGV-0006": [508,514],
        }

    # 仿真回调函数, 可以用来获取实时信息，
    # TODO：考虑做一些换绑的操作的需要这个做一些预测，还有考虑进一步减少sleep时间的时候需要用这个保证状态没有冲突
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
    

    # 移动地面车辆的函数，增加碰撞检测
    # 小车不能设置速度，会按照物理模型的设计尽快到达目的点
    def move_car_with_start_and_end(self, car_sn, start, end, next_state):
        # 检查是否会与其他车辆发生路径碰撞
        # TODO：还是要保留，但好像可以结合动力学预测改进一下地上交流通，我现在设置的几个线段都是前后无冲的，所以直接移动就行了
        for other_car_sn, other_path in self.car_paths.items():
            if other_car_sn != car_sn:
                other_start, other_end = other_path
                distance = minimum_distance_between_lines(start, end, other_start, other_end)
                if distance < 0.2:
                    print(f"车辆 {car_sn} 的路径与车辆 {other_car_sn} 的目标路径过近，取消移动")
                    # 不发送移动指令，直接返回
                    return
        
        for car in self.car_physical_status:
            if car.sn != car_sn:
                other_position = car.pos.position
                distance = minimum_distance_between_lines(start, end, other_position, other_position)
                if distance < 3.0:
                    print(f"车辆 {car_sn} 的路径与车辆 {car.sn} 的当前位置过近，取消移动")
                    # 不发送移动指令，直接返回
                    return
        
        # 发送移动指令
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_CAR_EXEC_ROUTE
        msg.car_route_info.carSn = car_sn
        msg.car_route_info.way_point.append(start)
        msg.car_route_info.way_point.append(end)
        self.cmd_pub.publish(msg)
        self.state_dict[car_sn] = next_state

        # 记录移动车辆的目标位置和路径
        self.car_paths[car_sn] = [start, end]
        return
    
    # 往车上挪机 (如果用于一开始飞机与小车绑定的时候，则是飞机从出生点直接瞬移到小车上)
    # 后续飞机和小车不用完全绑定，送飞和接驳可以是两个不同的小车
    def move_drone_on_car(self, drone_sn, car_sn, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_DRONE_ON_CAR
        msg.binding_drone.car_sn = car_sn
        msg.binding_drone.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        self.state_dict[car_sn] = next_state

    # 网飞机上挂餐
    def move_cargo_in_drone(self, cargo_id, drone_sn):
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
        self.state_dict[car_sn] = next_state

    # 抛餐函数
    def release_cargo(self, drone_sn, car_sn, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_RELEASE_CARGO
        msg.drone_msg.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        self.state_dict[car_sn] = next_state

    # 换电函数
    def battery_replacement(self, drone_sn, car_sn, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_BATTERY_REPLACEMENT
        msg.drone_msg.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
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
        
        # 保证车辆都刷出来了
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


        for i in range(N):
            car_sn = self.car_sn_list[i]
            drone_sn = self.drone_sn_list[i]
            unloading_station_sn = self.unloading_cargo_stations[i]
            self.state_dict[car_sn] = WorkState.START
            self.state_dict[drone_sn] = WorkState.START
            self.waybill_dict[drone_sn] = []
            for waybill in self.waybill_infos:
                waybill_target_position = waybill["targetPosition"]
                if (waybill_target_position["x"] == self.given_car_unloading_point[car_sn][0] and 
                        waybill_target_position["y"] == self.given_car_unloading_point[car_sn][1]):
                    self.waybill_dict[drone_sn].append(waybill)
            self.waybill_index_dict[drone_sn] = 0
            
        for car in self.car_physical_status:
            self.car_drone_key_positions[car.sn] = car.pos.position
            self.car_drone_key_positions[car.sn].x = self.given_car_drone_key_point[car.sn][0]
            self.car_drone_key_positions[car.sn].y = self.given_car_drone_key_point[car.sn][1]
        
        start_time = time.time()
        self.sys_init()

        while not rospy.is_shutdown() and self.state!=WorkState.FINISHED:
            # 记录车辆的实时路径
            for car_sn in self.car_sn_list:
                car_physical_status = next((car for car in self.car_physical_status if car.sn == car_sn), None)
                car_pos = car_physical_status.pos.position
                if car_sn in self.car_paths:
                    self.car_paths[car_sn][0] = car_pos

            for i in [0,3,1,4,2,5]:  # TODO：车辆接单顺序
                car_sn = self.car_sn_list[i]
                drone_sn = self.drone_sn_list[i]
                waybill = self.waybill_dict[drone_sn][self.waybill_index_dict[drone_sn]]

                # 获取车辆和无人机状态
                car_physical_status = next((car for car in self.car_physical_status if car.sn == car_sn), None)
                drone_physical_status = next((drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                bill_status = next((bill for bill in self.bills_status if bill.index == waybill["index"]), None)
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
                altitude = self.current_altitude_levels_for_cars[i % len(self.current_altitude_levels_for_cars)]
                state = self.state_dict[car_sn]
                
                print(f"正在处理无人车{car_sn}与无人机{drone_sn}，相应状态：{state}，当前送货index：{self.waybill_index_dict[drone_sn]}，货物状态：{bill_status.status}")

                if state == WorkState.START:
                    self.state_dict[car_sn] = WorkState.MOVE_CAR_GO_TO_LOADING_POINT
                elif state == WorkState.MOVE_CAR_GO_TO_LOADING_POINT:
                    if car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                        self.move_car_with_start_and_end(
                            car_sn, car_pos, loading_pos, WorkState.MOVE_DRONE_ON_CAR)
                    else:
                        print(f"车辆 {car_sn} 未就绪，等待...")
                elif state == WorkState.MOVE_DRONE_ON_CAR:
                    if (self.des_pos_reached(loading_pos, car_pos, 0.8) and
                        car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                        self.move_drone_on_car(
                            drone_sn, car_sn, WorkState.MOVE_CARGO_IN_DRONE)
                    else:
                        print(f"车辆 {car_sn} 未就绪，等待...")
                elif state == WorkState.MOVE_CARGO_IN_DRONE:
                    if (self.des_pos_reached(loading_pos, car_pos, 0.8) and
                        car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY and
                            drone_physical_status.drone_work_state == DronePhysicalStatus.READY and
                                len(car_physical_status.drone_sn) > 0):
                        cargo_id = waybill['cargoParam']['index']
                        self.move_cargo_in_drone(cargo_id, drone_sn)
                        self.state_dict[car_sn] = WorkState.MOVE_CAR_TO_LEAVING_POINT
                    else:
                        print(f"车辆 {car_sn} 未就绪，等待...")
                elif state == WorkState.MOVE_CAR_TO_LEAVING_POINT:
                    if (car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY and
                            bill_status.status == 2):
                        # 让小车不是返回自己的出生点，而是返回关键点
                        car_init_pos = self.car_drone_key_positions[car_sn]
                        self.move_car_with_start_and_end(
                            car_sn, car_pos, car_init_pos, WorkState.RELEASE_DRONE_OUT)
                    else:
                        print(f"车辆 {car_sn} 未就绪，等待...")

                elif state == WorkState.RELEASE_DRONE_OUT:
                    car_init_pos = self.car_drone_key_positions[car_sn]
                    if (self.des_pos_reached(car_init_pos, car_pos, 0.8) and
                        car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY and
                            drone_physical_status.drone_work_state == DronePhysicalStatus.READY):
                        start_pos = Position(car_pos.x, car_pos.y, car_pos.z)
                        end_station = waybill['targetPosition']
                        end_pos = Position(end_station['x'], end_station['y'], end_station['z']-1.2)
                        self.fly_one_route(
                            drone_sn, car_sn, start_pos, end_pos, altitude, 15.0, WorkState.RELEASE_CARGO)
                    else:
                        print(f"无人机或车辆未就绪，等待...")
                elif state == WorkState.RELEASE_CARGO:
                    print(f"位置：{drone_physical_status.pos.position.x},{drone_physical_status.pos.position.y},{drone_physical_status.pos.position.z}   状态：{drone_physical_status.drone_work_state}")
                    if drone_physical_status.drone_work_state == DronePhysicalStatus.READY:
                        des_pos = Position(
                            waybill['targetPosition']['x'],
                            waybill['targetPosition']['y'],
                            waybill['targetPosition']['z'])
                        if self.des_pos_reached(des_pos, drone_pos, 2.0):
                            self.release_cargo(
                                drone_sn, car_sn, WorkState.RELEASE_DRONE_RETURN)
                    else:
                        print(f"无人机 {drone_sn} 未就绪，等待...")
                elif state == WorkState.RELEASE_DRONE_RETURN:
                    if (drone_physical_status.drone_work_state == DronePhysicalStatus.READY and
                            bill_status.status == 3):
                        self.waybill_index_dict[drone_sn] += 1
                        start_pos = Position(drone_pos.x, drone_pos.y, drone_pos.z)
                        end_pos = Position(car_pos.x, car_pos.y, car_pos.z-1.2)
                        self.fly_one_route(
                            drone_sn, car_sn, start_pos, end_pos, altitude, 15.0, WorkState.MOVE_CAR_BACK_TO_LOADING_POINT)
                    else:
                        print(f"无人机 {drone_sn} 未就绪，等待...")
                elif state == WorkState.MOVE_CAR_BACK_TO_LOADING_POINT:
                    print(f"位置：{drone_physical_status.pos.position.x},{drone_physical_status.pos.position.y},{drone_physical_status.pos.position.z}   状态：{drone_physical_status.drone_work_state}")
                    if (self.des_pos_reached(car_pos, drone_pos, 0.8) and
                        drone_physical_status.drone_work_state == DronePhysicalStatus.READY and
                            car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                        
                        # 上货点有人占用，已经有去往上货点的合法路径
                        car_back_continue_flag = False
                        for other_car_sn in self.car_sn_list:
                            other_car_physical_status = next((car for car in self.car_physical_status if car.sn == other_car_sn), None)
                            other_car_pos = other_car_physical_status.pos.position
                            if (other_car_sn != car_sn) and \
                                (self.des_pos_reached(other_car_pos, loading_pos, 0.8) or self.des_pos_reached(self.car_paths[other_car_sn][1],loading_pos,0.8)):
                                print(f"发现有其它地勤车辆{other_car_sn}还在上货点或已经产生前往路径，我让它先去！！")
                                car_back_continue_flag = True

                        # 上货点无人占用，让离得近的先去
                        car_back_break_flag = False
                        self_car_and_loading_point_distance = np.hypot(car_pos.x - loading_pos.x, car_pos.y - loading_pos.y)
                        for other_car_sn in self.car_sn_list:
                            if car_back_continue_flag:
                                break
                            if other_car_sn != car_sn:
                                other_car_physical_status = next((car for car in self.car_physical_status if car.sn == other_car_sn), None)
                                other_car_pos = other_car_physical_status.pos.position
                                other_car_and_loading_point_distance = np.hypot(other_car_pos.x - loading_pos.x, other_car_pos.y - loading_pos.y)
                                last_four_digits = other_car_sn[-4:]
                                other_drone_sn = f"SIM-DRONE-{last_four_digits}"
                                other_drone_physical_status = next((drone for drone in self.drone_physical_status if drone.sn == other_drone_sn), None)
                                other_drone_pos = other_drone_physical_status.pos.position
                                if (self.des_pos_reached(other_car_pos, other_drone_pos, 0.8) and
                                    other_drone_physical_status.drone_work_state == DronePhysicalStatus.READY and
                                        other_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY and
                                            self.state_dict[other_car_sn] == WorkState.MOVE_CAR_BACK_TO_LOADING_POINT and
                                                other_car_and_loading_point_distance < self_car_and_loading_point_distance - 2):
                                    print(f"发现有更近的地勤车辆{other_car_sn}需要回家，我让它先回！！")
                                    car_back_break_flag = True
                        if car_back_break_flag:
                            break
                        
                        if car_back_continue_flag is False:
                            if drone_physical_status.remaining_capacity < 30:
                                self.move_car_with_start_and_end(
                                    car_sn, car_pos, loading_pos, WorkState.DRONE_BATTERY_REPLACEMENT)
                            else:
                                self.move_car_with_start_and_end(
                                    car_sn, car_pos, loading_pos, WorkState.MOVE_CARGO_IN_DRONE)
                    else:
                        print(f"无人机{drone_sn}或车辆{car_sn}未就绪，等待...")
                elif state == WorkState.DRONE_BATTERY_REPLACEMENT:
                    if (self.des_pos_reached(loading_pos, car_pos, 0.8) and
                        car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY and
                            drone_physical_status.drone_work_state == DronePhysicalStatus.READY):
                        self.battery_replacement(
                            drone_sn, car_sn, WorkState.MOVE_CARGO_IN_DRONE)
                    else:
                        print(f"无人机{drone_sn}或车辆{car_sn}未就绪，等待...")
                # elif state == WorkState.DRONE_RETRIEVE:  # TODO: 不考虑换绑的话没必要把飞机放回去吧
                #     if (drone_physical_status.drone_work_state == DronePhysicalStatus.READY and
                #         car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                #         self.drone_retrieve(
                #             drone_sn, car_sn, 5.0, WorkState.MOVE_DRONE_ON_CAR)
                #         # 分配下一个订单
                #         self.waybill_index_dict[drone_sn] += 1
                #     else:
                #         print(f"无人机 {drone_sn} 未就绪，等待...")
                rospy.sleep(0.5)
            
            # 自测阶段检查是否已经超过一小时，提交的时候应该注释掉
            if time.time() - start_time > 3600:
                self.state = WorkState.FINISHED
                print("\n运行时间已超过一小时，结束循环")
                print(f"得分：{self.score}")
            else:
                print(f"\n当前运行时间: {time.time() - start_time}\n")


if __name__ == '__main__':
    race_demo = DemoPipeline()
    race_demo.running()