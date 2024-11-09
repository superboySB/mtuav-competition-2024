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

from utils import dijkstra, astar, check_path, is_direct_path

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
    # MOVE_CAR_BACK_TO_LOADING_POINT = 10 # 已根据需求删除
    # DRONE_BATTERY_REPLACEMENT = 11  # 已根据需求删除
    DRONE_RETRIEVE = 12
    FINISHED = 13
    MOVE_CAR_TO_DRONE_KEY_POINT = 14
    WAIT_FOR_DRONE_RETURN = 15  # 新增状态
    MOVE_CAR_BACK_TO_DRONE_KEY_POINT = 16  # 新增状态

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

        # 读取配置文件和信息
        with open('/config/config.json', 'r') as file:
            self.config = json.load(file)
        self.drone_infos = self.config['taskParam']['droneParamList']
        self.car_infos = self.config['taskParam']['magvParamList']
        self.loading_cargo_point = self.config['taskParam']['loadingCargoPoint']
        self.loading_cargo_position = Position(x=self.loading_cargo_point["x"], y=self.loading_cargo_point["y"], z=self.loading_cargo_point["z"])
        self.drone_born_point = Position(x=185, y=425, z=self.loading_cargo_point["z"])
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

        # 定义高度层和空域划分
        self.full_altitude_levels = [-115, -105, -95, -85, -75, -65]
        self.occ_map_dict = {}  # 存储不同高度层的障碍物地图
        self.fast_path_dict = {}  # 存储不同高度层的快速通道

        # 固定路径从出生点到关键点
        self.fixed_paths_from_start_to_key_point = {
            "SIM-MAGV-0001": [(183,434), (183,431), (187,431)],
            "SIM-MAGV-0002": [(190,438), (193,438), (193,449)],
            "SIM-MAGV-0003": [(183,446), (181,446), (181,449)],
            "SIM-MAGV-0004": [(197,434), (197,431), (193,431)],
            "SIM-MAGV-0005": [(190,444), (190,449), (187,449)],
            "SIM-MAGV-0006": [(197,446), (199, 446), (199,449)],
        }

        # 固定循环路径
        self.fixed_cycles_from_key_point = {
            "SIM-MAGV-0001": [(187,431), (190,425), (187,431)],
            "SIM-MAGV-0002": [(193,449), (193,443), (193,439), (189,439), (181,439), (181,435), (181,431), (181,425), (185,425), (190,425), (195,425), (199,425), (199,431), (199,435), (199,439), (193,439), (193,443), (193,449)],
            "SIM-MAGV-0003": [(181,449), (181,443), (181,439), (181,435), (181,431), (181,425), (185,425), (190,425), (195,425), (199,425), (199,431), (199,435), (199,439), (193,439), (189,439), (181,439), (181,443), (181,449)],
            "SIM-MAGV-0004": [(193,431), (190,425), (193,431)],
            "SIM-MAGV-0005": [(187,449), (189,443), (189,439), (181,439), (181,435), (181,431), (181,425), (185,425), (190,425), (195,425), (199,425), (199,431), (199,435), (199,439), (193,439), (189,439), (189,443), (187,449)],
            "SIM-MAGV-0006": [(199,449), (199,443), (199,439), (193,439), (189,439), (181,439), (181,435), (181,431), (181,425), (185,425), (190,425), (195,425), (199,425), (199,431), (199,435), (199,439), (199,443), (199,449)],
        }

        # 初始化每个车辆的状态字典
        for car_sn in self.car_sn_list:
            self.car_state_dict[car_sn] = {
                'state': WorkState.START,
                'current_waypoint_index': 0,  # 新增字段用于循环路径
                'ready_for_landing': True, # 做好让飞机降落的准备
                'occupyed_for_landing': False,
            }

        # 初始化每个无人机的状态字典
        for drone_sn in self.drone_sn_list:
            self.drone_state_dict[drone_sn] = {
                'state': WorkState.START,
            }

        # 卸货点及其飞行高度
        self.unloading_points = {
            (146,186): {'delivery_height': -115, 'return_height': -85},
            (528,172): {'delivery_height': -115, 'return_height': -75},
            (564,394): {'delivery_height': -115, 'return_height': -85},
            (430,184): {'delivery_height': -115, 'return_height': -105},
            (490,390): {'delivery_height': -115, 'return_height': -95},
            (508,514): {'delivery_height': -115, 'return_height': -75},
        }

        # 将卸货点映射到车辆
        self.unloading_point_car_map = {
            (564,394): "SIM-MAGV-0002",
            (430,184): "SIM-MAGV-0004",
            (490,390): "SIM-MAGV-0005",
            (508,514): "SIM-MAGV-0006",
            (146,186): "SIM-MAGV-0003",
            (528,172): "SIM-MAGV-0006",
        }

        # 保证每个外卖只能同时被一条航线送
        self.is_delivering_pointed_cargos = {
            (564,394): False,
            (430,184): False,
            (490,390): False,
            (508,514): False,
            (146,186): False,
            (528,172): False
        }

        # 初始化无人机使用状态
        self.drone_usage = {}
        for drone_sn in self.drone_sn_list:
            self.drone_usage[drone_sn] = {
                'available': True,
                'capacity': 100, 
                'current_order': None,
                'current_order_x': -1,
                'current_order_y': -1,
                'go_to_unloading_point': False,
                'wait_for_landing_car': False,
            }

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
            self.occ_map_dict = {}
            self.fast_path_dict = {}
            # 获取需要加载的高度层
            altitudes_to_load = set(self.full_altitude_levels)
            for z in altitudes_to_load:
                occ_map_file = f'/root/mtuav-competition-2024/occ_map_{int(z)}.json'
                fast_path_file = f'/root/mtuav-competition-2024/fast_path_dict_{int(z)}.json'
                if os.path.exists(occ_map_file):
                    with open(occ_map_file, 'r') as f:
                        self.occ_map_dict[str(z)] = json.load(f)
                else:
                    print(f"缺少高度 {z} 的 occ_map 文件，无法加载")
                if os.path.exists(fast_path_file):
                    with open(fast_path_file, 'r') as f:
                        self.fast_path_dict[str(z)] = json.load(f)
                else:
                    print(f"缺少高度 {z} 的 fast_path 文件，无法加载")
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
            # 保存到本地文件（每个高度层一个文件）
            with open(f'/root/mtuav-competition-2024/occ_map_{int(z)}.json', 'w') as f:
                json.dump(occ_map, f)
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
            print(f"构建高度 {z} 的快速通道...")

            # 定义关键点集
            key_points = []

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

            # 保存到本地文件（每个高度层一个文件）
            with open(f'/root/mtuav-competition-2024/fast_path_dict_{int(z)}.json', 'w') as f:
                json.dump(self.fast_path_dict[str(z)], f)
        print("完成构建快速通道...")

    # 检测位置到达的函数
    def des_pos_reached(self, des_pos, cur_pos, threshold):
        des = np.array([des_pos.x, des_pos.y, des_pos.z])
        cur = np.array([cur_pos.x, cur_pos.y, cur_pos.z])
        return np.linalg.norm(des - cur) < threshold

    # 简化的障碍物获取函数
    def get_other_car_occupied_points(self, car_sn):
        occupied_points = set()
        for other_car_sn, other_data in self.car_state_dict.items():
            if other_car_sn == car_sn:
                continue  # 跳过自己
            current_car_physical_status = next(
                (car for car in self.car_physical_status if car.sn == other_car_sn), None)
            car_pos = current_car_physical_status.pos.position
            x = int(round(car_pos.x))
            y = int(round(car_pos.y))
            for dx in range(-3, 4):
                for dy in range(-3, 4):
                    occupied_points.add((x + dx, y + dy))
            if current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_RUNNING:
                # 添加下一路点
                next_waypoint_index = other_data['current_waypoint_index']
                path = self.fixed_cycles_from_key_point[other_car_sn]
                assert path is not None
                next_waypoint = path[next_waypoint_index]
                x = int(round(next_waypoint[0]))
                y = int(round(next_waypoint[1]))
                for dx in range(-3, 4):
                    for dy in range(-3, 4):
                        occupied_points.add((x + dx, y + dy))
        return occupied_points

    # 生成路径，简化了碰撞检测
    def plan_path_avoiding_obstacles(self, car_sn, start_pos, end_pos):
        # 检查下一个路点是否被占用
        occupied_points = self.get_other_car_occupied_points(car_sn)
        x = int(round(end_pos.x))
        y = int(round(end_pos.y))
        if (x,y) in occupied_points:
            return None
        return [start_pos, end_pos]

    # 移动地面车辆的函数
    def move_car_with_start_and_end(self, car_sn, start, end):                    
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_CAR_EXEC_ROUTE
        msg.car_route_info.carSn = car_sn
        msg.car_route_info.way_point.append(start)
        msg.car_route_info.way_point.append(end)
        self.cmd_pub.publish(msg)

        print(f"车辆 {car_sn} 从 ({int(round(start.x))}, {int(round(start.y))}) 移动到 ({int(round(end.x))}, {int(round(end.y))})")

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

    # 回收飞机函数
    def drone_retrieve(self, drone_sn, car_sn):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_DRONE_ON_BIRTHPLACE
        msg.unbind_info.drone_sn = drone_sn
        msg.unbind_info.car_sn = car_sn
        self.cmd_pub.publish(msg)
        
        cmd_pub_start_time = time.time()
        retry_times = 0
        while True:
            current_drone_physical_status = next((drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
            if current_drone_physical_status.drone_work_state == DronePhysicalStatus.READY and \
              self.des_pos_reached(current_drone_physical_status.pos.position,self.drone_born_point,2.0):
                self.drone_usage[drone_sn]['available'] = True
                self.drone_usage[drone_sn]['capacity'] = current_drone_physical_status.remaining_capacity
                print(f"无人机{drone_sn}成功收到指令并回到飞机出生点")
                break
            
            cmd_pub_current_time = time.time()
            if cmd_pub_current_time - cmd_pub_start_time > retry_times + 5:
                print("超时，重试发送")
                self.cmd_pub.publish(msg)
                retry_times += 5

    def select_best_order_when_on_loading_point(self):
        current_time = int(time.time() * 1000)
        available_orders = []
        for bill_status in self.bills_status:
            if bill_status.status != BillStatus.NOT_STARTED:
                continue
            orderTime = bill_status.orderTime
            betterTime = bill_status.betterTime
            timeout = bill_status.timeout
            if current_time < orderTime or current_time + 103000 > betterTime:
                continue  # 订单未开始或已超过最佳送达时间，我不接
            if self.is_delivering_pointed_cargos[(int(bill_status.target_pos.x),int(bill_status.target_pos.y))]:
                continue
            
            available_orders.append((bill_status, orderTime, betterTime, timeout))

            print(f"!!! Current available bill: {bill_status.index}, orderTime:{orderTime}, betterTime:{betterTime}, timeout:{timeout}")
            print(f"current_time = {betterTime-current_time}, order_time - current_time = {orderTime-current_time}, betterTime-current_time = {betterTime-current_time}, timeout-current_time = {timeout - current_time}")

        if available_orders:
            selected_order = max(available_orders, key=lambda x: x[2])
            return selected_order[0]
        else:
            return None

    def select_best_order_when_on_drone_releasing_point(self):
        current_time = int(time.time() * 1000)
        available_orders = []
        for bill_status in self.bills_status:
            if bill_status.status != BillStatus.NOT_STARTED:
                continue
            orderTime = bill_status.orderTime
            betterTime = bill_status.betterTime
            timeout = bill_status.timeout
            if current_time > timeout or current_time + 13000 < orderTime:
                continue
            if self.is_delivering_pointed_cargos[(int(bill_status.target_pos.x),int(bill_status.target_pos.y))]:
                continue
            if current_time + 115000 > betterTime:
                continue

            available_orders.append((bill_status, orderTime, betterTime, timeout))

            print(f"!!! Future available or current easy bill: {bill_status.index}, orderTime:{orderTime}, betterTime:{betterTime}, timeout:{timeout}")
            print(f"current_time = {betterTime-current_time}, order_time - current_time = {orderTime-current_time}, betterTime-current_time = {betterTime-current_time}, timeout-current_time = {timeout - current_time}")

        if available_orders:
            return True
        else:
            print("现在你过去取货也没前途呀，先别着急去了！！")
            return False
        
    def move_drone_on_car(self, drone_sn, car_sn):
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
                self.drone_usage[drone_sn]['available'] = False
                break

            cmd_pub_current_time = time.time()
            if cmd_pub_current_time - cmd_pub_start_time > retry_times + 5:
                print("超时，重试发送")
                self.cmd_pub.publish(msg)
                retry_times += 5

    def move_cargo_in_drone(self, car_sn, drone_sn, next_state):
        # 选择最佳订单
        waybill = self.select_best_order_when_on_loading_point()
        if waybill is None:
            print(f"无人机 {drone_sn} 无可用订单，等待...")
            return

        # 打印订单信息
        cargo_id = waybill.index
        orderTime = waybill.orderTime
        betterTime = waybill.betterTime
        timeout = waybill.timeout
        
        current_time = int(time.time() * 1000)
        print(f"无人机 {drone_sn} 接单：订单ID {cargo_id}, orderTime {orderTime}, betterTime {betterTime}, timeout {timeout}")
        print(f"当前时间 {current_time}, 距离orderTime {current_time - orderTime}, 距离betterTime {betterTime - current_time}")

        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_CARGO_IN_DRONE
        msg.binding_cargo.cargo_id = cargo_id
        msg.binding_cargo.drone_sn = drone_sn
        self.drone_usage[drone_sn]['current_order'] = cargo_id
        self.drone_usage[drone_sn]['current_order_x'] = waybill.target_pos.x
        self.drone_usage[drone_sn]['current_order_y'] = waybill.target_pos.y
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
        self.is_delivering_pointed_cargos[(int(waybill.target_pos.x),int(waybill.target_pos.y))] = True

    # 移动无人机的函数，增加碰撞检测和路径优化
    def fly_one_route(self, drone_sn, start_pos, end_pos, altitude, speed):
        # 检查无人机是否处于 READY 状态
        drone_status = next((drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
        if drone_status is None or drone_status.drone_work_state != DronePhysicalStatus.READY:
            print(f"无人机 {drone_sn} 当前不处于 READY 状态，等待...")
            return

        altitude_str = str(altitude)
        occ_map = set(map(tuple, self.occ_map_dict[altitude_str]))

        # 将起点和终点转换为整数坐标和字符串形式
        start_key = (int(round(start_pos.x)), int(round(start_pos.y)))
        end_key = (int(round(end_pos.x)), int(round(end_pos.y)))
        start_key_str = f"{start_key[0]}_{start_key[1]}"
        end_key_str = f"{end_key[0]}_{end_key[1]}"

        # 加载关键点集和图
        fast_path_data = self.fast_path_dict[altitude_str]
        key_points_str = fast_path_data['key_points']
        graph = fast_path_data['graph']

        key_points = [tuple(map(int, p_str.split('_'))) for p_str in key_points_str]

        # 尝试直接飞行
        if is_direct_path(start_key, end_key, occ_map):
            full_path_coords = [start_key, end_key]
        else:
            # 找到距离起点最近的关键点
            nearest_start_point = min(key_points, key=lambda p: np.hypot(p[0]-start_key[0], p[1]-start_key[1]))
            nearest_start_point_str = f"{nearest_start_point[0]}_{nearest_start_point[1]}"
            # 找到距离终点最近的关键点
            nearest_end_point = min(key_points, key=lambda p: np.hypot(p[0]-end_key[0], p[1]-end_key[1]))
            nearest_end_point_str = f"{nearest_end_point[0]}_{nearest_end_point[1]}"

            # 构建完整路径
            full_path_coords = []

            # 起点到最近关键点
            if is_direct_path(start_key, nearest_start_point, occ_map):
                full_path_coords.extend([start_key, nearest_start_point])
            else:
                # 使用 A* 计算路径
                path = astar(start_key, nearest_start_point, occ_map,
                            (min(start_key[0], nearest_start_point[0]) - 50, max(start_key[0], nearest_start_point[0]) + 50),
                            (min(start_key[1], nearest_start_point[1]) - 50, max(start_key[1], nearest_start_point[1]) + 50),
                            1, 1)
                if path:
                    full_path_coords.extend(path)
                else:
                    print(f"无法找到从 {start_key} 到最近关键点 {nearest_start_point} 的路径")
                    return

            # 使用预计算的快速路径
            key_point_path_str = dijkstra(graph, nearest_start_point_str, nearest_end_point_str)
            if key_point_path_str:
                # 将字符串形式的点转换回坐标
                key_point_path = [tuple(map(int, p_str.split('_'))) for p_str in key_point_path_str]
                full_path_coords.extend(key_point_path)
            else:
                print(f"无法找到关键点之间的路径：{nearest_start_point} 到 {nearest_end_point}")
                return

            # 最近关键点到终点
            if is_direct_path(nearest_end_point, end_key, occ_map):
                full_path_coords.extend([nearest_end_point, end_key])
            else:
                # 使用 A* 计算路径
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

        # 添加起飞点
        takeoff_point = DroneWayPoint()
        takeoff_point.type = DroneWayPoint.POINT_TAKEOFF
        takeoff_point.timeoutsec = 1000
        print(f"Takeoff Point: x={start_pos.x}, y={start_pos.y}, z={start_pos.z}")
        msg.drone_way_point_info.way_point.append(takeoff_point)

        # 添加飞行路径点
        # TODO： 针对start_pos为（146,186）、end_pos为进行一些特判吧。
        special_unloading_point_1 = Position(x=146, y=186, z=-34)
        special_unloading_point_2 = Position(x=528, y=172, z=-20)
        if self.des_pos_reached(start_pos, special_unloading_point_1, 2.0) or self.des_pos_reached(start_pos, special_unloading_point_2, 2.0):
            procressed_full_path_coords = [full_path_coords[0], (end_pos.x, end_pos.y)]
        else:
            procressed_full_path_coords = full_path_coords

        for i, coord in enumerate(procressed_full_path_coords):
            middle_point = DroneWayPoint()
            middle_point.type = DroneWayPoint.POINT_FLYING
            middle_point.pos.x = coord[0]
            middle_point.pos.y = coord[1]
            middle_point.pos.z = altitude
            middle_point.v = speed
            middle_point.timeoutsec = 1000
            print(f"Middle Point {i}: x={middle_point.pos.x}, y={middle_point.pos.y}, z={middle_point.pos.z}")
            msg.drone_way_point_info.way_point.append(middle_point)

            if i == 0 and self.des_pos_reached(start_pos, special_unloading_point_1, 2.0):
                middle_point = DroneWayPoint()
                middle_point.type = DroneWayPoint.POINT_FLYING
                middle_point.pos.x = 120
                middle_point.pos.y = 350
                middle_point.pos.z = altitude
                middle_point.v = speed
                middle_point.timeoutsec = 1000
                print(f"Special Middle Point {i}: x={middle_point.pos.x}, y={middle_point.pos.y}, z={middle_point.pos.z}")
                msg.drone_way_point_info.way_point.append(middle_point)
            
            if i == 0 and self.des_pos_reached(start_pos, special_unloading_point_2, 2.0):
                middle_point = DroneWayPoint()
                middle_point.type = DroneWayPoint.POINT_FLYING
                middle_point.pos.x = 590
                middle_point.pos.y = 394
                middle_point.pos.z = altitude
                middle_point.v = speed
                middle_point.timeoutsec = 1000
                print(f"Special Middle Point {i}: x={middle_point.pos.x}, y={middle_point.pos.y}, z={middle_point.pos.z}")
                msg.drone_way_point_info.way_point.append(middle_point)

                middle_point = DroneWayPoint()
                middle_point.type = DroneWayPoint.POINT_FLYING
                middle_point.pos.x = 590
                middle_point.pos.y = 450
                middle_point.pos.z = altitude
                middle_point.v = speed
                middle_point.timeoutsec = 1000
                print(f"Special Middle Point {i}: x={middle_point.pos.x}, y={middle_point.pos.y}, z={middle_point.pos.z}")
                msg.drone_way_point_info.way_point.append(middle_point)

                middle_point = DroneWayPoint()
                middle_point.type = DroneWayPoint.POINT_FLYING
                middle_point.pos.x = 508
                middle_point.pos.y = 530
                middle_point.pos.z = altitude
                middle_point.v = speed
                middle_point.timeoutsec = 1000
                print(f"Special Middle Point {i}: x={middle_point.pos.x}, y={middle_point.pos.y}, z={middle_point.pos.z}")
                msg.drone_way_point_info.way_point.append(middle_point)

                middle_point = DroneWayPoint()
                middle_point.type = DroneWayPoint.POINT_FLYING
                middle_point.pos.x = 400
                middle_point.pos.y = 530
                middle_point.pos.z = altitude
                middle_point.v = speed
                middle_point.timeoutsec = 1000
                print(f"Special Middle Point {i}: x={middle_point.pos.x}, y={middle_point.pos.y}, z={middle_point.pos.z}")
                msg.drone_way_point_info.way_point.append(middle_point)
        
        middle_point = DroneWayPoint()
        middle_point.type = DroneWayPoint.POINT_FLYING
        middle_point.pos.x = end_pos.x
        middle_point.pos.y = end_pos.y
        middle_point.pos.z = end_pos.z
        middle_point.v = speed
        middle_point.timeoutsec = 1000
        print(f"Land Point {i}: x={end_pos.x}, y={end_pos.y}, z={end_pos.z}")
        msg.drone_way_point_info.way_point.append(middle_point)
        
        # 添加降落点
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

    # 修改release_cargo函数
    def release_cargo(self, cargo_id, drone_sn):
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
                    print("提前送达，100分")
                elif betterTime < deliveryFinishTime <= timeout:
                    print(f"按时送达，不到100分")
                else:
                    print(f"超时送达，抠分")
                break

            cmd_pub_current_time = time.time()
            if cmd_pub_current_time - cmd_pub_start_time > retry_times + 5:
                print("超时，重试发送")
                self.cmd_pub.publish(msg)
                retry_times += 5

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

        self.sys_init()

        while not rospy.is_shutdown() and self.state != WorkState.FINISHED:
            # ----------------------------------------------------------------------------------------
            # 处理SIM-MAGV-0001发射车
            car_sn = "SIM-MAGV-0001"
            car_data = self.car_state_dict[car_sn]
            current_car_physical_status = next(
                (car for car in self.car_physical_status if car.sn == car_sn), None)
            release_car_pos = current_car_physical_status.pos.position
            state = car_data['state']

            print(f"正在处理放飞无人车{car_sn}, 位置：{release_car_pos.x}, {release_car_pos.y},{release_car_pos.z}, 小车物理状态：{current_car_physical_status.car_work_state}, 小车逻辑状态：{state}, 小车上飞机: {current_car_physical_status.drone_sn}")
            
            if state == WorkState.START:
                # 移动到关键点
                self.car_state_dict[car_sn]['state'] = WorkState.MOVE_CAR_TO_DRONE_KEY_POINT
                self.car_state_dict[car_sn]['current_waypoint_index'] = 0
            elif state == WorkState.MOVE_CAR_TO_DRONE_KEY_POINT:
                waypoint_index = self.car_state_dict[car_sn]['current_waypoint_index']
                next_waypoint = self.fixed_paths_from_start_to_key_point[car_sn][waypoint_index]
                end_pos = Position(x=next_waypoint[0], y=next_waypoint[1], z=release_car_pos.z)
                if self.des_pos_reached(release_car_pos, end_pos, 2.0) and current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                    if waypoint_index + 1 == len(self.fixed_paths_from_start_to_key_point[car_sn]):
                        self.car_state_dict[car_sn]['state'] = WorkState.MOVE_CAR_GO_TO_LOADING_POINT
                        self.car_state_dict[car_sn]['current_waypoint_index'] = 0
                        continue
                    self.car_state_dict[car_sn]['current_waypoint_index'] = waypoint_index + 1
                    waypoint_index = self.car_state_dict[car_sn]['current_waypoint_index']
                    next_waypoint = self.fixed_paths_from_start_to_key_point[car_sn][waypoint_index]
                    end_pos = Position(x=next_waypoint[0], y=next_waypoint[1], z=release_car_pos.z)
                    self.move_car_with_start_and_end(car_sn, release_car_pos, end_pos)
                else:
                    print(f"车辆 {car_sn} 正在从起点移动到key point，请等待...")
            elif state == WorkState.MOVE_CAR_GO_TO_LOADING_POINT:
                # 沿循环路径移动
                path = self.fixed_cycles_from_key_point[car_sn]
                waypoint_index = self.car_state_dict[car_sn]['current_waypoint_index']
                next_waypoint = path[waypoint_index]
                end_pos = Position(x=next_waypoint[0], y=next_waypoint[1], z=release_car_pos.z)
                if self.des_pos_reached(release_car_pos, end_pos, 2.0) and current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                    if self.des_pos_reached(release_car_pos, self.loading_cargo_position, 2.0):
                        self.car_state_dict[car_sn]['state'] = WorkState.MOVE_DRONE_ON_CAR
                        continue
                    next_waypoint = self.fixed_cycles_from_key_point[car_sn][waypoint_index + 1]
                    end_pos = Position(x=next_waypoint[0], y=next_waypoint[1], z=release_car_pos.z)
                    path_result = self.plan_path_avoiding_obstacles(car_sn, release_car_pos, end_pos)

                    if path_result:
                        # TODO：起飞和接驳协同trick2：让飞机先查外卖再去loading point
                        car_need_hurry_load_cargo_flag = self.select_best_order_when_on_drone_releasing_point()

                        # TODO: 起飞和接驳协同trick1：先让还飞机的走吧，增加更多取外卖的选择
                        car_wait_center_pos = Position(x=183, y=425, z=self.loading_cargo_position.z)
                        car_wait_flag = False
                        for other_car_sn, _ in self.car_state_dict.items():
                            if other_car_sn == car_sn:
                                continue  # 跳过自己
                            other_car_physical_status = next(
                                (car for car in self.car_physical_status if car.sn == other_car_sn), None)
                            other_car_pos = other_car_physical_status.pos.position
                            if self.des_pos_reached(other_car_pos, car_wait_center_pos, 3.0):
                                car_wait_flag = True
                                break

                        if car_need_hurry_load_cargo_flag:
                            print("现在你可以马上过去取货，很有前途！！")
                            self.car_state_dict[car_sn]['current_waypoint_index'] = waypoint_index + 1
                            self.move_car_with_start_and_end(car_sn, release_car_pos, end_pos)
                        
                        if car_wait_flag:
                            print("有其它接驳车在等待，先看他们把小飞机还了吧，反正现在也没有好货")

                    else:
                        print(f"车辆 {car_sn} 的目标点 ({end_pos.x}, {end_pos.y}) 被其他车辆占用")

                else:
                    print(f"车辆 {car_sn} 正在从key point移动到loading point，请等待...")
            elif state == WorkState.MOVE_DRONE_ON_CAR:
                # 选择一个可用的无人机
                available_drones = [drone_sn for drone_sn, usage in self.drone_usage.items() if (usage['available'] and usage['capacity']>30)]
                assert available_drones
                drone_sn = available_drones[0]
                self.move_drone_on_car(drone_sn,car_sn)
                self.car_state_dict[car_sn]['state'] = WorkState.MOVE_CARGO_IN_DRONE
            elif state == WorkState.MOVE_CARGO_IN_DRONE:
                # 装载货物
                drone_sn = current_car_physical_status.drone_sn
                self.move_cargo_in_drone(car_sn, drone_sn, WorkState.MOVE_CAR_TO_LEAVING_POINT)
            elif state == WorkState.MOVE_CAR_TO_LEAVING_POINT:
                drone_sn = current_car_physical_status.drone_sn
                assert drone_sn
                current_drone_physical_status = next(
                    (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                # 这个条件没准还是有初赛的问题
                if current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY and \
                        current_drone_physical_status.drone_work_state == DronePhysicalStatus.READY:
                    waypoint_index = self.car_state_dict[car_sn]['current_waypoint_index']
                    next_waypoint = self.fixed_cycles_from_key_point[car_sn][waypoint_index + 1]
                    end_pos = Position(x=next_waypoint[0], y=next_waypoint[1], z=release_car_pos.z)
                    self.car_state_dict[car_sn]['current_waypoint_index'] = waypoint_index + 1
                    self.move_car_with_start_and_end(car_sn, release_car_pos, end_pos)
                    self.car_state_dict[car_sn]['state'] = WorkState.RELEASE_DRONE_OUT
                else:
                    print(f"车辆 {car_sn} 上的飞机还没装好货，等待...")

            elif state == WorkState.RELEASE_DRONE_OUT:
                path = self.fixed_cycles_from_key_point[car_sn]
                waypoint_index = self.car_state_dict[car_sn]['current_waypoint_index']
                next_waypoint = path[waypoint_index]
                end_pos = Position(x=next_waypoint[0], y=next_waypoint[1], z=release_car_pos.z)
                if self.des_pos_reached(release_car_pos, end_pos, 2.0) and current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                    self.car_state_dict[car_sn]['current_waypoint_index'] = 0
                    drone_sn = current_car_physical_status.drone_sn
                    assert drone_sn
                    current_drone_physical_status = next(
                        (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                    cargo_id = current_drone_physical_status.bind_cargo_id
                    waybill = next((wb for wb in self.bills_status if wb.index == cargo_id), None)
                    assert waybill is not None
                    target_pos = waybill.target_pos
                    end_pos = Position(x=target_pos.x, y=target_pos.y, z=target_pos.z-5)
                    altitude = self.unloading_points[(int(round(target_pos.x)), int(round(target_pos.y)))]['delivery_height']
                    self.fly_one_route(drone_sn, release_car_pos, end_pos, altitude, 15.0)
                    self.car_state_dict[car_sn]['state'] = WorkState.MOVE_CAR_GO_TO_LOADING_POINT
                else:
                    print(f"车辆 {car_sn} 还没移动到起飞点，等待...")
            
            # ----------------------------------------------------------------------------------------
            # 处理接收车
            for car_sn in ["SIM-MAGV-0002", "SIM-MAGV-0003", "SIM-MAGV-0004", "SIM-MAGV-0005", "SIM-MAGV-0006"]:
                car_data = self.car_state_dict[car_sn]
                current_car_physical_status = next(
                    (car for car in self.car_physical_status if car.sn == car_sn), None)
                car_pos = current_car_physical_status.pos.position
                state = car_data['state']

                print(f"正在处理接驳无人车{car_sn}, 位置：{current_car_physical_status.pos.position.x}, {current_car_physical_status.pos.position.y},{current_car_physical_status.pos.position.z}, 小车物理状态：{current_car_physical_status.car_work_state}, 小车逻辑状态：{state}, 小车上飞机: {current_car_physical_status.drone_sn}")

                if state == WorkState.START:
                    # 移动到关键点
                    self.car_state_dict[car_sn]['state'] = WorkState.MOVE_CAR_TO_DRONE_KEY_POINT
                    self.car_state_dict[car_sn]['current_waypoint_index'] = 0
                elif state == WorkState.MOVE_CAR_TO_DRONE_KEY_POINT:
                    waypoint_index = self.car_state_dict[car_sn]['current_waypoint_index']
                    next_waypoint = self.fixed_paths_from_start_to_key_point[car_sn][waypoint_index]
                    end_pos = Position(x=next_waypoint[0], y=next_waypoint[1], z=car_pos.z)
                    if self.des_pos_reached(car_pos, end_pos, 2.0) and current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                        if waypoint_index + 1 == len(self.fixed_paths_from_start_to_key_point[car_sn]):
                            self.car_state_dict[car_sn]['state'] = WorkState.WAIT_FOR_DRONE_RETURN
                            self.car_state_dict[car_sn]['current_waypoint_index'] = 0
                            continue
                        self.car_state_dict[car_sn]['current_waypoint_index'] = waypoint_index + 1
                        waypoint_index = self.car_state_dict[car_sn]['current_waypoint_index']
                        next_waypoint = self.fixed_paths_from_start_to_key_point[car_sn][waypoint_index]
                        end_pos = Position(x=next_waypoint[0], y=next_waypoint[1], z=car_pos.z)
                        self.move_car_with_start_and_end(car_sn, car_pos, end_pos)
                    else:
                        print(f"车辆 {car_sn} 正在从起点移动到key point，请等待...")

                elif state == WorkState.WAIT_FOR_DRONE_RETURN:
                    # 检查是否有无人机降落
                    if current_car_physical_status.drone_sn and current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                        # 开始沿循环路径移动
                        self.car_state_dict[car_sn]['state'] = WorkState.MOVE_CAR_GO_TO_LOADING_POINT
                        self.car_state_dict[car_sn]['current_waypoint_index'] = 0
                        self.car_state_dict[car_sn]['ready_for_landing'] = False
                        self.car_state_dict[car_sn]["occupyed_for_landing"] = False
                        print(f"车辆 {car_sn} 接驳到了无人机 {current_car_physical_status.drone_sn}")
                        continue
                    else:
                        print(f"车辆 {car_sn} 等待无人机返回")
                elif state == WorkState.MOVE_CAR_GO_TO_LOADING_POINT:
                    # 沿循环路径移动
                    path = self.fixed_cycles_from_key_point[car_sn]
                    waypoint_index = self.car_state_dict[car_sn]['current_waypoint_index']
                    next_waypoint = path[waypoint_index]
                    end_pos = Position(x=next_waypoint[0], y=next_waypoint[1], z=car_pos.z)
                    if self.des_pos_reached(car_pos, end_pos, 2.0) and current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                        if self.des_pos_reached(car_pos, self.loading_cargo_position, 2.0):
                            # 到达加载点，解绑无人机
                            drone_sn = current_car_physical_status.drone_sn
                            assert len(drone_sn) > 0
                            self.drone_retrieve(drone_sn, car_sn)
                            self.car_state_dict[car_sn]['state'] = WorkState.MOVE_CAR_BACK_TO_DRONE_KEY_POINT
                            continue
                        next_waypoint = self.fixed_cycles_from_key_point[car_sn][waypoint_index + 1]
                        end_pos = Position(x=next_waypoint[0], y=next_waypoint[1], z=car_pos.z)
                        path_result = self.plan_path_avoiding_obstacles(car_sn, car_pos, end_pos)
                        car_need_hurry_load_cargo_flag = self.select_best_order_when_on_drone_releasing_point()

                        drone_release_pos = Position(x=187, y=431, z=self.loading_cargo_position.z)
                        if self.des_pos_reached(end_pos, self.loading_cargo_position, 2.0) and self.des_pos_reached(release_car_pos, drone_release_pos, 2.0) \
                                and car_need_hurry_load_cargo_flag:
                            print("对不起，你得等着还飞机，要让放飞车现在马上过去取货，更加有前途！！")
                            continue

                        if path_result:
                            self.car_state_dict[car_sn]['current_waypoint_index'] = waypoint_index + 1
                            self.move_car_with_start_and_end(car_sn, car_pos, end_pos)
                        else:
                            print(f"车辆 {car_sn} 的目标点 ({end_pos.x}, {end_pos.y}) 被其他车辆占用")
                    else:
                        print(f"车辆 {car_sn} 正在从key point移动到loading point，请等待...")
                elif state == WorkState.MOVE_CAR_BACK_TO_DRONE_KEY_POINT:
                    # 返回关键点
                    path = self.fixed_cycles_from_key_point[car_sn]
                    waypoint_index = self.car_state_dict[car_sn]['current_waypoint_index']
                    next_waypoint = path[waypoint_index]
                    end_pos = Position(x=next_waypoint[0], y=next_waypoint[1], z=car_pos.z)
                    if self.des_pos_reached(car_pos, end_pos, 2.0) and current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                        # TODO：同时决定小车和飞机之间有没有提前量，同时决定下一个飞机要不要起飞，这可能是safety和performance的性能取舍关键点
                        if car_sn == "SIM-MAGV-0003":
                            ready_steps = 9
                        elif car_sn == "SIM-MAGV-0005":
                            ready_steps = 17
                        elif car_sn == "SIM-MAGV-0002":
                            ready_steps = 19
                        else:
                            ready_steps = 21
                        if self.car_state_dict[car_sn]['current_waypoint_index'] + ready_steps > len(self.fixed_cycles_from_key_point[car_sn]):
                            self.car_state_dict[car_sn]['ready_for_landing'] = True

                        if self.car_state_dict[car_sn]['current_waypoint_index'] + 1 == len(self.fixed_cycles_from_key_point[car_sn]):
                            self.car_state_dict[car_sn]['current_waypoint_index'] = 0
                            self.car_state_dict[car_sn]['state'] = WorkState.WAIT_FOR_DRONE_RETURN
                            print(f"无人车{car_sn}成功返回到key point等待飞机着陆")
                            continue     
                        next_waypoint = self.fixed_cycles_from_key_point[car_sn][waypoint_index + 1]
                        end_pos = Position(x=next_waypoint[0], y=next_waypoint[1], z=car_pos.z)
                        path_result = self.plan_path_avoiding_obstacles(car_sn, car_pos, end_pos)
                        if path_result:
                            self.car_state_dict[car_sn]['current_waypoint_index'] = waypoint_index + 1
                            self.move_car_with_start_and_end(car_sn, car_pos, end_pos)
                        else:
                            print(f"车辆 {car_sn} 的目标点 ({end_pos.x}, {end_pos.y}) 被其他车辆占用")
                    else:
                        print(f"车辆 {car_sn} 正在从loading point回到key point，请等待...")

            # ----------------------------------------------------------------------------------------
            # 处理无人机的释放和返回
            # 检查无人机是否需要释放货物或返回
            prior_drone_is_waiting_flag = False
            for drone_sn, usage in self.drone_usage.items():
                current_drone_physical_status = next((drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                if usage['wait_for_landing_car'] and \
                  (current_drone_physical_status.drone_work_state == DronePhysicalStatus.READY or current_drone_physical_status.drone_work_state == DronePhysicalStatus.LANDING) and \
                  int(usage['current_order_x']) == 508 and int(usage['current_order_y']) == 514:
                    prior_drone_is_waiting_flag = True

            for drone_sn, usage in self.drone_usage.items():
                current_drone_physical_status = next((drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                print(f"正在处理无人机{drone_sn}, 位置：{current_drone_physical_status.pos.position.x}, {current_drone_physical_status.pos.position.y},{current_drone_physical_status.pos.position.z}, 无人机物理状态：{current_drone_physical_status.drone_work_state}")

                if usage['available']:
                    print(f"无人机{drone_sn}正在出生点躺尸")
                    continue
                
                if usage['current_order'] and usage['go_to_unloading_point'] and \
                  current_drone_physical_status.drone_work_state == DronePhysicalStatus.READY:
                    print(f"无人机{drone_sn}落地了")
                    # 释放货物
                    cargo_id = usage['current_order']
                    self.release_cargo(cargo_id, drone_sn)
                    usage['wait_for_landing_car'] = True
                    usage['go_to_unloading_point'] = False

                if usage['current_order'] and current_drone_physical_status.drone_work_state == DronePhysicalStatus.FLYING:
                    usage['go_to_unloading_point'] = True
                    print(f"无人机{drone_sn}正在飞行到卸货点")

                if usage['wait_for_landing_car'] and current_drone_physical_status.drone_work_state == DronePhysicalStatus.READY:
                    # 无人机返回
                    landing_car_sn = self.unloading_point_car_map[(int(round(current_drone_physical_status.pos.position.x)), int(round(current_drone_physical_status.pos.position.y)))]
    
                    if self.car_state_dict[landing_car_sn]["ready_for_landing"] and (not self.car_state_dict[landing_car_sn]["occupyed_for_landing"]):
                        if landing_car_sn == "SIM-MAGV-0006" and int(usage['current_order_x']) == 528 and int(usage['current_order_y']) == 172 and \
                          prior_drone_is_waiting_flag:
                            continue

                        print(f"现在接驳车{landing_car_sn}上没有飞机了，并且快要赶回来了，可以让无人机先回来了")
                        end_pos = Position(self.fixed_cycles_from_key_point[landing_car_sn][0][0], self.fixed_cycles_from_key_point[landing_car_sn][0][1], self.loading_cargo_point["z"]-5)
                        altitude = self.unloading_points[(int(round(current_drone_physical_status.pos.position.x)), int(round(current_drone_physical_status.pos.position.y)))]['return_height']
                        self.fly_one_route(drone_sn, current_drone_physical_status.pos.position, end_pos, altitude, 15.0)
                        usage['wait_for_landing_car'] = False
                        usage['current_order'] = None
                        usage['current_order_x'] = -1
                        usage['current_order_y'] = -1
                        self.car_state_dict[landing_car_sn]["occupyed_for_landing"] = True
                        self.is_delivering_pointed_cargos[(int(round(current_drone_physical_status.pos.position.x)), int(round(current_drone_physical_status.pos.position.y)))] = False
                    else:
                        print(f"现在接驳车{landing_car_sn}正在被其它飞机占用返航，或者是还没有来得及回到key point")

                if (not usage['current_order']) and current_drone_physical_status.drone_work_state == DronePhysicalStatus.FLYING:
                    print(f"无人机{drone_sn}正在飞行到接驳车")

            # ----------------------------------------------------------------------------------------
            print("--------------------------------------------------------------")
            print(f"当前用时{time.time() - start_time}秒, 当前得分：{self.score}")
            print("--------------------------------------------------------------")
            rospy.sleep(0.2)

            # 检查是否已经超过一小时
            if time.time() - start_time > 3700:
                self.state = WorkState.FINISHED
                print("\n运行时间已远远超过一小时，结束循环")

if __name__ == '__main__':
    race_demo = DemoPipeline()
    race_demo.running()
