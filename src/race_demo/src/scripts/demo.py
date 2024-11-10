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
    DRONE_RETRIEVE = 10
    FINISHED = 11
    MOVE_CAR_TO_DRONE_KEY_POINT = 12
    WAIT_FOR_DRONE_RETURN = 13 
    MOVE_CAR_BACK_TO_DRONE_KEY_POINT = 14

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

        # 读取配置文件和信息
        with open('/config/config.json', 'r') as file:
            self.config = json.load(file)
        self.drone_infos = self.config['taskParam']['droneParamList']
        self.car_infos = self.config['taskParam']['magvParamList']
        self.loading_cargo_point = self.config['taskParam']['loadingCargoPoint']
        self.loading_cargo_position = Position(x=self.loading_cargo_point["x"], y=self.loading_cargo_point["y"], z=self.loading_cargo_point["z"])
        self.drone_release_pos = Position(x=190, y=431, z=self.loading_cargo_position.z)
        self.drone_born_point = Position(x=185, y=425, z=self.loading_cargo_point["z"])
        self.buwei_point = Position(x=199, y=435, z=self.loading_cargo_point["z"])
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
        self.key_point_state_dict = {}

        # 定义高度层和空域划分
        self.full_altitude_levels = [-115, -105, -95, -85, -75, -65]

        # 固定路径从出生点到关键点
        self.fixed_paths_from_start_to_key_point = {
            "SIM-MAGV-0001": [(183,434), (183,431), (190,431)],
            "SIM-MAGV-0002": [(190,438), (193,438), (193,449)],
            "SIM-MAGV-0003": [(183,446), (181,446), (181,449)],
            "SIM-MAGV-0004": [(197,434), (199,434), (199,435)],
            "SIM-MAGV-0005": [(190,444), (190,449), (187,449)],
            "SIM-MAGV-0006": [(197,446), (199, 446), (199,449)],
        }

        # 固定循环路径
        # (199,431) 的小车用于服务大家，积极补位
        self.fixed_paths_from_key_point_to_buwei_point= {
            (181,449): [(181,449), (181,443), (181,439), (181,435), (181,431), (181,425), (185,425), (190,425), (195,425), (199,425), (199,431), (199,435)],
            (187,449): [(187,449), (189,443), (189,439), (181,439), (181,435), (181,431), (181,425), (185,425), (190,425), (195,425), (199,425), (199,431), (199,435)],
            (193,449): [(193,449), (193,443), (193,439), (189,439), (181,439), (181,435), (181,431), (181,425), (185,425), (190,425), (195,425), (199,425), (199,431), (199,435)],
            (199,449): [(199,449), (199,443), (199,439), (193,439), (189,439), (181,439), (181,435), (181,431), (181,425), (185,425), (190,425), (195,425), (199,425), (199,431), (199,435)],
        }

        self.fixed_paths_from_buwei_point_to_key_point = {
            (181,449): [(199,435), (199,439), (193,439), (189,439), (181,439), (181,443), (181,449)],
            (187,449): [(199,435), (199,439), (193,439), (189,439), (189,443), (187,449)],
            (193,449): [(199,435), (199,439), (193,439), (193,443), (193,449)],
            (199,449): [(199,435), (199,439), (199,443), (199,449)],
        }

        self.fixed_cycles_from_key_point = {
            "SIM-MAGV-0001": [(190,431), (190,425), (190,431)],
        }


        # 初始化每个车辆的状态字典
        # route: "", "from_key_point_to_buwei_point", "from_buwei_point_to_key_point"
        for car_sn in self.car_sn_list:
            if car_sn == "SIM-MAGV-0003":
                current_key_point = (181,449)
                path_state = "from_key_point_to_buwei_point"
            elif car_sn == "SIM-MAGV-0005":
                current_key_point = (187,449)
                path_state = "from_key_point_to_buwei_point"
            elif car_sn == "SIM-MAGV-0002":
                current_key_point = (193,449)
                path_state = "from_key_point_to_buwei_point"
            elif car_sn == "SIM-MAGV-0006":
                current_key_point = (199,449)
                path_state = "from_key_point_to_buwei_point"
            elif car_sn == "SIM-MAGV-0004":
                current_key_point = (-1,-1)
                path_state = "from_buwei_point_to_key_point"
            else:
                current_key_point = (-1,-1)
                path_state = ""

            self.car_state_dict[car_sn] = {
                'state': WorkState.START,
                'current_waypoint_index': 0,  # 新增字段用于循环路径
                "current_key_point": current_key_point,
                "path_state": path_state,
            }

        # 初始化每个无人机的状态字典
        for drone_sn in self.drone_sn_list:
            self.drone_state_dict[drone_sn] = {
                'state': WorkState.START,
                'available': True,
                'capacity': 100, 
                'current_order': None,
                'current_order_x': -1,
                'current_order_y': -1,
                'go_to_unloading_point': False,
                'wait_for_landing_car': False,
            }
        
        # keypoint的状态机
        for keypoint_sn in [(181,449),(187,449),(193,449),(199,449)]:
            self.key_point_state_dict[keypoint_sn] = {
                'ready_for_landing': True, # 做好让飞机降落的准备
                'occupyed_for_landing': False, # 正在有飞机降落占用这个降落点
            }

        # 卸货点及其飞行高度
        self.unloading_points = {
            (146,186): {'delivery_height': -115, 'return_height': -85},
            (528,172): {'delivery_height': -115, 'return_height': -85},
            (564,394): {'delivery_height': -115, 'return_height': -85},
            (430,184): {'delivery_height': -115, 'return_height': -105},
            (490,390): {'delivery_height': -115, 'return_height': -95},
            (508,514): {'delivery_height': -115, 'return_height': -75},
        }

        # 将卸货点映射到车辆
        self.unloading_point_to_key_point_map = {
            (146,186): (181,449),
            (430,184): (187,449),
            (490,390): (187,449),
            (564,394): (193,449),
            (528,172): (193,449),
            (508,514): (199,449),
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

        # 单点一开始应该可以无脑送两次
        self.initial_double_flag = {
            (564,394): False,
            (430,184): False,
            (490,390): False,
            (508,514): True,
            (146,186): True,
            (528,172): False
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
                current_key_point = other_data['current_key_point']
                path_state = other_data['path_state']
                if path_state == "from_key_point_to_buwei_point":
                    path = self.fixed_paths_from_key_point_to_buwei_point[current_key_point]
                elif path_state == "from_buwei_point_to_key_point":
                    path = self.fixed_paths_from_buwei_point_to_key_point[current_key_point]
                else:
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
                self.drone_state_dict[drone_sn]['available'] = True
                self.drone_state_dict[drone_sn]['capacity'] = current_drone_physical_status.remaining_capacity
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
            if current_time < orderTime or current_time + 100000 > betterTime:
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
            if current_time > timeout or current_time + 10000 < orderTime:
                continue
            if self.is_delivering_pointed_cargos[(int(bill_status.target_pos.x),int(bill_status.target_pos.y))]:
                continue
            if current_time + 110000 > betterTime:
                continue

            available_orders.append((bill_status, orderTime, betterTime, timeout))

            print(f"!!! Future available or current easy bill: {bill_status.index}, orderTime:{orderTime}, betterTime:{betterTime}, timeout:{timeout}")
            print(f"current_time = {betterTime-current_time}, order_time - current_time = {orderTime-current_time}, betterTime-current_time = {betterTime-current_time}, timeout-current_time = {timeout - current_time}")

        if available_orders:
            return True
        else:
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
                self.drone_state_dict[drone_sn]['available'] = False
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
        self.drone_state_dict[drone_sn]['current_order'] = cargo_id
        self.drone_state_dict[drone_sn]['current_order_x'] = waybill.target_pos.x
        self.drone_state_dict[drone_sn]['current_order_y'] = waybill.target_pos.y
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
        
        if self.initial_double_flag[(int(waybill.target_pos.x),int(waybill.target_pos.y))]:
            self.initial_double_flag[(int(waybill.target_pos.x),int(waybill.target_pos.y))] = False
        else:
            self.is_delivering_pointed_cargos[(int(waybill.target_pos.x),int(waybill.target_pos.y))] = True

    # 移动无人机的函数，增加碰撞检测和路径优化
    def fly_one_route(self, drone_sn, start_pos, end_pos, altitude, speed):
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
        # 针对不能直达的pos加一些特判吧。
        special_unloading_point_1 = Position(x=146, y=186, z=-34)
        special_unloading_point_2 = Position(x=528, y=172, z=-20)
        full_path_coords = [(start_pos.x,start_pos.y),(end_pos.x,end_pos.y)]

        for i, coord in enumerate(full_path_coords):
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
                # 还是不能加，太难受了
                # int_target_pos = (int(round(bill_status.target_pos.x)), int(round(bill_status.target_pos.y)))
                # if int_target_pos in [(146,186),(508,514)]:
                #     self.is_delivering_pointed_cargos[int_target_pos] = False

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
                car_data['state'] = WorkState.MOVE_CAR_TO_DRONE_KEY_POINT
                car_data['current_waypoint_index'] = 0
            elif state == WorkState.MOVE_CAR_TO_DRONE_KEY_POINT:
                waypoint_index = car_data['current_waypoint_index']
                next_waypoint = self.fixed_paths_from_start_to_key_point[car_sn][waypoint_index]
                end_pos = Position(x=next_waypoint[0], y=next_waypoint[1], z=release_car_pos.z)
                if self.des_pos_reached(release_car_pos, end_pos, 2.0) and current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                    if waypoint_index + 1 == len(self.fixed_paths_from_start_to_key_point[car_sn]):
                        car_data['state'] = WorkState.MOVE_CAR_GO_TO_LOADING_POINT
                        car_data['current_waypoint_index'] = 0
                        continue
                    car_data['current_waypoint_index'] = waypoint_index + 1
                    waypoint_index = car_data['current_waypoint_index']
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
                available_drones = [drone_sn for drone_sn, usage in self.drone_state_dict.items() if (usage['available'] and usage['capacity']>30)]
                assert available_drones
                drone_sn = available_drones[0]
                self.move_drone_on_car(drone_sn,car_sn)
                car_data['state'] = WorkState.MOVE_CARGO_IN_DRONE
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
            # 处理4+1接收车 (TODO: 地面动态补位+接驳)
            for keypoint_sn, keypoint_dict in self.key_point_state_dict.items():
                print("----------------")
                print(f"正在观察keypoint{keypoint_sn}, ready_for_landing: {keypoint_dict['ready_for_landing']}, occupyed_for_landing: {keypoint_dict['occupyed_for_landing']}")
                print("----------------")

            for car_sn in ["SIM-MAGV-0002", "SIM-MAGV-0003", "SIM-MAGV-0004", "SIM-MAGV-0005", "SIM-MAGV-0006"]:
                car_data = self.car_state_dict[car_sn]
                current_car_physical_status = next(
                    (car for car in self.car_physical_status if car.sn == car_sn), None)
                car_pos = current_car_physical_status.pos.position
                state = car_data['state']
                current_key_point = car_data['current_key_point']
                path_state = car_data['path_state']

                print(f"正在处理接驳无人车{car_sn}, 位置：{current_car_physical_status.pos.position.x}, {current_car_physical_status.pos.position.y},{current_car_physical_status.pos.position.z}, 小车物理状态：{current_car_physical_status.car_work_state}, 小车逻辑状态：{state}, 小车上飞机: {current_car_physical_status.drone_sn}")
                print(f"小车当前current_key_point: {current_key_point}, path_state: {path_state}, current_waypoint_index: {car_data['current_waypoint_index']}")

                if state == WorkState.START:
                    car_data['state'] = WorkState.MOVE_CAR_TO_DRONE_KEY_POINT
                    car_data['current_waypoint_index'] = 0
                elif state == WorkState.MOVE_CAR_TO_DRONE_KEY_POINT:
                    waypoint_index = car_data['current_waypoint_index']
                    next_waypoint = self.fixed_paths_from_start_to_key_point[car_sn][waypoint_index]
                    end_pos = Position(x=next_waypoint[0], y=next_waypoint[1], z=car_pos.z)
                    if self.des_pos_reached(car_pos, end_pos, 2.0) and current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                        if waypoint_index + 1 == len(self.fixed_paths_from_start_to_key_point[car_sn]):
                            car_data['state'] = WorkState.WAIT_FOR_DRONE_RETURN if path_state == "from_key_point_to_buwei_point" else WorkState.MOVE_CAR_BACK_TO_DRONE_KEY_POINT
                            car_data['current_waypoint_index'] = 0
                            continue
                        car_data['current_waypoint_index'] = waypoint_index + 1
                        waypoint_index = car_data['current_waypoint_index']
                        next_waypoint = self.fixed_paths_from_start_to_key_point[car_sn][waypoint_index]
                        end_pos = Position(x=next_waypoint[0], y=next_waypoint[1], z=car_pos.z)
                        self.move_car_with_start_and_end(car_sn, car_pos, end_pos)
                    else:
                        print(f"车辆 {car_sn} 正在从起点移动到key point，请等待...")

                elif state == WorkState.WAIT_FOR_DRONE_RETURN:
                    # 检查是否有无人机降落
                    if current_car_physical_status.drone_sn and current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                        # 开始沿循环路径移动
                        car_data['state'] = WorkState.MOVE_CAR_GO_TO_LOADING_POINT
                        car_data['current_waypoint_index'] = 0
                        self.key_point_state_dict[current_key_point]['occupyed_for_landing'] = False
                        self.key_point_state_dict[current_key_point]['ready_for_landing'] = False
                        print(f"车辆 {car_sn} 接驳到了无人机 {current_car_physical_status.drone_sn}")
                    else:
                        print(f"车辆 {car_sn} 等待无人机返回")
                elif state == WorkState.MOVE_CAR_GO_TO_LOADING_POINT:
                    # 沿循环路径移动
                    assert path_state == "from_key_point_to_buwei_point"
                    assert current_key_point[0] != -1
                    path = self.fixed_paths_from_key_point_to_buwei_point[current_key_point]
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
                        next_waypoint = path[waypoint_index + 1]
                        end_pos = Position(x=next_waypoint[0], y=next_waypoint[1], z=car_pos.z)
                        path_result = self.plan_path_avoiding_obstacles(car_sn, car_pos, end_pos)
                        car_need_hurry_load_cargo_flag = self.select_best_order_when_on_drone_releasing_point()

                        if self.des_pos_reached(end_pos, self.loading_cargo_position, 2.0) and self.des_pos_reached(release_car_pos, self.drone_release_pos, 2.0) \
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
                    if path_state == "from_key_point_to_buwei_point":
                        path = self.fixed_paths_from_key_point_to_buwei_point[current_key_point]
                    elif path_state == "from_buwei_point_to_key_point":
                        if current_key_point[0] == -1:
                            wait_flag = True
                            for keypoint_sn, keypoint_dict in self.key_point_state_dict.items():
                                if keypoint_dict['ready_for_landing'] or keypoint_dict['occupyed_for_landing']:
                                    continue

                                for other_car_sn, other_data in self.car_state_dict.items():
                                    if other_car_sn == car_sn or other_car_sn =="SIM-MAGV-0001":
                                        continue  # 跳过自己
                                    other_next_waypoint_index = other_data['current_waypoint_index']
                                    other_current_key_point = other_data['current_key_point']
                                    other_path_state = other_data['path_state']
                                    if other_path_state == "from_key_point_to_buwei_point" and other_current_key_point == keypoint_sn and \
                                            other_next_waypoint_index > 3:
                                        wait_flag = False
                                        self.car_state_dict[car_sn]['current_key_point'] = keypoint_sn
                                        print(f"地面车辆{car_sn}领到补位接驳点{keypoint_sn}的任务!!!")
                                        self.key_point_state_dict[other_current_key_point]['ready_for_landing'] = True
                                        break
                                
                                if not wait_flag:
                                    break

                            if wait_flag:
                                print("原地等待分配补位任务")
                                continue
                        else:
                            path = self.fixed_paths_from_buwei_point_to_key_point[current_key_point]
                    else:
                        path = None

                    assert path is not None

                    waypoint_index = self.car_state_dict[car_sn]['current_waypoint_index']
                    next_waypoint = path[waypoint_index]
                    end_pos = Position(x=next_waypoint[0], y=next_waypoint[1], z=car_pos.z)
                    if self.des_pos_reached(car_pos, end_pos, 2.0) and current_car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                        if self.car_state_dict[car_sn]['current_waypoint_index'] + 1 == len(path):
                            self.car_state_dict[car_sn]['current_waypoint_index'] = 0
                            if path_state == "from_buwei_point_to_key_point":
                                self.car_state_dict[car_sn]['path_state'] = "from_key_point_to_buwei_point"
                                self.car_state_dict[car_sn]['state'] = WorkState.WAIT_FOR_DRONE_RETURN
                                print(f"无人车{car_sn}成功到达key point({end_pos.x},{end_pos.y})等待飞机着陆")
                            else:
                                self.car_state_dict[car_sn]['path_state'] = "from_buwei_point_to_key_point"
                                my_keypoint_dict = self.key_point_state_dict[current_key_point]
                                my_keypoint_occupy_flag = False
                                for other_car_sn, other_data in self.car_state_dict.items():
                                    if other_car_sn == car_sn or other_car_sn =="SIM-MAGV-0001":
                                        continue  # 跳过自己
                                    other_current_key_point = other_data['current_key_point']
                                    other_path_state = other_data['path_state']
                                    if other_path_state == "from_key_point_to_buwei_point" and other_current_key_point == current_key_point:
                                        my_keypoint_occupy_flag = True
                                        break

                                if my_keypoint_dict['ready_for_landing'] or my_keypoint_dict['occupyed_for_landing'] or my_keypoint_occupy_flag:
                                    self.car_state_dict[car_sn]['current_key_point'] = (-1,-1)
                                else:
                                    self.car_state_dict[car_sn]['current_key_point'] = current_key_point
                                    self.key_point_state_dict[current_key_point]['ready_for_landing'] = True
                            continue
                                
                        next_waypoint = path[waypoint_index + 1]
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
            # 检查无人机是否需要释放货物或返回
            prior_drone_is_waiting_564_394_flag = False
            for drone_sn, usage in self.drone_state_dict.items():
                current_drone_physical_status = next((drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                if usage['wait_for_landing_car'] and \
                  (current_drone_physical_status.drone_work_state == DronePhysicalStatus.READY or current_drone_physical_status.drone_work_state == DronePhysicalStatus.LANDING) and \
                  int(usage['current_order_x']) == 564 and int(usage['current_order_y']) == 394:
                    prior_drone_is_waiting_564_394_flag = True
            
            prior_drone_is_waiting_490_390_flag = False
            for drone_sn, usage in self.drone_state_dict.items():
                current_drone_physical_status = next((drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
                if usage['wait_for_landing_car'] and \
                  (current_drone_physical_status.drone_work_state == DronePhysicalStatus.READY or current_drone_physical_status.drone_work_state == DronePhysicalStatus.LANDING) and \
                  int(usage['current_order_x']) == 490 and int(usage['current_order_y']) == 390:
                    prior_drone_is_waiting_490_390_flag = True

            for drone_sn, usage in self.drone_state_dict.items():
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
                    print(f"无人机{drone_sn}正在飞行到货物{usage['current_order']}的卸货点({usage['current_order_x']},{usage['current_order_y']})")

                if usage['wait_for_landing_car'] and current_drone_physical_status.drone_work_state == DronePhysicalStatus.READY:
                    # 无人机返回
                    landing_key_point = self.unloading_point_to_key_point_map[(int(round(current_drone_physical_status.pos.position.x)), int(round(current_drone_physical_status.pos.position.y)))]
    
                    if self.key_point_state_dict[landing_key_point]["ready_for_landing"] and (not self.key_point_state_dict[landing_key_point]["occupyed_for_landing"]):
                        if landing_key_point == (193,449) and prior_drone_is_waiting_564_394_flag and \
                                int(usage['current_order_x']) == 528 and int(usage['current_order_y']) == 172:
                            continue

                        if landing_key_point == (187,449) and prior_drone_is_waiting_490_390_flag and \
                                int(usage['current_order_x']) == 430 and int(usage['current_order_y']) == 184:
                            continue

                        print(f"现在接驳点{landing_key_point}上已经有接驳车或者已经有正在补位的接驳车，可以让无人机先回来了")
                        end_pos = Position(landing_key_point[0], landing_key_point[1], self.loading_cargo_point["z"]-5)
                        altitude = self.unloading_points[(int(round(current_drone_physical_status.pos.position.x)), int(round(current_drone_physical_status.pos.position.y)))]['return_height']
                        self.fly_one_route(drone_sn, current_drone_physical_status.pos.position, end_pos, altitude, 15.0)
                        usage['wait_for_landing_car'] = False
                        usage['current_order'] = None
                        usage['current_order_x'] = -1
                        usage['current_order_y'] = -1
                        self.key_point_state_dict[landing_key_point]["occupyed_for_landing"] = True
                        self.is_delivering_pointed_cargos[(int(round(current_drone_physical_status.pos.position.x)), int(round(current_drone_physical_status.pos.position.y)))] = False
                    else:
                        print(f"飞机还不能返航，有两个可能原因：现在接驳点{landing_key_point}正在被其它飞机占用返航，或者是接驳点上还没有任何接驳车和补位车")

                if (not usage['current_order']) and current_drone_physical_status.drone_work_state == DronePhysicalStatus.FLYING:
                    print(f"无人机{drone_sn}正在返航回接驳点")

            # ----------------------------------------------------------------------------------------
            print("--------------------------------------------------------------")
            print(f"当前用时{time.time() - start_time}秒, 当前得分：{self.score}")
            print("--------------------------------------------------------------")
            rospy.sleep(0.3)

            # 检查是否已经超过一小时
            if time.time() - start_time > 3700:
                self.state = WorkState.FINISHED
                print("\n运行时间已远远超过一小时，结束循环")

if __name__ == '__main__':
    race_demo = DemoPipeline()
    race_demo.running()
