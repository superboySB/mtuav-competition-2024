#!/usr/bin/env python3
import rospy
import json
import os
import numpy as np
import heapq  # 导入用于实现优先队列的库
from concurrent.futures import ThreadPoolExecutor, as_completed


from enum import Enum

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

# demo定义的状态流转


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

class Node:
    def __init__(self, x, y, cost, heuristic, parent=None):
        self.x = x
        self.y = y
        self.cost = cost  # 从起点到当前节点的代价
        self.heuristic = heuristic  # 启发式估计值
        self.parent = parent  # 父节点

    def __lt__(self, other):
        return self.cost + self.heuristic < other.cost + other.heuristic

def astar(start, goal, occ_map, x_range, y_range, x_step, y_step):
    open_list = []
    closed_set = set()
    start_node = Node(start[0], start[1], 0, abs(start[0] - goal[0]) + abs(start[1] - goal[1]))
    heapq.heappush(open_list, start_node)
    while open_list:
        current_node = heapq.heappop(open_list)
        if (current_node.x, current_node.y) == goal:
            # 路径找到，回溯获取路径
            path = []
            while current_node:
                path.append((current_node.x, current_node.y))
                current_node = current_node.parent
            return path[::-1]  # 反转路径
        closed_set.add((current_node.x, current_node.y))
        # 生成邻居节点
        for dx, dy in [(-x_step, 0), (x_step, 0), (0, -y_step), (0, y_step)]:
            neighbor_x = current_node.x + dx
            neighbor_y = current_node.y + dy
            if (neighbor_x, neighbor_y) in closed_set:
                continue
            if neighbor_x < x_range[0] or neighbor_x > x_range[1] or neighbor_y < y_range[0] or neighbor_y > y_range[1]:
                continue
            if (neighbor_x, neighbor_y) in occ_map:
                continue  # 避开障碍物
            neighbor_cost = current_node.cost + ((dx**2 + dy**2)**0.5)
            neighbor_heuristic = abs(neighbor_x - goal[0]) + abs(neighbor_y - goal[1])
            neighbor_node = Node(neighbor_x, neighbor_y, neighbor_cost, neighbor_heuristic, current_node)
            heapq.heappush(open_list, neighbor_node)
    return None  # 无法到达目标


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

        # 定义高度层和空域划分
        self.altitude_levels = [-60, -70, -80, -90, -100, -110, -120]
        self.occ_map_dict = {}  # 存储不同高度层的障碍物地图
        self.fast_path_dict = {}  # 存储不同高度层的快速通道
        self.car_destination_dict = {}  # 存储每辆车的目标位置
        self.car_paths = {}  # 存储每辆车的规划路径
        self.car_init_positions = {}  # 记录每个车辆的初始位置

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
        rospy.sleep(10.0)

        # 初始化地图和路径
        if self.need_init:
            self.init_occ_map()
            self.init_fast_paths()
            self.state = WorkState.FINISHED
            print("地图和路径初始化完成")
        else:
            # 从本地文件加载
            if os.path.exists('occ_map_dict.json'):
                with open('occ_map_dict.json', 'r') as f:
                    self.occ_map_dict = json.load(f)
            if os.path.exists('fast_path_dict.json'):
                with open('fast_path_dict.json', 'r') as f:
                    self.fast_path_dict = json.load(f)
            self.state = WorkState.TEST_MAP_QUERY
            print("地图和路径初始化导入成功")

    # 构建障碍物地图
    def init_occ_map(self):
        print("开始构建障碍物地图...")
        x_min = int(self.map_boundary['bottomLeft']['x'])
        x_max = int(self.map_boundary['bottomRight']['x'])
        y_min = int(self.map_boundary['bottomLeft']['y'])
        y_max = int(self.map_boundary['topLeft']['y'])
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
                request = QueryVoxelRequest()
                request.x = x
                request.y = y
                request.z = z
                response = self.map_client(request)
                if response.voxel.semantic == 255:
                    return None  # 超出地图范围
                if response.voxel.distance < 2:
                    return (x, y)
                return None

            # 使用线程池并行处理
            with ThreadPoolExecutor(max_workers=48) as executor:
                futures = [executor.submit(process_point, point) for point in points]
                for future in as_completed(futures):
                    result = future.result()
                    if result is not None:
                        occ_map.append(result)
            self.occ_map_dict[str(z)] = occ_map
        # 保存到本地文件
        with open('occ_map_dict.json', 'w') as f:
            json.dump(self.occ_map_dict, f)
        print("完成构建障碍物地图...")

    def is_direct_path(self, start, end, occ_map):
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

    # 构建快速通道
    def init_fast_paths(self):
        print("开始构建快速通道...")
        x_min = int(self.map_boundary['bottomLeft']['x'])
        x_max = int(self.map_boundary['bottomRight']['x'])
        y_min = int(self.map_boundary['bottomLeft']['y'])
        y_max = int(self.map_boundary['topLeft']['y'])
        x_step = 1  # 采样步长
        y_step = 1

        x_range = (x_min, x_max)
        y_range = (y_min, y_max)

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

            # 添加地图边界上的采样点（每隔一定距离采样一次）
            boundary_sampling_step = 50  # 可以根据地图大小调整
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

            # 预先计算关键点之间的路径
            for i in range(len(key_points)):
                for j in range(i + 1, len(key_points)):
                    start = key_points[i]
                    end = key_points[j]
                    path_key = f"{start}_{end}"
                    # 检查两点之间是否有直线路径
                    if self.is_direct_path(start, end, occ_map):
                        # 如果有直线路径，直接保存
                        paths[path_key] = [start, end]
                    else:
                        # 使用A*算法计算路径
                        path = astar(start, end, occ_map, x_range, y_range, x_step, y_step)
                        if path:
                            paths[path_key] = path
                        else:
                            print(f"无法找到从 {start} 到 {end} 的路径")
            self.fast_path_dict[str(z)] = paths
            print(f"高度 {z} 的预先计算路径数量：{len(paths)}")

        # 保存到本地文件
        with open('fast_path_dict.json', 'w') as f:
            json.dump(self.fast_path_dict, f)
        print("完成构建快速通道...")


    # 测试地图查询接口，可用这个或地图SDK进行航线规划
    # response -> distance 当前体素距离障碍物的最近距离，<=0 的区域代表本身就是障碍物
    # height是当前体素距离地面的高度(所以是整数）， current height是查询点距离地面的高度（所以是小数）
    # 目前semantic=18的范围是危险区域（尽量避开、目前暂不扣分）
    # TODO: 这里可以地图预处理，采样合理的路径反复使用
    def test_map_query(self):
        request = QueryVoxelRequest()
        request.x = 1.0
        request.y = 2.0
        request.z = -3.0
        response = self.map_client(request)
        print(response)
        if response.success:
            self.state = WorkState.MOVE_CAR_GO_TO_LOADING_POINT

    # 检测位置到达的函数
    def des_pos_reached(self, des_pos, cur_pos, threshold):
        des = np.array([des_pos.x, des_pos.y, des_pos.z])
        cur = np.array([cur_pos.x, cur_pos.y, cur_pos.z])
        return np.linalg.norm(des - cur) < threshold
    
    def minimum_distance_between_lines(self, start1, end1, start2, end2):
        """
        计算两条线段在二维平面上的最小距离
        """
        # 将位置转换为numpy数组
        p1 = np.array([start1.x, start1.y])
        p2 = np.array([end1.x, end1.y])
        q1 = np.array([start2.x, start2.y])
        q2 = np.array([end2.x, end2.y])

        # 各线段的向量
        u = p2 - p1
        v = q2 - q1
        w = p1 - q1

        a = np.dot(u, u)
        b = np.dot(u, v)
        c = np.dot(v, v)
        d = np.dot(u, w)
        e = np.dot(v, w)

        D = a * c - b * b
        sc, sN, sD = 0.0, 0.0, D
        tc, tN, tD = 0.0, 0.0, D

        if D < 1e-8:  # 线段近似平行
            sN = 0.0
            sD = 1.0
            tN = e
            tD = c
        else:
            sN = (b * e - c * d)
            tN = (a * e - b * d)
            if sN < 0.0:
                sN = 0.0
            elif sN > sD:
                sN = sD

        if tN < 0.0:
            tN = 0.0
            if -d < 0.0:
                sN = 0.0
            elif -d > a:
                sN = sD
            else:
                sN = -d
                sD = a
        elif tN > tD:
            tN = tD
            if (-d + b) < 0.0:
                sN = 0
            elif (-d + b) > a:
                sN = sD
            else:
                sN = (-d + b)
                sD = a

        sc = 0.0 if abs(sN) < 1e-8 else sN / sD
        tc = 0.0 if abs(tN) < 1e-8 else tN / tD

        # 最近点之间的向量
        dP = w + (sc * u) - (tc * v)
        distance = np.linalg.norm(dP)
        return distance

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
                distance = self.minimum_distance_between_lines(start, end, other_start, other_end)
                if distance < 5.0:
                    print(f"车辆 {car_sn} 的路径与车辆 {other_car_sn} 的路径过近，取消移动")
                    # 不发送移动指令，直接返回
                    return
        # 检查目标位置是否与其他车辆过近
        for other_car in self.car_physical_status:
            if other_car.sn != car_sn:
                other_car_pos = other_car.pos.position
                if self.des_pos_reached(end, other_car_pos, 5.0):
                    print(f"车辆 {car_sn} 的目标位置与车辆 {other_car.sn} 的当前位置过近，取消移动")
                    return
        # 记录车辆的目标位置和路径
        self.car_destination_dict[car_sn] = end
        self.car_paths[car_sn] = (start, end)
        # 发送移动指令
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_CAR_EXEC_ROUTE
        msg.car_route_info.carSn = car_sn
        msg.car_route_info.way_point.append(start)
        msg.car_route_info.way_point.append(end)
        msg.car_route_info.yaw = 0.0  # 小车停车时的角度
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state_dict[car_sn] = next_state
    
    # 往车上挪机 (如果用于一开始飞机与小车绑定的时候，则是飞机从出生点直接瞬移到小车上)
    # 后续飞机和小车不用完全绑定，送飞和接驳可以是两个不同的小车
    def move_drone_on_car(self, car_sn, drone_sn, time_est, next_state):
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
    def fly_one_route(self, drone_sn, start_pos, end_pos, altitude, speed, time_est, next_state):
        # 检查无人机是否处于 READY 状态
        drone_status = next((drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
        if drone_status is None or drone_status.drone_work_state != DronePhysicalStatus.READY:
            print(f"无人机 {drone_sn} 当前不处于 READY 状态，等待...")
            return
        
        altitude_str = str(altitude)
        occ_map = set(map(tuple, self.occ_map_dict[altitude_str]))

        # 将起点和终点转换为整数坐标
        start_key = (int(start_pos.x), int(start_pos.y))
        end_key = (int(end_pos.x), int(end_pos.y))

        # 从关键点集中找到距离起点和终点最近的关键点
        key_points = []
        # 加载关键点集
        paths = self.fast_path_dict[altitude_str]
        for path_key in paths.keys():
            start_point_str, end_point_str = path_key.split('_')
            start_point = eval(start_point_str)
            end_point = eval(end_point_str)
            key_points.extend([start_point, end_point])
        key_points = list(set(key_points))

        # 找到距离起点最近的关键点
        nearest_start_point = min(key_points, key=lambda p: np.hypot(p[0]-start_key[0], p[1]-start_key[1]))
        # 找到距离终点最近的关键点
        nearest_end_point = min(key_points, key=lambda p: np.hypot(p[0]-end_key[0], p[1]-end_key[1]))

        # 构建完整路径
        full_path_coords = []

        # 起点到最近关键点
        if start_key != nearest_start_point:
            if self.is_direct_path(start_key, nearest_start_point, occ_map):
                full_path_coords.extend([start_key, nearest_start_point])
            else:
                # 使用 A* 计算短路径
                path = astar(start_key, nearest_start_point, occ_map, (start_key[0]-50, start_key[0]+50), (start_key[1]-50, start_key[1]+50), 1, 1)
                if path:
                    full_path_coords.extend(path)
                else:
                    print(f"无法找到从 {start_key} 到最近关键点 {nearest_start_point} 的路径")
                    return

        # 关键点之间的预先计算路径
        middle_path_key = f"{nearest_start_point}_{nearest_end_point}"
        if middle_path_key in paths:
            middle_path = paths[middle_path_key]
        else:
            middle_path_key = f"{nearest_end_point}_{nearest_start_point}"
            if middle_path_key in paths:
                middle_path = paths[middle_path_key][::-1]  # 反转路径
            else:
                print(f"无法找到关键点之间的预先计算路径：{nearest_start_point} 到 {nearest_end_point}")
                return
        full_path_coords.extend(middle_path[1:])  # 避免重复添加关键点

        # 最近关键点到终点
        if end_key != nearest_end_point:
            if self.is_direct_path(nearest_end_point, end_key, occ_map):
                full_path_coords.extend([nearest_end_point, end_key])
            else:
                # 使用 A* 计算短路径
                path = astar(nearest_end_point, end_key, occ_map, (end_key[0]-50, end_key[0]+50), (end_key[1]-50, end_key[1]+50), 1, 1)
                if path:
                    full_path_coords.extend(path[1:])  # 避免重复添加关键点
                else:
                    print(f"无法找到从最近关键点 {nearest_end_point} 到终点 {end_key} 的路径")
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
        for coord in full_path_coords:
            middle_point = DroneWayPoint()
            middle_point.type = DroneWayPoint.POINT_FLYING
            middle_point.pos.x = coord[0]
            middle_point.pos.y = coord[1]
            middle_point.pos.z = altitude  # 保持在指定高度
            middle_point.v = speed
            middle_point.timeoutsec = 1000
            msg.drone_way_point_info.way_point.append(middle_point)

        land_point = DroneWayPoint()
        land_point.type = DroneWayPoint.POINT_LANDING
        land_point.timeoutsec = 1000
        msg.drone_way_point_info.way_point.append(land_point)
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state_dict[drone_sn] = next_state


    # 抛餐函数
    def release_cargo(self, drone_sn, time_est, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_RELEASE_CARGO
        msg.drone_msg.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state_dict[drone_sn] = next_state

    # 换电函数
    def battery_replacement(self, drone_sn, time_est, next_state):
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
        self.state_dict[drone_sn] = next_state

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
        waybill_index = 0
        car_num = len(self.car_sn_list)
        drone_num = len(self.drone_sn_list)
        N = min(car_num, drone_num)
        for i in range(N):
            car_sn = self.car_sn_list[i]
            drone_sn = self.drone_sn_list[i]
            self.state_dict[car_sn] = WorkState.START
            self.state_dict[drone_sn] = WorkState.START
            self.waybill_dict[drone_sn] = self.waybill_infos[waybill_index]
            waybill_index += 1
        for car in self.car_physical_status:
            self.car_init_positions[car.sn] = car.pos.position

        while not rospy.is_shutdown() and waybill_index < len(self.waybill_infos):
            for i in range(N):
                car_sn = self.car_sn_list[i]
                drone_sn = self.drone_sn_list[i]
                waybill = self.waybill_dict[drone_sn]

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

                if state == WorkState.START:
                    self.sys_init()
                    self.state_dict[car_sn] = WorkState.TEST_MAP_QUERY
                elif state == WorkState.TEST_MAP_QUERY:
                    self.state_dict[car_sn] = WorkState.MOVE_CAR_GO_TO_LOADING_POINT
                elif state == WorkState.MOVE_CAR_GO_TO_LOADING_POINT:
                    if car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                        self.move_car_with_start_and_end(
                            car_sn, car_pos, loading_pos, 5.0, WorkState.MOVE_DRONE_ON_CAR)
                    else:
                        print(f"车辆 {car_sn} 未就绪，等待...")
                    if self.des_pos_reached(loading_pos, car_pos, 0.5):
                        # 清除车辆的目标位置和路径
                        self.car_destination_dict.pop(car_sn, None)
                        self.car_paths.pop(car_sn, None)
                elif state == WorkState.MOVE_DRONE_ON_CAR:
                    if (self.des_pos_reached(loading_pos, car_pos, 0.5) and
                        car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                        self.move_drone_on_car(
                            car_sn, drone_sn, 3.0, WorkState.MOVE_CARGO_IN_DRONE)
                elif state == WorkState.MOVE_CARGO_IN_DRONE:
                    cargo_id = waybill['cargoParam']['index']
                    self.move_cargo_in_drone(cargo_id, drone_sn, 10.0)
                    self.state_dict[car_sn] = WorkState.MOVE_CAR_TO_LEAVING_POINT
                elif state == WorkState.MOVE_CAR_TO_LEAVING_POINT:
                    if car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY:
                        # 让小车返回自己的出生点
                        car_init_pos = self.car_init_positions[car_sn]
                        self.move_car_with_start_and_end(
                            car_sn, car_pos, car_init_pos, 5.0, WorkState.RELEASE_DRONE_OUT)
                        if self.des_pos_reached(car_init_pos, car_pos, 0.5):
                            # 清除车辆的目标位置和路径
                            self.car_destination_dict.pop(car_sn, None)
                            self.car_paths.pop(car_sn, None)
                    else:
                        print(f"车辆 {car_sn} 未就绪，等待...")

                elif state == WorkState.RELEASE_DRONE_OUT:
                    car_init_pos = self.car_init_positions[car_sn]
                    if (self.des_pos_reached(car_init_pos, car_pos, 0.5) and
                        car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY and
                        drone_physical_status.drone_work_state == DronePhysicalStatus.READY):
                        start_pos = Position(car_pos.x, car_pos.y, car_pos.z)
                        end_station = waybill['targetPosition']
                        end_pos = Position(end_station['x'], end_station['y'], end_station['z'])
                        self.fly_one_route(
                            drone_sn, start_pos, end_pos, altitude, 15.0, 10, WorkState.RELEASE_CARGO)
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
                                drone_sn, 5.0, WorkState.RELEASE_DRONE_RETURN)
                    else:
                        print(f"无人机 {drone_sn} 未就绪，等待...")
                elif state == WorkState.RELEASE_DRONE_RETURN:
                    if drone_physical_status.drone_work_state == DronePhysicalStatus.READY:
                        start_pos = Position(drone_pos.x, drone_pos.y, drone_pos.z)
                        end_pos = Position(car_pos.x, car_pos.y, car_pos.z)
                        self.fly_one_route(
                            drone_sn, start_pos, end_pos, altitude, 15.0, 10, WorkState.MOVE_CAR_BACK_TO_LOADING_POINT)
                    else:
                        print(f"无人机 {drone_sn} 未就绪，等待...")
                elif state == WorkState.MOVE_CAR_BACK_TO_LOADING_POINT:
                    if (self.des_pos_reached(car_pos, drone_pos, 0.8) and
                        drone_physical_status.drone_work_state == DronePhysicalStatus.READY and
                        car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                        self.move_car_with_start_and_end(
                            car_sn, car_pos, loading_pos, 5.0, WorkState.DRONE_BATTERY_REPLACEMENT)
                        if self.des_pos_reached(loading_pos, car_pos, 0.5):
                            # 清除车辆的目标位置和路径
                            self.car_destination_dict.pop(car_sn, None)
                            self.car_paths.pop(car_sn, None)
                    else:
                        print(f"无人机{drone_sn}或车辆{car_sn}未就绪，等待...")
                elif state == WorkState.DRONE_BATTERY_REPLACEMENT:
                    if (self.des_pos_reached(loading_pos, car_pos, 0.8) and
                        car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY and
                        drone_physical_status.drone_work_state == DronePhysicalStatus.READY):
                        self.battery_replacement(
                            drone_sn, 10.0, WorkState.DRONE_RETRIEVE)
                    else:
                        print(f"无人机或车辆未就绪，等待...")
                elif state == WorkState.DRONE_RETRIEVE:
                    if drone_physical_status.drone_work_state == DronePhysicalStatus.READY:
                        self.drone_retrieve(
                            drone_sn, car_sn, 5.0, WorkState.MOVE_DRONE_ON_CAR)
                        # 分配下一个订单
                        if waybill_index < len(self.waybill_infos):
                            self.waybill_dict[drone_sn] = self.waybill_infos[waybill_index]
                            waybill_index += 1
                        else:
                            print("所有订单已完成")
                            self.state_dict[car_sn] = WorkState.FINISHED
                    else:
                        print(f"无人机 {drone_sn} 未就绪，等待...")
            rospy.sleep(1.0)
        print('总订单完成数: ', waybill_index, ', 总得分: ', self.score)


if __name__ == '__main__':
    race_demo = DemoPipeline()
    race_demo.running()
