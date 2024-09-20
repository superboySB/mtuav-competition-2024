#!/usr/bin/env python3
import rospy
import json
import numpy as np
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


class DemoPipeline:
    def __init__(self):
        # 初始化ros全局变量
        self.state = WorkState.START
        rospy.init_node('race_demo')
        self.cmd_pub = rospy.Publisher(
            '/cmd_exec', UserCmdRequest, queue_size=10000)
        self.info_sub = rospy.Subscriber(
            '/panoramic_info',
            PanoramicInfo,
            self.panoramic_info_callback,
            queue_size=10)
        self.map_client = rospy.ServiceProxy('query_voxel', QueryVoxel)
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
        self.state = WorkState.TEST_MAP_QUERY

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

    # 移动地面车辆的函数
    # 小车不能设置速度，会按照物理模型的设计尽快到达目的点
    def move_car_with_start_and_end(
            self, car_sn, start, end, time_est, next_state):
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
        self.state = next_state

    # 检测位置到达的函数
    def des_pos_reached(self, des_pos, cur_pos, threshold):
        des = np.array([des_pos.x, des_pos.y, des_pos.z])
        cur = np.array([cur_pos.x, cur_pos.y, cur_pos.z])
        return np.linalg.norm(np.array(des - cur)) < threshold

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
        self.state = next_state

    # 网飞机上挂餐
    def move_cargo_in_drone(self, cargo_id, drone_sn, time_est):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_CARGO_IN_DRONE
        msg.binding_cargo.cargo_id = cargo_id
        msg.binding_cargo.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state = WorkState.MOVE_CAR_TO_LEAVING_POINT

    # 飞机航线飞行函数
    def fly_one_route(self, drone_sn, route, speed, time_est, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_EXEC_ROUTE
        msg.drone_way_point_info.droneSn = drone_sn
        takeoff_point = DroneWayPoint()
        takeoff_point.type = DroneWayPoint.POINT_TAKEOFF
        takeoff_point.timeoutsec = 1000  # 设置一个比实际飞行时间（加速、最大速度匀速、减速）大一些的数字即可
        msg.drone_way_point_info.way_point.append(takeoff_point)
        for waypoint in route:
            middle_point = DroneWayPoint()
            middle_point.type = DroneWayPoint.POINT_FLYING
            middle_point.pos.x = waypoint.x
            middle_point.pos.y = waypoint.y
            middle_point.pos.z = waypoint.z
            middle_point.v = speed
            middle_point.timeoutsec = 1000
            msg.drone_way_point_info.way_point.append(middle_point)
        land_point = DroneWayPoint()
        land_point.type = DroneWayPoint.POINT_LANDING
        land_point.timeoutsec = 1000
        msg.drone_way_point_info.way_point.append(land_point)
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state = next_state

    # 抛餐函数
    def release_cargo(self, drone_sn, time_est, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_RELEASE_CARGO
        msg.drone_msg.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state = next_state

    # 换电函数
    def battery_replacement(self, drone_sn, time_est, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_BATTERY_REPLACEMENT
        msg.drone_msg.drone_sn = drone_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state = next_state

    # 回收飞机函数
    def drone_retrieve(self, drone_sn, car_sn, time_est, next_state):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_DRONE_ON_BIRTHPLACE
        msg.unbind_info.drone_sn = drone_sn
        msg.unbind_info.car_sn = car_sn
        self.cmd_pub.publish(msg)
        rospy.sleep(time_est)
        self.state = next_state
    # 状态流转主函数

    def running(self):
        rospy.sleep(2.0)
        waybill_count = 0
        car_sn = self.car_sn_list[0]
        drone_sn = self.drone_sn_list[0]
        car_physical_status = next(
            (car for car in self.car_physical_status if car.sn == car_sn), None)
        car_init_pos = car_physical_status.pos.position
        while not rospy.is_shutdown() and waybill_count < len(self.waybill_infos):
            waybill = self.waybill_infos[waybill_count]
            car_physical_status = next(
                (car for car in self.car_physical_status if car.sn == car_sn), None)
            car_pos = car_physical_status.pos.position
            drone_physical_status = next(
                (drone for drone in self.drone_physical_status if drone.sn == drone_sn), None)
            drone_pos = drone_physical_status.pos.position
            loading_pos = Position(
                self.loading_cargo_point['x'],
                self.loading_cargo_point['y'],
                self.loading_cargo_point['z'])
            print(self.state)
            if self.state == WorkState.START:
                self.sys_init()
            elif self.state == WorkState.TEST_MAP_QUERY:
                self.test_map_query()
            elif self.state == WorkState.MOVE_CAR_GO_TO_LOADING_POINT:
                self.move_car_with_start_and_end(
                    car_sn, car_pos, loading_pos, 5.0, WorkState.MOVE_DRONE_ON_CAR)
            elif self.state == WorkState.MOVE_DRONE_ON_CAR:
                if(self.des_pos_reached(loading_pos, car_pos, 0.5) and car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                    self.move_drone_on_car(
                        car_sn, drone_sn, 3.0, WorkState.MOVE_CARGO_IN_DRONE)
            elif self.state == WorkState.MOVE_CARGO_IN_DRONE:
                cargo_id = waybill['cargoParam']['index']
                self.move_cargo_in_drone(cargo_id, drone_sn, 10.0)
            elif self.state == WorkState.MOVE_CAR_TO_LEAVING_POINT:
                self.move_car_with_start_and_end(
                    car_sn, car_pos, car_init_pos, 5.0, WorkState.RELEASE_DRONE_OUT)
            elif self.state == WorkState.RELEASE_DRONE_OUT:
                if(self.des_pos_reached(car_init_pos, car_pos, 0.5) and car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                    start_pos = Position(drone_pos.x, drone_pos.y, -120)
                    middle_pos = Position(
                        waybill['targetPosition']['x'], waybill['targetPosition']['y'], -120)
                    end_pos = Position(
                        waybill['targetPosition']['x'],
                        waybill['targetPosition']['y'],
                        waybill['targetPosition']['z'] - 5)
                    route = [start_pos, middle_pos, end_pos]  # 目前航线下发后，就不能够更改了
                    self.fly_one_route(
                        drone_sn, route, 15.0, 10, WorkState.RELEASE_CARGO)
            elif self.state == WorkState.RELEASE_CARGO:
                des_pos = Position(
                    waybill['targetPosition']['x'],
                    waybill['targetPosition']['y'],
                    waybill['targetPosition']['z'])
                if(self.des_pos_reached(des_pos, drone_pos, 2.0) and drone_physical_status.drone_work_state == DronePhysicalStatus.READY):
                    self.release_cargo(
                        drone_sn, 5.0, WorkState.RELEASE_DRONE_RETURN)
            elif self.state == WorkState.RELEASE_DRONE_RETURN:
                des_pos = Position(
                    waybill['targetPosition']['x'],
                    waybill['targetPosition']['y'],
                    waybill['targetPosition']['z'])
                if(self.des_pos_reached(des_pos, drone_pos, 2.0) and drone_physical_status.drone_work_state == DronePhysicalStatus.READY):
                    start_pos = Position(drone_pos.x, drone_pos.y, -120)
                    middle_pos = Position(car_init_pos.x, car_init_pos.y, -120)
                    end_pos = Position(car_pos.x, car_pos.y, car_pos.z - 20)
                    route = [start_pos, middle_pos, end_pos]
                    self.fly_one_route(
                        drone_sn, route, 15.0, 10, WorkState.MOVE_CAR_BACK_TO_LOADING_POINT)
            elif self.state == WorkState.MOVE_CAR_BACK_TO_LOADING_POINT:
                if(self.des_pos_reached(car_pos, drone_pos, 0.8) and drone_physical_status.drone_work_state == DronePhysicalStatus.READY and car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                    self.move_car_with_start_and_end(
                        car_sn, car_pos, loading_pos, 5.0, WorkState.DRONE_BATTERY_REPLACEMENT)
            elif self.state == WorkState.DRONE_BATTERY_REPLACEMENT:
                if(self.des_pos_reached(loading_pos, car_pos, 0.8) and car_physical_status.car_work_state == CarPhysicalStatus.CAR_READY):
                    self.battery_replacement(
                        drone_sn, 10.0, WorkState.DRONE_RETRIEVE)
            elif self.state == WorkState.DRONE_RETRIEVE:
                self.drone_retrieve(
                    drone_sn, car_sn, 5.0, WorkState.MOVE_DRONE_ON_CAR)
                waybill_count += 1
            rospy.sleep(1.0)
        print(
            'Total waybill finished: ',
            waybill_count,
            ', Total score: ',
            self.score)


if __name__ == '__main__':
    race_demo = DemoPipeline()
    race_demo.running()
