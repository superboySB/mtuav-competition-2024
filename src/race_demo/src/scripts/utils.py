import heapq  # 导入用于实现优先队列的库
import numpy as np

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

# 在关键点图中寻找最短路径
def dijkstra(graph, start, end):
    import heapq
    queue = []
    heapq.heappush(queue, (0, start, [start]))
    visited = set()
    while queue:
        cost, node, path = heapq.heappop(queue)
        if node == end:
            return path
        if node in visited:
            continue
        visited.add(node)
        for neighbor, weight in graph.get(node, []):
            if neighbor not in visited:
                heapq.heappush(queue, (cost + weight, neighbor, path + [neighbor]))
    return None  # 无法到达终点

def point_to_segment_distance(point, seg_start, seg_end):
    """
    计算点到线段的最短距离
    """
    # 将位置转换为numpy数组
    point = np.array(point)
    seg_start = np.array(seg_start)
    seg_end = np.array(seg_end)
    
    # 线段的向量
    seg_vec = seg_end - seg_start
    point_vec = point - seg_start
    
    seg_len_sq = np.dot(seg_vec, seg_vec)
    
    if seg_len_sq < 1e-8:
        # 线段退化为点
        return np.linalg.norm(point - seg_start)
    
    # 投影参数
    t = np.dot(point_vec, seg_vec) / seg_len_sq
    t = max(0, min(1, t))
    
    # 投影点
    projection = seg_start + t * seg_vec
    distance = np.linalg.norm(point - projection)
    return distance

def minimum_distance_between_lines(start1, end1, start2, end2):
    """
    计算两条线段在二维平面上的最小距离
    
    用法示例：
    # 检查是否会与其他车辆发生路径碰撞
        for other_car_sn, other_path in self.car_paths.items():
            if other_car_sn != car_sn:
                other_start, other_end = other_path
                distance = minimum_distance_between_lines(start, end, other_start, other_end)
                if distance < 3.0:
                    print(f"车辆 {car_sn} 的路径与车辆 {other_car_sn} 的目标路径过近，取消移动")
                    # 不发送移动指令，直接返回
                    return
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

    # 检查线段是否退化为点
    u_len_sq = np.dot(u, u)
    v_len_sq = np.dot(v, v)

    if u_len_sq < 1e-8 and v_len_sq < 1e-8:
        # 两条线段都退化为点
        distance = np.linalg.norm(p1 - q1)
        return distance
    elif u_len_sq < 1e-8:
        # 第一条线段退化为点
        distance = point_to_segment_distance(p1, q1, q2)
        return distance
    elif v_len_sq < 1e-8:
        # 第二条线段退化为点
        distance = point_to_segment_distance(q1, p1, p2)
        return distance

    # 以下是原始计算过程
    a = np.dot(u, u)
    b = np.dot(u, v)
    c = np.dot(v, v)
    d = np.dot(u, w)
    e = np.dot(v, w)

    D = a * c - b * b
    sc, sN, sD = 0.0, D, D
    tc, tN, tD = 0.0, D, D

    if D < 1e-8:
        # 线段近似平行
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
            sN = 0.0
        elif (-d + b) > a:
            sN = sD
        else:
            sN = (-d + b)
            sD = a

    sc = sN / sD if abs(sD) > 1e-8 else 0.0
    tc = tN / tD if abs(tD) > 1e-8 else 0.0

    # 最近点之间的向量
    dP = w + (sc * u) - (tc * v)
    distance = np.linalg.norm(dP)
    return distance

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
