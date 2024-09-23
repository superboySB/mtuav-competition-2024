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


def minimum_distance_between_lines(start1, end1, start2, end2):
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
