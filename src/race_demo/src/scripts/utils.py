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
