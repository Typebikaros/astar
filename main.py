import matplotlib.pyplot as plt
import numpy as np
import heapq
import time

# 创建200*200的栅格
grid_size = 200
grid = np.zeros((grid_size, grid_size))

# 定义障碍物的形状（封闭图形）
def create_obstacle(shape, start_x, start_y, size):
    if shape == 'circle':
        for i in range(grid_size):
            for j in range(grid_size):
                if (i - start_x) ** 2 + (j - start_y) ** 2 <= size ** 2:
                    grid[i][j] = 1  # 山形，标记为1
    elif shape == 'muddy_road':
        for i in range(start_x - size, start_x + size):
            for j in range(start_y - size, start_y + size):
                if 0 <= i < grid_size and 0 <= j < grid_size:
                    grid[i][j] = 2  # 泥泞道路，标记为2

# 创建圆形和泥泞道路的障碍物
create_obstacle('circle', 30, 30, 20)  # 圆形障碍物（模拟山），蓝色
create_obstacle('muddy_road', 70, 70, 5)  # 泥泞道路障碍物，棕色

# 添加更多障碍物（示例）
for i in range(10):
    create_obstacle('circle', np.random.randint(20, 180), np.random.randint(20, 180), np.random.randint(5, 15))
    create_obstacle('muddy_road', np.random.randint(20, 180), np.random.randint(20, 180), np.random.randint(3, 7))

# 定义启发式函数（曼哈顿距离）
def heuristic(node, goal):
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

# A*算法找最短路径
def astar(start, end):
    open_list = []
    closed_list = set()
    heapq.heappush(open_list, (0, start, []))  # 优先队列中存储节点及其路径成本

    while open_list:
        cost, current_node, path = heapq.heappop(open_list)

        if current_node == end:
            return path + [current_node]

        if current_node not in closed_list:
            closed_list.add(current_node)

            for dx, dy in [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)]:
                new_x, new_y = current_node[0] + dx, current_node[1] + dy
                if 0 <= new_x < grid_size and 0 <= new_y < grid_size and grid[new_x][new_y] != 1:
                    new_cost = cost + 1
                    if grid[new_x][new_y] == 0:
                        new_cost += 1  # 平地消耗较低
                    elif grid[new_x][new_y] == 2:
                        new_cost += 5  # 泥泞道路，消耗较高
                    heapq.heappush(open_list, (new_cost + heuristic((new_x, new_y), end), (new_x, new_y), path + [current_node]))

    return None

# 找到最短路径
start = (5, 5)
end = (grid_size - 5, grid_size - 5)

start_time = time.time()  # 记录开始时间
shortest_path = astar(start, end)
end_time = time.time()  # 记录结束时间
if shortest_path:
    # 绘制最短路径和障碍
    plt.figure(figsize=(8, 8))
    for i in range(grid_size):
        for j in range(grid_size):
            if grid[i][j] == 1:
                plt.scatter(j, i, color='blue')  # 山形，蓝色
            elif grid[i][j] == 2:
                plt.scatter(j, i, color='brown')  # 泥泞道路，棕色
            elif grid[i][j] == 0:
                plt.scatter(j, i, color='white')  # 平地

    for i in range(len(shortest_path) - 1):
        plt.plot([shortest_path[i][1], shortest_path[i + 1][1]], [shortest_path[i][0], shortest_path[i + 1][0]], color='red', linewidth=1)

    plt.scatter(start[1], start[0], color='green', marker='o', s=50, label='起点')
    plt.scatter(end[1], end[0], color='red', marker='o', s=50, label='终点')
    plt.legend()
    plt.title('Different Types of Obstacles with Annotations')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.grid(True)
    plt.savefig('complex_obstacles_with_annotations.png')  # 保存图像
    plt.show()  # 显示图像
else:
    print("No path found between start and end points due to obstacles.")
