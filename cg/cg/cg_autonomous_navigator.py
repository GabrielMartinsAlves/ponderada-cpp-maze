
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from cg_interfaces.srv import GetMap, MoveCmd
import time

class Cell:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    def __hash__(self):
        return hash((self.x, self.y))

class AutonomousNavigator(Node):
    def __init__(self):
        super().__init__('cg_autonomous_navigator')
        self.get_logger().info('Autonomous Navigator node has started!')
        self.move_client = self.create_client(MoveCmd, '/move_command')
        self.get_map_client = self.create_client(GetMap, '/get_map')
        self.timer = self.create_timer(0.5, self.main_loop)
        self.received_map = False
        self.flat_map = []
        self.width = 0
        self.height = 0
        self.robot = None
        self.goal = None
        self.path = []
        self.path_index = 0

    def main_loop(self):
        if not self.received_map:
            self.request_map()
            return
        if not self.path:
            self.path = self.a_star(self.robot, self.goal)
            self.path_index = 1
            if not self.path:
                self.get_logger().error('Nenhum caminho encontrado!')
                rclpy.shutdown()
                return
            self.get_logger().info(f'Caminho calculado com {len(self.path)} passos.')
        if self.path_index < len(self.path):
            self.move_to(self.path[self.path_index])
            self.path_index += 1
        else:
            self.get_logger().info('Robô chegou ao alvo!')
            rclpy.shutdown()

    def request_map(self):
        req = GetMap.Request()
        self.get_logger().info('Tentando conectar ao serviço /get_map...')
        if not self.get_map_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('Aguardando serviço /get_map...')
            return
        future = self.get_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.done():
            resp = future.result()
            self.flat_map = []
            for s in resp.occupancy_grid_flattened:
                if s == 'b': self.flat_map.append(1)
                elif s == 'f': self.flat_map.append(0)
                elif s == 'r': self.flat_map.append(2)
                elif s == 't': self.flat_map.append(3)
                else: self.flat_map.append(0)
            self.height = resp.occupancy_grid_shape[0]
            self.width = resp.occupancy_grid_shape[1]
            for y in range(self.height):
                for x in range(self.width):
                    v = self.flat_map[y * self.width + x]
                    if v == 2:
                        self.robot = Cell(x, y)
                    if v == 3:
                        self.goal = Cell(x, y)
            self.received_map = True
            self.get_logger().info(f'Mapa recebido: {self.width}x{self.height}')
        else:
            self.get_logger().error('Timeout ao aguardar resposta do serviço /get_map.')

    def is_free(self, c):
        if c.x < 0 or c.x >= self.width or c.y < 0 or c.y >= self.height:
            return False
        v = self.flat_map[c.y * self.width + c.x]
        return v == 0 or v == 2 or v == 3

    def neighbors(self, c):
        result = []
        for dx, dy in [(0,1),(1,0),(0,-1),(-1,0)]:
            n = Cell(c.x+dx, c.y+dy)
            if self.is_free(n):
                result.append(n)
        return result

    def a_star(self, start, goal):
        import heapq
        came_from = {}
        cost_so_far = {}
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from[start] = None
        cost_so_far[start] = 0
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                break
            for next_cell in self.neighbors(current):
                new_cost = cost_so_far[current] + 1
                if next_cell not in cost_so_far or new_cost < cost_so_far[next_cell]:
                    cost_so_far[next_cell] = new_cost
                    priority = new_cost + self.heuristic(next_cell, goal)
                    heapq.heappush(open_set, (priority, next_cell))
                    came_from[next_cell] = current
        # Reconstruct path
        path = []
        current = goal
        if current not in came_from:
            return []
        while current:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    def heuristic(self, a, b):
        return abs(a.x - b.x) + abs(a.y - b.y)

    def move_to(self, target):
        dir = self.get_direction(self.robot, target)
        if not dir:
            return
        req = MoveCmd.Request()
        req.direction = dir
        if not self.move_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Aguardando serviço /move_command...')
            return
        future = self.move_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done():
            self.robot = target
        else:
            self.get_logger().warn('Falha ao mover.')

    def get_direction(self, from_cell, to_cell):
        if to_cell.x == from_cell.x and to_cell.y == from_cell.y+1:
            return 'down'
        if to_cell.x == from_cell.x and to_cell.y == from_cell.y-1:
            return 'up'
        if to_cell.x == from_cell.x+1 and to_cell.y == from_cell.y:
            return 'right'
        if to_cell.x == from_cell.x-1 and to_cell.y == from_cell.y:
            return 'left'
        return ''

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNavigator()
    try:
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
