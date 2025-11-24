// cg_autonomous_navigator.cpp
// Navegação autônoma no labirinto usando BFS (mapeamento) e A* (busca de caminho)
// ROS2 node
/*
Como usar:
1. Compile o projeto normalmente com colcon build.
2. Inicie o simulador do labirinto:
   ros2 run cg maze
3. Em outro terminal, execute este nó:
   ros2 run cg cg_autonomous_navigator
O nó irá:
 - Obter o mapa completo via serviço /get_map
 - Calcular o melhor caminho até o alvo usando A*
 - Movimentar o robô automaticamente até o alvo
 - (Opcional) Para mapeamento exploratório, altere o método para usar BFS e construir o mapa durante a navegação
*/

#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/get_map.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <cg_interfaces/msg/robot_sensors.hpp>
#include <vector>
#include <queue>
#include <unordered_map>
#include <set>
#include <algorithm>
#include <memory>
#include <string>
#include <iostream>

using namespace std;


struct Cell {
    int x, y;
    bool operator==(const Cell& other) const { return x == other.x && y == other.y; }
    bool operator<(const Cell& other) const {
        return (x < other.x) || (x == other.x && y < other.y);
    }
};

namespace std {
    template<>
    struct hash<Cell> {
        size_t operator()(const Cell& c) const {
            return hash<int>()(c.x) ^ hash<int>()(c.y);
        }
    };
}

class AutonomousNavigator : public rclcpp::Node {
public:
    AutonomousNavigator() : Node("cg_autonomous_navigator") {
        move_client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        get_map_client_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");
        sensor_sub_ = this->create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors", 10,
            std::bind(&AutonomousNavigator::sensor_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&AutonomousNavigator::main_loop, this));
        received_map_ = false;
        waiting_map_ = false;
        mapping_ = false;
        map_unknown_ = true;
        // Inicializa mapa parcial com células desconhecidas (-1)
        partial_map_.resize(30 * 30, -1); // Supondo tamanho máximo 30x30
    }

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr get_map_client_;
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool waiting_map_;
    bool mapping_;
    bool map_unknown_;
    std::vector<int> partial_map_; // -1 desconhecido, 0 livre, 1 parede, 2 robô, 3 alvo

    vector<int> flat_map_;
    int width_ = 0, height_ = 0;
    Cell robot_, goal_;
    bool received_map_;
    vector<Cell> path_;
    size_t path_index_ = 0;

    void main_loop() {
        if (!received_map_ && !waiting_map_) {
            request_map();
            return;
        }
        if (map_unknown_) {
            // Se mapa incompleto, inicia mapeamento
            if (!mapping_) {
                RCLCPP_INFO(get_logger(), "Iniciando mapeamento exploratório...");
                mapping_ = true;
                // BFS: adiciona posição inicial do robô à fila de exploração
                bfs_queue_.push(robot_);
                explored_.insert(robot_);
            }
            if (!bfs_queue_.empty()) {
                Cell next = bfs_queue_.front(); bfs_queue_.pop();
                move_to(next);
            } else if (is_map_complete()) {
                RCLCPP_INFO(get_logger(), "Mapeamento completo! Iniciando navegação.");
                map_unknown_ = false;
                mapping_ = false;
            }
            return;
        }
        if (path_.empty()) {
            path_ = a_star(robot_, goal_);
            path_index_ = 1; // 0 é a posição inicial
            if (path_.empty()) {
                RCLCPP_ERROR(get_logger(), "Nenhum caminho encontrado!");
                rclcpp::shutdown();
                return;
            }
            RCLCPP_INFO(get_logger(), "Caminho calculado com %zu passos.", path_.size());
        }
        if (path_index_ < path_.size()) {
            move_to(path_[path_index_]);
            path_index_++;
        } else {
            RCLCPP_INFO(get_logger(), "Robô chegou ao alvo!");
            rclcpp::shutdown();
        }
    }

    void request_map() {
        auto req = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        RCLCPP_INFO(get_logger(), "Tentando conectar ao serviço /get_map...");
        if (!get_map_client_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_WARN(get_logger(), "Aguardando serviço /get_map...");
            return;
        }
        RCLCPP_INFO(get_logger(), "Serviço /get_map disponível, enviando request...");
        waiting_map_ = true;
        get_map_client_->async_send_request(req,
            std::bind(&AutonomousNavigator::map_response_callback, this, std::placeholders::_1));
    }

    void map_response_callback(rclcpp::Client<cg_interfaces::srv::GetMap>::SharedFuture future) {
        auto resp = future.get();
        flat_map_.clear();
        for (const auto& s : resp->occupancy_grid_flattened) {
            if (s == "b") flat_map_.push_back(1);      // parede
            else if (s == "f") flat_map_.push_back(0); // livre
            else if (s == "r") flat_map_.push_back(2); // robô
            else if (s == "t") flat_map_.push_back(3); // alvo
            else flat_map_.push_back(-1); // desconhecido
        }
        height_ = resp->occupancy_grid_shape[0];
        width_ = resp->occupancy_grid_shape[1];
        // Copia para mapa parcial
        partial_map_ = flat_map_;
        // Verifica se há células desconhecidas
        map_unknown_ = std::find(partial_map_.begin(), partial_map_.end(), -1) != partial_map_.end();
        // Encontrar posições do robô (2) e alvo (3)
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                int v = partial_map_[y * width_ + x];
                if (v == 2) {
                    robot_.x = x;
                    robot_.y = y;
                }
                if (v == 3) {
                    goal_.x = x;
                    goal_.y = y;
                }
            }
        }
        received_map_ = true;
        waiting_map_ = false;
        RCLCPP_INFO(get_logger(), "Mapa recebido: %dx%d", width_, height_);
    }

    void sensor_callback(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
        // Atualiza células adjacentes no mapa parcial
        std::vector<std::pair<std::string, int>> dirs = {
            {"up", 0}, {"down", 0}, {"left", 0}, {"right", 0},
            {"up_left", 0}, {"up_right", 0}, {"down_left", 0}, {"down_right", 0}
        };
        std::map<std::string, std::pair<int,int>> dir_map = {
            {"up", {0,-1}}, {"down", {0,1}}, {"left", {-1,0}}, {"right", {1,0}},
            {"up_left", {-1,-1}}, {"up_right", {1,-1}}, {"down_left", {-1,1}}, {"down_right", {1,1}}
        };
        for (auto& d : dir_map) {
            int nx = robot_.x + d.second.first;
            int ny = robot_.y + d.second.second;
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                int idx = ny * width_ + nx;
                std::string val;
                if (d.first == "up") val = msg->up;
                else if (d.first == "down") val = msg->down;
                else if (d.first == "left") val = msg->left;
                else if (d.first == "right") val = msg->right;
                else if (d.first == "up_left") val = msg->up_left;
                else if (d.first == "up_right") val = msg->up_right;
                else if (d.first == "down_left") val = msg->down_left;
                else if (d.first == "down_right") val = msg->down_right;
                if (val == "b") partial_map_[idx] = 1;
                else if (val == "f") partial_map_[idx] = 0;
                else if (val == "t") partial_map_[idx] = 3;
            }
        }
    }

    bool is_map_complete() {
        return std::find(partial_map_.begin(), partial_map_.end(), -1) == partial_map_.end();
    }

    std::queue<Cell> bfs_queue_;
    std::set<Cell> explored_;

    bool is_free(const Cell& c) {
        if (c.x < 0 || c.x >= width_ || c.y < 0 || c.y >= height_) return false;
        int v = flat_map_[c.y * width_ + c.x];
        // 0 = livre, 1 = parede, 2 = robô, 3 = alvo
        return v == 0 || v == 2 || v == 3;
    }

    vector<Cell> neighbors(const Cell& c) {
        vector<Cell> result;
        for (auto [dx, dy] : vector<pair<int,int>>{{0,1},{1,0},{0,-1},{-1,0}}) {
            Cell n{c.x+dx, c.y+dy};
            if (is_free(n)) result.push_back(n);
        }
        return result;
    }

    // Algoritmo A*
    vector<Cell> a_star(Cell start, Cell goal) {
        unordered_map<Cell, Cell> came_from;
        unordered_map<Cell, int> cost_so_far;
        auto cmp = [](pair<int, Cell> a, pair<int, Cell> b) { return a.first > b.first; };
        priority_queue<pair<int, Cell>, vector<pair<int, Cell>>, decltype(cmp)> open(cmp);
        open.push({0, start});
        came_from[start] = start;
        cost_so_far[start] = 0;
        while (!open.empty()) {
            auto [_, current] = open.top(); open.pop();
            if (current == goal) break;
            for (auto& next : neighbors(current)) {
                int new_cost = cost_so_far[current] + 1;
                if (!cost_so_far.count(next) || new_cost < cost_so_far[next]) {
                    cost_so_far[next] = new_cost;
                    int priority = new_cost + heuristic(next, goal);
                    open.push({priority, next});
                    came_from[next] = current;
                }
            }
        }
        // Reconstruir caminho
        vector<Cell> path;
        Cell current = goal;
        if (!came_from.count(goal)) return path; // Falha
        while (!(current == start)) {
            path.push_back(current);
            current = came_from[current];
        }
        path.push_back(start);
        reverse(path.begin(), path.end());
        return path;
    }

    int heuristic(const Cell& a, const Cell& b) {
        return abs(a.x - b.x) + abs(a.y - b.y); // Manhattan
    }

    // Algoritmo BFS para mapeamento (exemplo de uso)
    vector<Cell> bfs(Cell start) {
        set<Cell> visited;
        queue<Cell> q;
        vector<Cell> order;
        q.push(start);
        visited.insert(start);
        while (!q.empty()) {
            Cell c = q.front(); q.pop();
            order.push_back(c);
            for (auto& n : neighbors(c)) {
                if (!visited.count(n)) {
                    visited.insert(n);
                    q.push(n);
                }
            }
        }
        return order;
    }

    void move_to(const Cell& target) {
        string dir = get_direction(robot_, target);
        if (dir.empty()) return;
        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        req->direction = dir;
        if (!move_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(get_logger(), "Aguardando serviço /move_command...");
            return;
        }
        auto future = move_client_->async_send_request(req);
        auto status = future.wait_for(std::chrono::seconds(2));
        if (status == std::future_status::ready) {
            robot_ = target;
        } else {
            RCLCPP_WARN(get_logger(), "Falha ao mover.");
        }
    }

    string get_direction(const Cell& from, const Cell& to) {
        if (to.x == from.x && to.y == from.y+1) return "down";
        if (to.x == from.x && to.y == from.y-1) return "up";
        if (to.x == from.x+1 && to.y == from.y) return "right";
        if (to.x == from.x-1 && to.y == from.y) return "left";
        return "";
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<AutonomousNavigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
