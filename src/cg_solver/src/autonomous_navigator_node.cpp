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
    enum class Phase { MAPPING, GOING_TO_START, FINAL_RUN, DONE };

    AutonomousNavigator() : Node("cg_autonomous_navigator") {
        this->declare_parameter<std::string>("mode", "pathfind");
        this->get_parameter("mode", mode_);

        move_client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        get_map_client_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");
        sensor_sub_ = this->create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors", rclcpp::SensorDataQoS(),
            std::bind(&AutonomousNavigator::sensor_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&AutonomousNavigator::main_loop, this));
        
        received_map_ = false;
        waiting_map_ = false;
        move_in_progress_ = false;
        current_phase_ = Phase::MAPPING;

        partial_map_.resize(30 * 30, -1);
    }

private:
    std::string mode_;
    Phase current_phase_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr get_map_client_;
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool waiting_map_;
    bool move_in_progress_;
    std::vector<int> partial_map_;

    int width_ = 0, height_ = 0;
    Cell robot_, goal_;
    bool received_map_;
    vector<Cell> path_;
    size_t path_index_ = 0;

    Cell find_exploration_target() {
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                if (partial_map_[y * width_ + x] == 0 || partial_map_[y * width_ + x] == 2) {
                    Cell current{x, y};
                    for (auto [dx, dy] : vector<pair<int,int>>{{0,1},{1,0},{0,-1},{-1,0}}) {
                        Cell neighbor{current.x + dx, current.y + dy};
                        if (neighbor.x >= 0 && neighbor.x < width_ && neighbor.y >= 0 && neighbor.y < height_) {
                            if (partial_map_[neighbor.y * width_ + neighbor.x] == -1) {
                                return current;
                            }
                        }
                    }
                }
            }
        }
        return {-1, -1};
    }

    void main_loop() {
        if (!received_map_ || move_in_progress_) {
            if (!received_map_ && !waiting_map_) request_map();
            return;
        }

        if (mode_ == "pathfind") {
            if (path_.empty()) {
                path_ = a_star(robot_, goal_);
                path_index_ = 1;
                RCLCPP_INFO(get_logger(), "Modo PATHFIND: Caminho calculado com %zu passos.", path_.size());
            }
        } else { // explore mode
            if (current_phase_ == Phase::MAPPING) {
                if (path_index_ >= path_.size()) {
                    path_.clear();
                    path_index_ = 0;
                    Cell target = find_exploration_target();
                    if (target.x != -1) {
                        RCLCPP_INFO(get_logger(), "Mapeando: Novo alvo de exploração: (%d, %d)", target.x, target.y);
                        path_ = a_star(robot_, target);
                        path_index_ = 1;
                    } else {
                        RCLCPP_INFO(get_logger(), "Mapeamento completo! Planejando rota para o início.");
                        current_phase_ = Phase::GOING_TO_START;
                    }
                }
            }
            if (current_phase_ == Phase::GOING_TO_START) {
                if (path_.empty()) {
                    path_ = a_star(robot_, {1, 1});
                    path_index_ = 1;
                    RCLCPP_INFO(get_logger(), "Retornando ao início. Passos: %zu", path_.size());
                } else if (path_index_ >= path_.size()) {
                    RCLCPP_INFO(get_logger(), "De volta ao início! Calculando caminho final.");
                    path_.clear();
                    path_index_ = 0;
                    current_phase_ = Phase::FINAL_RUN;
                }
            }
            if (current_phase_ == Phase::FINAL_RUN && path_.empty()) {
                path_ = a_star(robot_, goal_);
                path_index_ = 1;
                RCLCPP_INFO(get_logger(), "Caminho final para o alvo calculado com %zu passos.", path_.size());
            }
        }

        if (path_index_ < path_.size()) {
            move_to(path_[path_index_]);
        } else if (current_phase_ == Phase::FINAL_RUN) {
            RCLCPP_INFO(get_logger(), "Robô chegou ao alvo!");
            current_phase_ = Phase::DONE;
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
        get_map_client_->async_send_request(req, std::bind(&AutonomousNavigator::map_response_callback, this, std::placeholders::_1));
    }

    void map_response_callback(rclcpp::Client<cg_interfaces::srv::GetMap>::SharedFuture future) {
        auto resp = future.get();
        vector<int> ground_truth_map;
        for (const auto& s : resp->occupancy_grid_flattened) {
            if (s == "b") ground_truth_map.push_back(1);
            else if (s == "f") ground_truth_map.push_back(0);
            else if (s == "r") ground_truth_map.push_back(2);
            else if (s == "t") ground_truth_map.push_back(3);
            else ground_truth_map.push_back(-1);
        }
        height_ = resp->occupancy_grid_shape[0];
        width_ = resp->occupancy_grid_shape[1];

        if (mode_ == "explore") {
            RCLCPP_INFO(get_logger(), "Modo EXPLORE: O robô está 'cego' e deve mapear o labirinto.");
            partial_map_.assign(width_ * height_, -1);
            current_phase_ = Phase::MAPPING;
        } else {
            RCLCPP_INFO(get_logger(), "Modo PATHFIND: Robô conhece o mapa completo.");
            partial_map_ = ground_truth_map;
            current_phase_ = Phase::FINAL_RUN;
        }

        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                if (ground_truth_map[y * width_ + x] == 2) robot_ = {x, y};
                if (ground_truth_map[y * width_ + x] == 3) goal_ = {x, y};
            }
        }
        
        received_map_ = true;
        waiting_map_ = false;
        RCLCPP_INFO(get_logger(), "Dimensões do mapa recebidas: %dx%d. O robô começa em (%d, %d).", width_, height_, robot_.x, robot_.y);
    }

    void sensor_callback(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
        if (robot_.x >= 0 && robot_.x < width_ && robot_.y >= 0 && robot_.y < height_) {
            partial_map_[robot_.y * width_ + robot_.x] = 0;
        }
        std::map<std::string, std::pair<int,int>> dir_map = {
            {"up", {0,-1}}, {"down", {0,1}}, {"left", {-1,0}}, {"right", {1,0}},
            {"up_left", {-1,-1}}, {"up_right", {1,-1}}, {"down_left", {-1,1}}, {"down_right", {1,1}}
        };
        for (auto const& [key, val] : dir_map) {
            int nx = robot_.x + val.first;
            int ny = robot_.y + val.second;
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                int idx = ny * width_ + nx;
                std::string s_val;
                if (key == "up") s_val = msg->up; else if (key == "down") s_val = msg->down;
                else if (key == "left") s_val = msg->left; else if (key == "right") s_val = msg->right;
                else if (key == "up_left") s_val = msg->up_left; else if (key == "up_right") s_val = msg->up_right;
                else if (key == "down_left") s_val = msg->down_left; else if (key == "down_right") s_val = msg->down_right;
                
                if (s_val == "b") partial_map_[idx] = 1;
                else if (s_val == "f") partial_map_[idx] = 0;
                else if (s_val == "t") partial_map_[idx] = 3;
            }
        }
    }

    bool is_free(const Cell& c) {
        if (c.x < 0 || c.x >= width_ || c.y < 0 || c.y >= height_) return false;
        int v = partial_map_[c.y * width_ + c.x];
        
        if (current_phase_ == Phase::MAPPING || current_phase_ == Phase::GOING_TO_START) {
            return v == 0 || v == 2;
        } else {
            return v == 0 || v == 2 || v == 3;
        }
    }

    vector<Cell> neighbors(const Cell& c) {
        vector<Cell> result;
        for (auto [dx, dy] : vector<pair<int,int>>{{0,1},{1,0},{0,-1},{-1,0}}) {
            Cell n{c.x+dx, c.y+dy};
            if (is_free(n)) result.push_back(n);
        }
        return result;
    }

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
        vector<Cell> path;
        Cell current = goal;
        if (!came_from.count(goal)) return path;
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

    void move_to(const Cell& target) {
        string dir = get_direction(robot_, target);
        if (dir.empty()) {
            RCLCPP_WARN(get_logger(), "Não é possível determinar a direção ou o alvo não é adjacente. Limpando o caminho.");
            path_.clear();
            path_index_ = 0;
            return;
        }

        if (!move_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(get_logger(), "Aguardando serviço /move_command...");
            return;
        }

        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = dir;
        move_in_progress_ = true;

        auto response_callback = [this](rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture future) {
            auto result = future.get();
            if (result->success) {
                this->robot_.x = result->robot_pos[0];
                this->robot_.y = result->robot_pos[1];
                this->path_index_++;
            } else {
                RCLCPP_ERROR(get_logger(), "O serviço de movimento indicou FALHA.");
                path_.clear();
                path_index_ = 0;
            }
            this->move_in_progress_ = false;
        };

        move_client_->async_send_request(request, response_callback);
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
