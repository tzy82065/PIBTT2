/*
 * Implementation of Priority Inheritance with Backtracking (PIBT)
 *
 * - ref
 * Okumura, K., Machida, M., Défago, X., & Tamura, Y. (2019).
 * Priority Inheritance with Backtracking for Iterative Multi-agent Path
 * Finding. In Proceedings of the Twenty-Eighth International Joint Conference
 * on Artificial Intelligence (pp. 535–542).
 */

#pragma once
#include "solver.hpp"
#include "orientation.hpp"
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>

class PIBT : public MAPF_Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  // PIBT agent
  struct Agent {
    int id;
    Node* v_now;        // current location
    Node* v_next;       // next location
    Node* g;            // goal
    std::optional<Orientation> ott_now;
    std::optional<Orientation> ott_next;
    int elapsed;        // eta
    int init_d;         // initial distance
    float tie_breaker;  // epsilon, tie-breaker
    bool swap_completed;// test:swap
  };
  using Agents = std::vector<Agent*>;
  
  //
  struct Request {
    Agent* agent;      
    Node* requested_node; 
  };
  std::vector<Request> request_chain;
  bool cycle_detected = false;       
  bool cycle_handled = false;   
  Agent* initial_requester = nullptr; 
  
  private:
  // <node-id, agent>, whether the node is occupied or not
  // work as reservation table 
  Agents occupied_now;
  Agents occupied_next;

  // option
  bool disable_dist_init = false;

  // result of priority inheritance: true -> valid, false -> invalid
  bool funcPIBT(Agent* ai, Agent* aj = nullptr, bool is_initial = true);

  // main
  void run();
  
  // minimal distance to 4 states of goal in cost table
  float getMinDistToGoal(int agent_id, Node* node, Orientation current_dir);

  // 计算从节点所有可能方向到目标的最小距离
  float getMinDistAllDirections(int agent_id, Node* node);

  // handle cycle
  void handleCycleWithOrientation();

  // 节点预留表，大小为智能体数量
  std::vector<Node*> reserved_nodes;  // R[i] 表示智能体i预留的节点

  //  swap operation
  Agent* swap_possible_and_required(Agent* ai, const Nodes& C);
  bool is_swap_required(const int pusher, const int puller,
                        Node* v_pusher_origin, Node* v_puller_origin);
  bool is_swap_possible(Node* v_pusher_origin, Node* v_puller_origin);

  // [pushed_agent_id][pusher_id] = push_times
  std::vector<std::vector<int>> push_count_table;
  // 更新 push 次数的辅助函数
  void updatePushCount(int pushed_agent_id, int pusher_id);
  // 获取某个智能体被特定智能体push的次数
  int getPushCount(int pushed_agent_id, int pusher_id) const;
  // 打印push计数表
  void printPushCountTable() const;
  // 根据概率执行额外操作（移动节点）
  void PushEscapeTrigger(Nodes& C, int pushed_agent_id, int pusher_id);

public:
  PIBT(MAPF_Instance* _P);
  ~PIBT() {}

  void setParams(int argc, char* argv[]);
  static void printHelp();
};
