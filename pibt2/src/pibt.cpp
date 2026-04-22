#include "../include/pibt.hpp"

// for debug
#define SAFE_VALUE(opt, agent_id) \
    ([&]() -> decltype(auto) { \
        if (!(opt).has_value()) { \
            std::cerr << "FATAL: Agent " << (agent_id) \
                      << " has no orientation at " << __FILE__ \
                      << ":" << __LINE__ << std::endl; \
            std::abort(); \
        } \
        return (opt).value(); \
    })()


std::string orientationToString(Orientation dir) {
    switch (dir) {
        case Orientation::X_PLUS:  return "X_PLUS";
        case Orientation::X_MINUS: return "X_MINUS";
        case Orientation::Y_PLUS:  return "Y_PLUS";
        case Orientation::Y_MINUS: return "Y_MINUS";
        default: return "UNKNOWN";
    }
}

const std::string PIBT::SOLVER_NAME = "PIBT";

PIBT::PIBT(MAPF_Instance* _P)
    : MAPF_Solver(_P), 
      occupied_now(Agents(G->getNodesSize(), nullptr)),
      occupied_next(Agents(G->getNodesSize(), nullptr)),
      reserved_nodes(P->getNum(), nullptr),  
      push_count_table(P->getNum(), std::vector<int>(P->getNum(), 0))
{
  solver_name = PIBT::SOLVER_NAME;
}


void PIBT::run()
{
  
  std::cout << "=== Starting PIBT::run() ===" << std::endl;
  // compare priority of agents
  auto compare = [](Agent* a, const Agent* b) {
    if (a->elapsed != b->elapsed) return a->elapsed > b->elapsed;
    // use initial distance
    if (a->init_d != b->init_d) return a->init_d > b->init_d;
    return a->tie_breaker > b->tie_breaker;
  };
  Agents A;

  // initialize
  for (int i = 0; i < P->getNum(); ++i) {
    Node* s = P->getStart(i);
    Node* g = P->getGoal(i);
    int d = disable_dist_init ? 0 : pathDist(i,s,Orientation::Y_MINUS);
    Agent* a = new Agent{i,                          // id
                         s,                          // current location
                         nullptr,                    // next location
                         g,                          // goal
                         Orientation::Y_MINUS,        // current orientation
                         std::nullopt,               // next orientation
                         0,                          // elapsed
                         d,                          // dist from s -> g
                         getRandomFloat(0, 1, MT)};  // tie-breaker
    a->swap_completed = true;
    A.push_back(a);
    occupied_now[s->id] = a;
  }
  Config initial_config = P->getConfigStart();
  std::vector<Orientation> initial_orients(P->getNum(), Orientation::Y_MINUS);
  solution.addWithOrientation(initial_config, initial_orients);

  // main loop
  int timestep = 0;
  //int max_loop = 100;
  while (true) {
  //while (timestep < max_loop) {
    info(" ", "elapsed:", getSolverElapsedTime(), ", timestep:", timestep);

    for (size_t i = 0; i < occupied_next.size(); ++i) {
            if (occupied_next[i] != nullptr) {
                std::cout << "Warning: occupied_next[" << i 
                << "] not cleared from previous timestep" << std::endl;}}

    // planning
    std::sort(A.begin(), A.end(), compare);
    for (auto a : A) {
      // if the agent has next location, then skip
      if (a->v_next == nullptr) {
        // determine its next location
        funcPIBT(a);
      }
    }

    // plan.cpp
    bool check_goal_cond = true;
    Config config(P->getNum(), nullptr); //plan.cpp
    std::vector<Orientation> orients(P->getNum());

    for (auto a : A) {
      if (occupied_now[a->v_now->id] == a) occupied_now[a->v_now->id] = nullptr;
      occupied_next[a->v_next->id] = nullptr;
      
      // set next location and orientation
      config[a->id] = a->v_next;
      orients[a->id] = SAFE_VALUE(a->ott_next, a->id); 
      occupied_now[a->v_next->id] = a;

      // check goal condition
      check_goal_cond &= (a->v_next == a->g);
      // update priority
      a->elapsed = (a->v_next == a->g) ? 0 : a->elapsed + 1;
      // reset params
      a->v_now = a->v_next;
      a->v_next = nullptr;

      a->ott_now = a->ott_next;
      a->ott_next = std::nullopt;
    }

    // update plan
    solution.addWithOrientation(config, orients);

    ++timestep;

    // success
    if (check_goal_cond) {
      solved = true;
      break;
    }

    // failed
    if (timestep >= max_timestep || overCompTime()) {
      break;
    }
  }

  // memory clear
  for (auto a : A) {
    if (a) {
        delete a;
    }
  }
  A.clear();
}

bool PIBT::funcPIBT(Agent* ai, Agent* aj, bool is_initial)
{
  if (is_initial) {
        request_chain.clear();
        cycle_handled = false;
        initial_requester = ai;
    }

  // compare two nodes by LGS
  auto compare = [&](Node* const v, Node* const u) {   
    float d_v;
    if (v == ai->v_now) {
        d_v = getMinDistToGoal(ai->id, v, SAFE_VALUE(ai->ott_now, ai->id)) + 1;
    } else {
        Orientation target_dir_v = solution.getRelativePosition(ai->v_now, v);
        d_v = getMinDistToGoal(ai->id, v, target_dir_v);
    }

    float d_u;
    if (u == ai->v_now) {
        d_u = getMinDistToGoal(ai->id, u, SAFE_VALUE(ai->ott_now, ai->id)) + 1;
    } else {
        Orientation target_dir_u = solution.getRelativePosition(ai->v_now, u);
        d_u = getMinDistToGoal(ai->id, u, target_dir_u);
    }
    
    if (v != ai->v_now) {
        Orientation target_dir_v = solution.getRelativePosition(ai->v_now, v);
        int angle_diff_v = solution.getAngleDifference(SAFE_VALUE(ai->ott_now, ai->id), target_dir_v);
        if (angle_diff_v == 0) d_v += 1;
        else if (angle_diff_v == 90) d_v += 2;
        else if (angle_diff_v == 180) d_v += 3;
    }
    
    if (u != ai->v_now) {
        Orientation target_dir_u = solution.getRelativePosition(ai->v_now, u);
        int angle_diff_u = solution.getAngleDifference(SAFE_VALUE(ai->ott_now, ai->id), target_dir_u);
        if (angle_diff_u == 0) d_u += 1;
        else if (angle_diff_u == 90) d_u += 2;
        else if (angle_diff_u == 180) d_u += 3;
    }

    if (d_v != d_u) return d_v < d_u;

    // tie break
    if (occupied_now[v->id] != nullptr && occupied_now[u->id] == nullptr)
      return false;
    if (occupied_now[v->id] == nullptr && occupied_now[u->id] != nullptr)
      return true;
    return false;
  };

  // get candidates by LGS
  Nodes C = ai->v_now->neighbor;
  C.push_back(ai->v_now);
  // randomize
  std::shuffle(C.begin(), C.end(), *MT);
  // sort
  std::sort(C.begin(), C.end(), compare);
  
  if (!is_initial && aj != nullptr) {
    PushEscapeTrigger(C, ai->id, aj->id);
  }
  
  Agent* swap_agent = swap_possible_and_required(ai, C);
  if (swap_agent != nullptr){
    std::reverse(C.begin(), C.end());
    std::cout << "Swap agent :" << swap_agent->id << std::endl;    
    }
  
  int m = 0;

  if (reserved_nodes[ai->id] != nullptr) {
    auto reserved_node = std::find(C.begin(), C.end(), reserved_nodes[ai->id]);
    if (reserved_node != C.end()) {
        Node* reserved = *reserved_node;
        C.erase(reserved_node);
        C.insert(C.begin(), reserved);
        }
    }
  

  for (auto u : C) {
    // avoid conflicts
    if (occupied_next[u->id] != nullptr) {
                  m++;
                  continue;
    }
    if (aj != nullptr && u == aj->v_now) {
        m++;
        continue;
    }

    // reserve
    occupied_next[u->id] = ai;
    ai->v_next = u;

    // check if cycle occurs
    if (!is_initial && u == initial_requester->v_now) {
        std::cout << "Cycle detected: Agent " << ai->id 
                    << " requests node occupied by initial requester " 
                    << initial_requester->id << std::endl;

    // [Debug] available orientation or not
        if (!ai->ott_now.has_value()) {
            std::cerr << "Error: Agent " << ai->id << " has no valid orientation during cycle detection" << std::endl;
            exit(1);
        }

        request_chain.push_back({ai, u});
        handleCycleWithOrientation();
        cycle_handled = true;
        return true;
    }

    auto ak = occupied_now[u->id];
    if (ak != nullptr && ak->v_next == nullptr) {
      request_chain.push_back({ai, u});
      if (!funcPIBT(ak, ai, false)) {
        request_chain.pop_back();
        occupied_next[u->id] = nullptr;
        ai->v_next = nullptr;
        m++;
        continue;
      }  // replanning

    }

    // if action has been determined when handling cycle, further planning is unnecessary
    if (cycle_handled) {
        return true;
    }

    // compute the first to move to next vertex u
    auto [next_node, next_orientation] = solution.computeAction(
        ai->v_now,    
        u,           
        SAFE_VALUE(ai->ott_now, ai->id)  
    );
        
    // if agent cannot move to a new vertex, change its orientation
    if (next_node == ai->v_now) {
        // reset the vertex occupied and change orientation when needed
        ai->v_next = ai->v_now;
        occupied_next[u->id] = nullptr;
        occupied_next[ai->v_next->id] = ai;
        ai->ott_next = next_orientation;
        if(ai->swap_completed){reserved_nodes[ai->id] = nullptr;} // reserve the node before swap is completed
        if (next_orientation != ai->ott_now){
            reserved_nodes[ai->id] = u;    
        }
    }
    else {
        // if agent can moving forward then do so
        ai->v_next = next_node;
        ai->ott_next = next_orientation;
        occupied_next[ai->v_next->id] = ai;
        reserved_nodes[ai->id] = nullptr;

        if (!is_initial && aj != nullptr && ai->v_next != ai->v_now) {
            updatePushCount(ai->id, aj->id);
        }
        
        
    }

    auto al = occupied_now[u->id];
    if (al != nullptr && al->v_next == al->v_now) {
        // other agent must stay because it will adjust orientation, current agent must also stay
        if(next_node!=ai->v_now){ //if current agent wants to moving forward
        occupied_next[ai->v_now->id] = ai;
        ai->v_next = ai->v_now; // reserve current vertex
        ai->ott_next = ai->ott_now; 

        reserved_nodes[ai->id] = u;
        }
    }

    // compute action for the other agent involved in swap
    if (m == 0 && swap_agent != nullptr && swap_agent->v_next == nullptr && 
        (occupied_next[ai->v_now->id] == nullptr or occupied_next[ai->v_now->id] == ai)) {
        std::cout << "-----compute action for swap agent----- " << std::endl;
        swap_agent->swap_completed = false;
        swap_agent->v_next = ai->v_now;
        occupied_next[swap_agent->v_next->id] = swap_agent;
        auto [next_node_swap_agent, next_orientation_swap_agent] = solution.computeAction(
        swap_agent->v_now,
        swap_agent->v_next,           
        SAFE_VALUE(swap_agent->ott_now, swap_agent->id)  
        );

        if (next_node_swap_agent == swap_agent->v_now) {
            occupied_next[swap_agent->v_next->id] = nullptr;
            swap_agent->v_next = swap_agent->v_now;
            occupied_next[swap_agent->v_next->id] = swap_agent;
            swap_agent->ott_next = next_orientation_swap_agent;
            reserved_nodes[swap_agent->id] = nullptr;
            if (next_orientation_swap_agent != swap_agent->ott_now){
                reserved_nodes[swap_agent->id] = ai->v_now; 
            }
        }
        else {
            swap_agent->v_next = next_node_swap_agent;
            swap_agent->ott_next = next_orientation_swap_agent;
            occupied_next[swap_agent->v_next->id] = swap_agent;
            reserved_nodes[swap_agent->id] = nullptr;
            swap_agent->swap_completed = true;
        }

        if (ai->v_next == ai->v_now) {
            if(next_node_swap_agent!=swap_agent->v_now){
            occupied_next[swap_agent->v_now->id] = swap_agent;
            swap_agent->v_next = swap_agent->v_now;
            swap_agent->ott_next = swap_agent->ott_now; 
            
            reserved_nodes[swap_agent->id] = ai->v_now;
            }
        }         
    }
    
    return true;
  }



  // failed to secure node
  //std::cout << "invalid" << aj->id << std::endl;
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  ai->ott_next = ai->ott_now;
  return false;
}

float PIBT::getMinDistToGoal(int agent_id, Node* node, Orientation current_dir) {
    float min_dist = std::numeric_limits<float>::max();
    for (Orientation goal_dir : {Orientation::X_PLUS, Orientation::X_MINUS, 
                                Orientation::Y_PLUS, Orientation::Y_MINUS}) {
        float dist = pathDist(agent_id, node, current_dir);
        min_dist = std::min(min_dist, dist);
    }
    return min_dist;
}

float PIBT::getMinDistAllDirections(int agent_id, Node* node) {
    float min_dist = std::numeric_limits<float>::max();
    
    // 遍历当前节点的所有可能方向
    for (Orientation current_dir : {Orientation::X_PLUS, Orientation::X_MINUS, 
                                  Orientation::Y_PLUS, Orientation::Y_MINUS}) {
        float dist = getMinDistToGoal(agent_id, node, current_dir);
        min_dist = std::min(min_dist, dist);
    }
    return min_dist;
}

// check whether all agents in cycle is heading to their requesting node
// if no, adjust the orientation; if yes, moving forward
void PIBT::handleCycleWithOrientation() {
    //std::cout << "Cycle detected at timestep " << solution.getMakespan() << std::endl;
    
    if (request_chain.empty()) {
        std::cout << "[Error] Empty request chain" << std::endl;
        return;
    }
    
    bool all_oriented_correctly = true;
    std::vector<bool> correct_orientations(request_chain.size());
    
    // check the orientation of all agents in the cycle
    for (size_t i = 0; i < request_chain.size(); ++i) {
        Agent* current_agent = request_chain[i].agent;
        Node* requested_node = request_chain[i].requested_node;
        
        if (!current_agent || !requested_node || !current_agent->v_now) {
            std::cout << "[Error] Invalid vertex in request chain" << std::endl;
            continue;
        }

        Orientation target_orientation = solution.getRelativePosition(
            current_agent->v_now, requested_node);  
              
        if (!current_agent->ott_now.has_value()) {
            std::cout << "[Error] Agent " << current_agent->id << " has no orientation" << std::endl;
            all_oriented_correctly = false;
            correct_orientations[i] = false;
            continue;
        }
        
        correct_orientations[i] = (SAFE_VALUE(current_agent->ott_now, current_agent->id) == target_orientation);
        if (!correct_orientations[i]) {
            all_oriented_correctly = false;
        }
    }

    if (!all_oriented_correctly) {
        // adjust the orientation if needed
        for (size_t i = 0; i < request_chain.size(); ++i) {
            Agent* current_agent = request_chain[i].agent;
            Node* requested_node = request_chain[i].requested_node;

            if (!current_agent->ott_now.has_value()) {
            std::cout << "[Error] Agent " << current_agent->id 
                      << " has no orientation in cycle handling!" << std::endl;
            }
            
            if (!correct_orientations[i]) {
                if (!current_agent->ott_now.has_value()) {
                    current_agent->ott_now = Orientation::Y_MINUS;
                }

                auto [next_node, next_orientation] = solution.computeAction(
                    current_agent->v_now,
                    requested_node,
                    SAFE_VALUE(current_agent->ott_now, current_agent->id)
                );
                
                current_agent->v_next = current_agent->v_now;
                current_agent->ott_next = next_orientation;
                occupied_next[current_agent->v_now->id] = current_agent;
            } else {
                // hold current vertex and orientation
                current_agent->v_next = current_agent->v_now;
                current_agent->ott_next = current_agent->ott_now;
                occupied_next[current_agent->v_now->id] = current_agent;
            }
        }
    } else {
        // all agents in cycle are facing to their desired node, then moving forward
        for (size_t i = 0; i < request_chain.size(); ++i) {
            Agent* current_agent = request_chain[i].agent;
            Node* requested_node = request_chain[i].requested_node;
            
            current_agent->v_next = requested_node;
            current_agent->ott_next = current_agent->ott_now;
            occupied_next[requested_node->id] = current_agent;
        }
    }
}

auto PIBT::swap_possible_and_required(Agent* ai, const Nodes& C) -> Agent*
{
    const auto i = ai->id;
    if (C[0] == ai->v_now) return nullptr;

    auto aj = occupied_now[C[0]->id];
    if (aj != nullptr && aj->v_next == nullptr &&
        // is_swap_required(ai->id, aj->id, ai->v_now, aj->v_now) &&
        is_swap_required(ai->id, aj->id, ai->v_now, aj->v_now) &&
        is_swap_possible(aj->v_now, ai->v_now)) {
        return aj;
    }

    for (auto u : ai->v_now->neighbor) {
        auto ak = occupied_now[u->id];
        if (ak == nullptr || C[0] == ak->v_now) continue;
        // if (is_swap_required(ak->id, ai->id, ai->v_now, C[0]) &&
        if (is_swap_required(ak->id, ai->id, ai->v_now, C[0]) &&
            is_swap_possible(C[0], ai->v_now)) {
            return ak;
        }
    }
    return nullptr;
}

bool PIBT::is_swap_required(const int pusher, const int puller,
                           Node* v_pusher_origin, Node* v_puller_origin)
{
    auto v_pusher = v_pusher_origin;
    auto v_puller = v_puller_origin;
    Node* tmp = nullptr;

    while (getMinDistAllDirections(pusher, v_puller) < 
           getMinDistAllDirections(pusher, v_pusher)) {
        auto n = v_puller->neighbor.size();
        for (auto u : v_puller->neighbor) {
            auto a = occupied_now[u->id];
            if (u == v_pusher ||
                (u->neighbor.size() == 1 && a != nullptr && a->g == u)) {
                --n;
            } else {
                tmp = u;
            }
        }
        if (n >= 2) return false;
        if (n <= 0) break;
        v_pusher = v_puller;
        v_puller = tmp;
    }

    return (getMinDistAllDirections(puller, v_pusher) < 
            getMinDistAllDirections(puller, v_puller)) &&
           (getMinDistAllDirections(pusher, v_pusher) == 0 ||
            getMinDistAllDirections(pusher, v_puller) < 
            getMinDistAllDirections(pusher, v_pusher));
}


bool PIBT::is_swap_possible(Node* v_pusher_origin, Node* v_puller_origin)
{
    auto v_pusher = v_pusher_origin;
    auto v_puller = v_puller_origin;
    Node* tmp = nullptr;

    while (v_puller != v_pusher_origin) {
        auto n = v_puller->neighbor.size();
        for (auto u : v_puller->neighbor) {
            auto a = occupied_now[u->id];
            if (u == v_pusher ||
                (u->neighbor.size() == 1 && a != nullptr && a->g == u)) {
                --n;
            } else {
                tmp = u;
            }
        }
        if (n >= 2) return true;
        if (n <= 0) return false;
        v_pusher = v_puller;
        v_puller = tmp;
    }
    return false;
}

void PIBT::updatePushCount(int pushed_agent_id, int pusher_id) {
    if (pushed_agent_id >= 0 && pushed_agent_id < (int)push_count_table.size() &&
        pusher_id >= 0 && pusher_id < (int)push_count_table[0].size()) {
        push_count_table[pushed_agent_id][pusher_id]++;
    }
}

int PIBT::getPushCount(int pushed_agent_id, int pusher_id) const {
    if (pushed_agent_id >= 0 && pushed_agent_id < (int)push_count_table.size() &&
        pusher_id >= 0 && pusher_id < (int)push_count_table[0].size()) {
        return push_count_table[pushed_agent_id][pusher_id];
    }
    return 0;
}

void PIBT::printPushCountTable() const {
    std::cout << "Push Count Table:" << std::endl;
    std::cout << "Format: [pushed_agent_id][pusher_id] = count" << std::endl;
    for (size_t i = 0; i < push_count_table.size(); ++i) {
        for (size_t j = 0; j < push_count_table[i].size(); ++j) {
            if (push_count_table[i][j] > 0) {
                std::cout << "[" << i << "][" << j << "] = " 
                         << push_count_table[i][j] << std::endl;
            }
        }
    }
}

void PIBT::PushEscapeTrigger(Nodes& C, int pushed_agent_id, int pusher_id) {
    int push_time = getPushCount(pushed_agent_id, pusher_id);
    if (push_time >= 2 && C.size() > 1) { // change k value here
        std::shuffle(C.begin(), C.end(), *MT);
        push_count_table[pushed_agent_id][pusher_id] = 0;
    }
}


void PIBT::setParams(int argc, char* argv[])
{
  struct option longopts[] = {
      {"disable-dist-init", no_argument, 0, 'd'},
      {0, 0, 0, 0},
  };
  optind = 1;  // reset
  int opt, longindex;
  while ((opt = getopt_long(argc, argv, "d", longopts, &longindex)) != -1) {
    switch (opt) {
      case 'd':
        disable_dist_init = true;
        break;
      default:
        break;
    }
  }
}

void PIBT::printHelp()
{
  std::cout << PIBT::SOLVER_NAME << "\n"
            << "  -d --disable-dist-init"
            << "        "
            << "disable initialization of priorities "
            << "using distance from starts to goals" << std::endl;
}
