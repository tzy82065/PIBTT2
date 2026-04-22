#include "../include/plan.hpp"

Config Plan::get(const int t) const
{
  const int configs_size = configs.size();
  if (!(0 <= t && t < configs_size)) halt("invalid timestep");
  return configs[t];
}

Node* Plan::get(const int t, const int i) const
{
  if (empty()) halt("invalid operation");
  if (!(0 <= t && t < (int)configs.size())) halt("invalid timestep");
  if (!(0 <= i && i < (int)configs[0].size())) halt("invalid agent id");
  return configs[t][i];
}

Path Plan::getPath(const int i) const
{
  Path path;
  int makespan = getMakespan();
  for (int t = 0; t <= makespan; ++t) path.push_back(get(t, i));
  return path;
}

// test with orientation
void Plan::addOrientation(const std::vector<Orientation>& orients) {
    if (configs.empty()) {
        halt("invalid operation: cannot add orientation before config");
    }
    if (orients.size() != configs.back().size()) {
        halt("invalid orientation operation");
    }
    orientations.back() = orients;
}

Orientation Plan::getOrientation(const int t, const int i) const {
    if (orientations.empty()) halt("invalid orientation operation");
    if (!(0 <= t && t < (int)orientations.size())) halt("invalid timestep");
    if (!(0 <= i && i < (int)orientations[0].size())) halt("invalid agent id");
    return orientations[t][i];
}

std::vector<Orientation> Plan::getOrientations(const int t) const {
    if (!(0 <= t && t < (int)orientations.size())) halt("invalid timestep");
    return orientations[t];
}


//
Orientation Plan::getRelativePosition(Node* current, Node* target) const {
    bool is_neighbor = false;
    for (auto& n : current->neighbor) {
        if (n == target) {
            is_neighbor = true;
            break;
        }
    }
    if (!is_neighbor) {
        halt("Nodes are not neighbors");
    }

    // relative position
    if (target->pos.x > current->pos.x) return Orientation::X_PLUS;
    if (target->pos.x < current->pos.x) return Orientation::X_MINUS;
    if (target->pos.y > current->pos.y) return Orientation::Y_PLUS;
    if (target->pos.y < current->pos.y) return Orientation::Y_MINUS;
    
    halt("Invalid position relationship");
    return Orientation::X_PLUS;
}

int Plan::getAngleDifference(Orientation dir1, Orientation dir2) const {
    const std::map<Orientation, int> dir_to_angle = {
        {Orientation::X_PLUS, 0},
        {Orientation::Y_PLUS, 90},
        {Orientation::X_MINUS, 180},
        {Orientation::Y_MINUS, 270}
    };
    
    int angle1 = dir_to_angle.at(dir1);
    int angle2 = dir_to_angle.at(dir2);
    int diff = std::abs(angle1 - angle2);
    return std::min(diff, 360 - diff);
}

Orientation Plan::rotateCounterClockwise(Orientation orient) const {
    switch (orient) {
        case Orientation::X_PLUS: return Orientation::Y_PLUS;
        case Orientation::Y_PLUS: return Orientation::X_MINUS;
        case Orientation::X_MINUS: return Orientation::Y_MINUS;
        case Orientation::Y_MINUS: return Orientation::X_PLUS;
        default: halt("Invalid orientation");
    }
    return Orientation::X_PLUS; 
}

std::pair<Node*, Orientation> Plan::computeAction(
    Node* current, Node* target, Orientation current_orient) const {
    
    // current node or neighboring node
    if (current == target) {
        return {current, current_orient};
    }
    
    bool is_neighbor = false;
    for (auto& n : current->neighbor) {
        if (n == target) {
            is_neighbor = true;
            break;
        }
    }
    if (!is_neighbor) {
        halt("Target node must be either current node or its neighbor");
    }

    Orientation relative_pos = getRelativePosition(current, target);
    
    int angle_diff = getAngleDifference(current_orient, relative_pos);
    
    if (angle_diff == 0) {
        // move forward
        return {target, relative_pos};
    }
    else if (angle_diff == 90) {
        // adjust direction
        return {current, relative_pos};
    }
    else if (angle_diff == 180) {
        // adjust direction
        return {current, rotateCounterClockwise(current_orient)};
    }
    else {
        halt("Invalid angle difference");
    }
    
    return {nullptr, Orientation::X_PLUS};
}

/*
ActionSequence Plan::computeActionSequence(
    Node* current, Node* target, Orientation current_orient) const {
    
    ActionSequence sequence;
    
    if (current == target) {
        sequence.emplace_back(current, current_orient);
        return sequence;
    }
    
    bool is_neighbor = false;
    for (auto& n : current->neighbor) {
        if (n == target) {
            is_neighbor = true;
            break;
        }
    }
    if (!is_neighbor) {
        halt("Target node must be either current node or its neighbor");
    }

    Orientation relative_pos = getRelativePosition(current, target);
    int angle_diff = getAngleDifference(current_orient, relative_pos);
    
    if (angle_diff == 0) {
        sequence.emplace_back(target, relative_pos);
    }
    else if (angle_diff == 90) {
        sequence.emplace_back(current, relative_pos);
        sequence.emplace_back(target, relative_pos);
    }
    else if (angle_diff == 180) {
        Orientation intermediate = rotateCounterClockwise(current_orient);
        sequence.emplace_back(current, intermediate);
        sequence.emplace_back(current, relative_pos);
        sequence.emplace_back(target, relative_pos);
    }
    
    return sequence;
}

void Plan::storeActionSequence(int agent_id, int timestep, 
                             const ActionSequence& sequence) {
    while (action_tables.size() <= timestep) {
        action_tables.emplace_back();
    }
    action_tables[timestep][agent_id] = sequence;
}

ActionSequence Plan::getStoredActions(int agent_id, int timestep) const {
    if (timestep >= action_tables.size()) {
        return ActionSequence();
    }
    auto& table = action_tables[timestep];
    auto it = table.find(agent_id);
    if (it == table.end()) {
        return ActionSequence();
    }
    return it->second;
}

void Plan::removeStoredActions(int agent_id, int timestep) {
    if (timestep < action_tables.size()) {
        action_tables[timestep].erase(agent_id);
    }
}

bool Plan::hasStoredActions(int agent_id, int timestep) const {
    if (timestep >= action_tables.size()) {
        return false;
    }
    return action_tables[timestep].count(agent_id) > 0;
}

void Plan::clearActionTables() {
    action_tables.clear();
}

std::optional<Action> Plan::getAndRemoveFirstAction(int agent_id, int timestep) {
    if (timestep >= action_tables.size()) {
        return std::nullopt;
    }
    
    auto& table = action_tables[timestep];
    auto it = table.find(agent_id);
    if (it == table.end() || it->second.empty()) {
        return std::nullopt;
    }

    // 获取第一个动作
    Action first_action = it->second.front();
    
    // 移除第一个动作，保留剩余的动作
    it->second.erase(it->second.begin());
    
    // 如果动作序列为空，则移除该条目
    if (it->second.empty()) {
        table.erase(it);
    }

    return first_action;
}
*/

Config Plan::last() const
{
  if (empty()) halt("invalid operation");
  return configs[getMakespan()];
}

Node* Plan::last(const int i) const
{
  if (empty()) halt("invalid operation");
  if (i < 0 || (int)configs[0].size() <= i) halt("invalid operation");
  return configs[getMakespan()][i];
}

void Plan::clear() { configs.clear(); orientations.clear();}

void Plan::add(const Config& c)
{
  if (!configs.empty() && configs.at(0).size() != c.size()) {
    halt("invalid operation");
  }
  configs.push_back(c);
  orientations.push_back(std::vector<Orientation>(c.size()));
}

void Plan::addWithOrientation(const Config& c, const std::vector<Orientation>& orients) {
  if (!configs.empty() && configs.at(0).size() != c.size()) {
    halt("invalid operation");
  }
  if (c.size() != orients.size()) {
    halt("invalid orientation size");
  }
  configs.push_back(c);
  orientations.push_back(orients);
}

bool Plan::empty() const { return configs.empty(); }

int Plan::size() const { return configs.size(); }

int Plan::getMakespan() const { return size() - 1; }

int Plan::getPathCost(const int i) const
{
  const int makespan = getMakespan();
  const Node* g = get(makespan, i);
  int c = makespan;
  while (c > 0 && get(c - 1, i) == g) --c;
  return c;
}

int Plan::getSOC() const
{
  int makespan = getMakespan();
  if (makespan <= 0) return 0;
  int num_agents = get(0).size();
  int soc = 0;
  for (int i = 0; i < num_agents; ++i) soc += getPathCost(i);
  return soc;
}

Plan Plan::operator+(const Plan& other) const
{
  // check validity
  Config c1 = last();
  Config c2 = other.get(0);
  const int c1_size = c1.size();
  const int c2_size = c2.size();
  if (c1_size != c2_size) halt("invalid operation");
  for (int i = 0; i < c1_size; ++i) {
    if (c1[i] != c2[i]) halt("invalid operation.");
  }
  // merge
  Plan new_plan;
  new_plan.configs = configs;
  for (int t = 1; t < other.size(); ++t) new_plan.add(other.get(t));
  return new_plan;
}

void Plan::operator+=(const Plan& other)
{
  if (configs.empty()) {
    configs = other.configs;
    return;
  }
  // check validity
  if (!sameConfig(last(), other.get(0))) halt("invalid operation");
  // merge
  for (int t = 1; t < other.size(); ++t) add(other.get(t));
}

bool Plan::validate(MAPF_Instance* P) const
{
  return validate(P->getConfigStart(), P->getConfigGoal());
}

bool Plan::validate(MAPD_Instance* P) const
{
  // check tasks
  if ((int)P->getOpenTasks().size() > 0) {
    warn("validation, tasks remain");
    return false;
  }
  auto closed_tasks = P->getClosedTasks();
  if ((int)closed_tasks.size() != P->getTaskNum()) {
    warn("validation, num of closed_tasks is invalid");
    return false;
  }
  if (!std::all_of(closed_tasks.begin(), closed_tasks.end(), [](Task* task) {
        return task->loc_current == task->loc_delivery;
      })) {
    warn("validation, some tasks seem to be invalid");
    return false;
  }

  return validate(P->getConfigStart());
}

bool Plan::validate(const Config& starts, const Config& goals) const
{
  // check goal
  if (!sameConfig(last(), goals)) {
    warn("validation, invalid goals");
    return false;
  }
  return validate(starts);
}

bool Plan::validate(const Config& starts) const
{
  if (configs.empty()) return false;

  // start
  if (!sameConfig(starts, get(0))) {
    warn("validation, invalid starts");
    return false;
  }

  // check conflicts and continuity
  int num_agents = get(0).size();
  for (int t = 1; t <= getMakespan(); ++t) {
    if ((int)configs[t].size() != num_agents) {
      warn("validation, invalid size");
      return false;
    }
    for (int i = 0; i < num_agents; ++i) {
      Node* v_i_t = get(t, i);
      Node* v_i_t_1 = get(t - 1, i);
      Nodes cands = v_i_t_1->neighbor;
      cands.push_back(v_i_t_1);
      if (!inArray(v_i_t, cands)) {
        warn("validation, invalid move at t=" + std::to_string(t));
        return false;
      }
      // see conflicts
      for (int j = i + 1; j < num_agents; ++j) {
        Node* v_j_t = get(t, j);
        Node* v_j_t_1 = get(t - 1, j);
        if (v_i_t == v_j_t) {
          warn("validation, vertex conflict at v=" + std::to_string(v_i_t->id) +
               ", t=" + std::to_string(t));
          return false;
        }
        if (v_i_t == v_j_t_1 && v_i_t_1 == v_j_t) {
          warn("validation, swap conflict");
          return false;
        }
      }
    }
  }
  return true;
}

int Plan::getMaxConstraintTime(const int id, Node* s, Node* g, Graph* G) const
{
  const int makespan = getMakespan();
  const int dist = G->pathDist(s, g);
  const int num = configs[0].size();
  for (int t = makespan - 1; t >= dist; --t) {
    for (int i = 0; i < num; ++i) {
      if (i != id && get(t, i) == g) return t;
    }
  }
  return 0;
}

int Plan::getMaxConstraintTime(const int id, MAPF_Instance* P) const
{
  return getMaxConstraintTime(id, P->getStart(id), P->getGoal(id), P->getG());
}

void Plan::halt(const std::string& msg) const
{
  std::cout << "error@Plan: " << msg << std::endl;
  this->~Plan();
  std::exit(1);
}

void Plan::warn(const std::string& msg) const
{
  std::cout << "warn@Plan: " << msg << std::endl;
}
