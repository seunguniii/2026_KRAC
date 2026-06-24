#include "stack_cpp/mission_manager.h"

MissionManager::MissionManager()
  : data(0)
  {}
  
void MissionManager::set(NodeName node, NodeState state){
  uint32_t shift = static_cast<uint32_t>(node)*BITS_PER_NODESTATE;

  data &= ~(MASK_NODESTATE << shift);
  data |= (static_cast<uint32_t>(state) << shift);
}

NodeState MissionManager::get(NodeName node) const {
  uint32_t shift = static_cast<uint32_t>(node)*BITS_PER_NODESTATE;
  
  return static_cast<NodeState>((data >> shift) & MASK_NODESTATE);
}

void MissionManager::clear() {
  data = 0;
}

uint32_t MissionManager::raw() const {
  return data;
}

uint32_t MissionManager::pack(NodeName node, NodeState state){
  uint32_t node_bits = static_cast<uint32_t>(node) & MASK_NODE;
  uint32_t state_bits = static_cast<uint32_t>(state) & MASK_NODESTATE;
  
  return (node_bits << BITS_PER_NODESTATE) | state_bits;
}

NodeName MissionManager::get_node(uint32_t cmd){
  return static_cast<NodeName>((cmd >> BITS_PER_NODESTATE) & MASK_NODE);
}

NodeState MissionManager::get_command(uint32_t cmd){
  return static_cast<NodeState>(cmd & MASK_NODESTATE);
}

//TODO
/*
bool isError(int node);
bool isFinished(int node);
void resetNode(int node);
int maxNodes() const;
*/
