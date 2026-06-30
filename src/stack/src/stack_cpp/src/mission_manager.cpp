#include "stack_cpp/mission_manager.h"
#include <iostream>

MissionManager::MissionManager()
  : data(0)
  {}

//sets node state
void MissionManager::set(NodeName node, NodeState state) {
  uint32_t shift =  static_cast<uint32_t>(node) * BITS_PER_NODESTATE;

  data &= ~(MASK_NODESTATE << shift);
  data |= ((static_cast<uint32_t>(state) & MASK_NODESTATE) << shift);
}

void MissionManager::set_mode(MissionMode mode) {
  data &= ~(MASK_MODE << SHIFT_SUMMARY_MODE);
  data |= (static_cast<uint32_t>(mode) & MASK_MODE) << SHIFT_SUMMARY_MODE;
}

MissionMode MissionManager::get_mode() const {
  return static_cast<MissionMode>((data >> SHIFT_SUMMARY_MODE) & MASK_MODE);
}

//gets node state
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


//pack command
uint32_t MissionManager::pack(NodeName node, NodeState state, MissionMode mode) const {
  uint32_t packet = pack(node, state);
  
  packet |= (static_cast<uint32_t>(mode) & MASK_MODE) << SHIFT_MODE;

  return packet;
}


//pack status 
uint32_t MissionManager::pack(NodeName node, NodeState state) const {
  uint32_t packet = 0;

  packet |= (static_cast<uint32_t>(state) & MASK_NODESTATE) << SHIFT_NODESTATE;
  packet |= (static_cast<uint32_t>(node) & MASK_NODE) << SHIFT_NODE;

  return packet;
}


//gets target node from command
NodeName MissionManager::get_node(uint32_t cmd){
  return static_cast<NodeName>((cmd >> SHIFT_NODE) & MASK_NODE);
}

//gets command from command
NodeState MissionManager::get_command(uint32_t cmd){
  return static_cast<NodeState>((cmd >> SHIFT_NODESTATE) & MASK_NODESTATE);
}

//gets mission mode from command
MissionMode MissionManager::get_mode(uint32_t cmd){
  return static_cast<MissionMode>((cmd >> SHIFT_MODE) & MASK_MODE);
}

//TODO
/*
bool isError(int node);
bool isFinished(int node);
void resetNode(int node);
int maxNodes() const;
*/
