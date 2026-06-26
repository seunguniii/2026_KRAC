#pragma once

#include <cstdint>

enum class MissionMode {
  IDLE = 0,
  WP_FLIGHT = 1,
  RESCUE = 2,
  INVERSE_WP_FLIGHT = 3,
  DROP = 4,
  LANDING = 5,
  FINISHED = 6,
  ABORT = 100
};
    
enum class NodeState : uint32_t {
  IDLE = 0b00,
  BUSY = 0b01,
  SUCCESS = 0b10,
  ABORT = 0b11
};
    
enum class NodeName {
  MISSION = 0,
  FLIGHT = 1,
  TARGET = 2,
  GRIPPER = 3,
  VISION = 4,
  MARKER = 5,
  YOLO = 6,
  LOGGER = 7
};

class MissionManager{
  public:
    MissionManager();
    
    //used by master
    void set(NodeName node, NodeState state);
    
    //used by slaves
    NodeName get_node(uint32_t cmd);
    NodeState get_command(uint32_t cmd);
    
    //used by both
    uint32_t pack(NodeName node, NodeState state);	//master commands, slaves report
    NodeState get(NodeName node) const;			//slaves only use on themselves
    
    void clear();
    uint32_t raw() const;
    
  private:
    uint32_t data;
    
    static constexpr uint32_t BITS_PER_NODESTATE = 2;	//bits per NodeState
    static constexpr uint32_t BITS_PER_NODE = 3;	//bits per Node
    static constexpr uint32_t MASK_NODESTATE = 0b11;	//2^(bits per NodeState) - 1
    static constexpr uint32_t MASK_NODE = 0b111;	//2^(bits per Node) - 1
    static constexpr uint32_t MAX_NODES = 16;
};
