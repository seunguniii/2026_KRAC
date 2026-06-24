#pragma once

#include <cstdint>

enum class MissionMode {
  IDLE = 0,
  TAKEOFF = 1,
  TRANSITION_2_FW = 2,
  WP_FLIGHT = 3,
  TRANSITION_2_MC = 4,
  RESCUE = 5,
  //TRANSITION_2_FW = 1,
  INVERSE_WP_FLIGHT = 6,
  //TRANSITION_2_MC = 3,
  DROP = 7,
  LANDING = 8,
  FINISHED = 9,
  ABORT = 100
};
    
enum class NodeState : uint32_t {
  IDLE = 0b00,
  BUSY = 0b01,
  COMPLETED = 0b10,
  ERROR = 0b11
};
    
enum class NodeName {
  MASTER = 0,
  WP_NAVIGATOR = 1,
  TARGET_GUIDE = 2,
  GRIPPER = 3,
  VISION = 4,
  LOGGER = 5
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
