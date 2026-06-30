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
  ABORT = 7
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
    uint32_t pack(NodeName node, NodeState state, MissionMode mode) const;	//master commands
    void set_mode(MissionMode mode);
    
    void clear();
    uint32_t raw() const;
    
    //used by slaves
    NodeName get_node(uint32_t cmd);
    NodeState get_command(uint32_t cmd);
    uint32_t pack(NodeName node, NodeState state) const;	//slaves report
    MissionMode get_mode(uint32_t cmd);
    
    //used by both
    NodeState get(NodeName node) const;				//slaves only use on themselves
    MissionMode get_mode() const;
    
  private:
    uint32_t data;
    static constexpr uint32_t NODE_COUNT = 8;
    
    static constexpr uint32_t BITS_PER_NODESTATE = 2;	//bits per NodeState
    static constexpr uint32_t BITS_PER_NODE = 3;	//bits per Node
    static constexpr uint32_t BITS_PER_MODE = 3;	//bits per MissionMode
    
    static constexpr uint32_t SHIFT_NODESTATE = 0;
    static constexpr uint32_t SHIFT_NODE = SHIFT_NODESTATE + BITS_PER_NODESTATE;
    static constexpr uint32_t SHIFT_MODE = SHIFT_NODE + BITS_PER_NODE;
    static constexpr uint32_t SHIFT_SUMMARY_MODE = NODE_COUNT * BITS_PER_NODESTATE;
    
    static constexpr uint32_t MASK_NODESTATE = (1u << BITS_PER_NODESTATE) - 1;
    static constexpr uint32_t MASK_NODE = (1u << BITS_PER_NODE) - 1;
    static constexpr uint32_t MASK_MODE = (1u << BITS_PER_MODE) - 1;
    
    static constexpr uint32_t MAX_NODES = 16;
};
