from enum import IntEnum


class MissionMode(IntEnum):
    IDLE = 0
    WP_FLIGHT = 1
    RESCUE = 2
    INVERSE_WP_FLIGHT = 3
    DROP = 4
    LANDING = 5
    FINISHED = 6
    ABORT = 7


class NodeState(IntEnum):
    IDLE = 0b00
    BUSY = 0b01
    SUCCESS = 0b10
    ABORT = 0b11


class NodeName(IntEnum):
    MISSION = 0
    FLIGHT = 1
    TARGET = 2
    GRIPPER = 3
    VISION = 4
    MARKER = 5
    YOLO = 6
    LOGGER = 7


class MissionManager:
    NODE_COUNT = 8

    BITS_PER_NODESTATE = 2
    BITS_PER_NODE = 3
    BITS_PER_MODE = 3
    
    SHIFT_NODESTATE = 0
    SHIFT_NODE = SHIFT_NODESTATE + BITS_PER_NODESTATE
    SHIFT_MODE = SHIFT_NODE + BITS_PER_NODE
    SHIFT_SUMMARY_MODE = BITS_PER_NODESTATE * NODE_COUNT

    MASK_NODESTATE = (1 << BITS_PER_NODESTATE) - 1
    MASK_NODE = (1 << BITS_PER_NODE) - 1
    MASK_MODE = (1 << BITS_PER_MODE) - 1


    def __init__(self):
        self.data = 0


    def set_raw(self, data: int):
        self.data = int(data)


    def raw(self):
        return self.data


    def clear(self):
        self.data = 0


    def set_mode(self, mode: MissionMode):
        self.data &= ~(self.MASK_MODE << self.SHIFT_SUMMARY_MODE)
        self.data |= (int(mode) & self.MASK_MODE) << self.SHIFT_SUMMARY_MODE
        
     
    def get_summary_mode(self):
        return MissionMode((self.data >> self.SHIFT_SUMMARY_MODE) & self.MASK_MODE)


    def set(self, node: NodeName, state: NodeState):
        shift = (int(node) * self.BITS_PER_NODESTATE)
        self.data &= ~(self.MASK_NODESTATE << shift)
        self.data |= ((int(state) & self.MASK_NODESTATE) << shift)


    def get(self, node: NodeName):
        shift = (int(node) * self.BITS_PER_NODESTATE)
        value = ((self.data >> shift) & self.MASK_NODESTATE)

        return NodeState(value)


    @staticmethod
    def pack(node: NodeName, state: NodeState, mode: MissionMode | None = None):
        packet = 0
        
        packet |= (
            (int(state) & MissionManager.MASK_NODESTATE)
            << MissionManager.SHIFT_NODESTATE
        )
        packet |= (
            (int(node) & MissionManager.MASK_NODE)
            << MissionManager.SHIFT_NODE
        )
        
        if mode is not None:
            packet |= (
                (int(mode) & MissionManager.MASK_MODE)
                << MissionManager.SHIFT_MODE
            )

        return packet


    @staticmethod
    def get_node(cmd: int):
        value = (
            (cmd >> MissionManager.SHIFT_NODE)
            & MissionManager.MASK_NODE
        )

        return NodeName(value)


    @staticmethod
    def get_command(cmd: int):
        value = (
            (cmd >> MissionManager.SHIFT_NODESTATE)
            & MissionManager.MASK_NODESTATE
        )

        return NodeState(value)

    @staticmethod
    def get_mode(cmd: int):
        value = (
            (cmd >> MissionManager.SHIFT_MODE)
            & MissionManager.MASK_MODE
        )

        return MissionMode(value)

    def is_idle(self, node: NodeName):
        return (
            self.get(node)
            == NodeState.IDLE
        )

    def is_busy(self, node: NodeName):
        return (
            self.get(node)
            == NodeState.BUSY
        )

    def is_success(self, node: NodeName):
        return (
            self.get(node)
            == NodeState.SUCCESS
        )

    def is_abort(self, node: NodeName):
        return (
            self.get(node)
            == NodeState.ABORT
        )
