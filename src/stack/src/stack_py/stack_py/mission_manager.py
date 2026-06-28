from enum import IntEnum


class MissionMode(IntEnum):
    IDLE = 0
    WP_FLIGHT = 1
    RESCUE = 2
    INVERSE_WP_FLIGHT = 3
    DROP = 4
    LANDING = 5
    FINISHED = 6
    ABORT = 100


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
    BITS_PER_NODESTATE = 2
    BITS_PER_NODE = 3

    MASK_NODESTATE = 0b11
    MASK_NODE = 0b111


    def __init__(self):
        self.data = 0


    def set_raw(self, data: int):
        self.data = int(data)


    def raw(self):
        return self.data


    def clear(self):
        self.data = 0


    def set(self, node: NodeName, state: NodeState):

        shift = (int(node) * self.BITS_PER_NODESTATE)
        self.data &= ~(self.MASK_NODESTATE << shift)
        self.data |= (int(state) << shift)


    def get(self, node: NodeName):
        shift = (int(node) * self.BITS_PER_NODESTATE)
        value = (self.data >> shift) & self.MASK_NODESTATE

        return NodeState(value)


    @staticmethod
    def pack(node: NodeName, state: NodeState):
        node_bits = (int(node) & MissionManager.MASK_NODE)
        state_bits = (int(state) & MissionManager.MASK_NODESTATE)

        return (
            (node_bits << MissionManager.BITS_PER_NODESTATE)
            | state_bits
        )


    @staticmethod
    def get_node(cmd: int):
        value = (
            (cmd >> MissionManager.BITS_PER_NODESTATE)
            & MissionManager.MASK_NODE
        )

        return NodeName(value)


    @staticmethod
    def get_command(cmd: int):
        value = (
            cmd
            & MissionManager.MASK_NODESTATE
        )

        return NodeState(value)


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

    def is_completed(self, node: NodeName):
        return (
            self.get(node)
            == NodeState.COMPLETED
        )

    def is_error(self, node: NodeName):
        return (
            self.get(node)
            == NodeState.ERROR
        )
