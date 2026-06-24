from enum import IntEnum


# ============================================================
# MISSION MODES
# ============================================================

class MissionMode(IntEnum):
    IDLE = 0
    TAKEOFF = 1
    TRANSITION_2_FW = 2
    WP_FLIGHT = 3
    TRANSITION_2_MC = 4
    RESCUE = 5
    INVERSE_WP_FLIGHT = 6
    DROP = 7
    LANDING = 8
    FINISHED = 9
    ABORT = 100


# ============================================================
# NODE STATES
# ============================================================

class NodeState(IntEnum):
    IDLE = 0b00
    BUSY = 0b01
    COMPLETED = 0b10
    ERROR = 0b11


# ============================================================
# NODE NAMES
# ============================================================

class NodeName(IntEnum):
    MASTER = 0
    WP_NAVIGATOR = 1
    TARGET_GUIDE = 2
    GRIPPER = 3
    VISION = 4
    LOGGER = 5


# ============================================================
# MISSION MANAGER
# ============================================================

class MissionManager:

    BITS_PER_NODESTATE = 2
    BITS_PER_NODE = 3

    MASK_NODESTATE = 0b11
    MASK_NODE = 0b111

    def __init__(self):

        self.data = 0

    # ========================================================
    # RAW ACCESS
    # ========================================================

    def set_raw(self, data: int):

        self.data = int(data)

    def raw(self):

        return self.data

    def clear(self):

        self.data = 0

    # ========================================================
    # AGGREGATE STATUS
    # ========================================================

    def set(
        self,
        node: NodeName,
        state: NodeState
    ):

        shift = (
            int(node)
            * self.BITS_PER_NODESTATE
        )

        self.data &= ~(
            self.MASK_NODESTATE << shift
        )

        self.data |= (
            int(state) << shift
        )

    def get(
        self,
        node: NodeName
    ):

        shift = (
            int(node)
            * self.BITS_PER_NODESTATE
        )

        value = (
            (self.data >> shift)
            & self.MASK_NODESTATE
        )

        return NodeState(value)

    # ========================================================
    # COMMAND PACKETS
    # ========================================================

    @staticmethod
    def pack(
        node: NodeName,
        state: NodeState
    ):

        node_bits = (
            int(node)
            & MissionManager.MASK_NODE
        )

        state_bits = (
            int(state)
            & MissionManager.MASK_NODESTATE
        )

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

    # ========================================================
    # HELPERS
    # ========================================================

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
