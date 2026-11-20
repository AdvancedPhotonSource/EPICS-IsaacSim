"""EPICS / Bluesky adapter layer for NVIDIA Isaac Sim digital twins.

Exposes an Isaac Sim articulation to the control system as motor records, so
that existing scan code, ophyd device definitions and Phoebus screens drive the
twin unmodified -- the experiment is run, not reimplemented.

See ``EPICS-INTEGRATION-CONTEXT.md`` for the architecture decisions this
implements, and ``docs/`` for protocol and deployment notes.
"""

from .config import AdapterConfig, AxisConfig, ConfigError, default_config
from .motor import AxisOutputs, MotorAxis, Phase, Severity
from .plant import Plant, PlantState, SharedPlant
from .protocol import PROTOCOL_VERSION, ProtocolError, SimFrame, TargetFrame

__version__ = "0.1.0"

__all__ = [
    "AdapterConfig",
    "AxisConfig",
    "AxisOutputs",
    "ConfigError",
    "MotorAxis",
    "PROTOCOL_VERSION",
    "Phase",
    "Plant",
    "PlantState",
    "ProtocolError",
    "Severity",
    "SharedPlant",
    "SimFrame",
    "TargetFrame",
    "__version__",
    "default_config",
]
