"""Raw synchronous frame boundary; policy history belongs only to the environment."""
from dataclasses import dataclass, field
from typing import Protocol
import math
from numbers import Real

SCHEMA_VERSION = 'local-drive-v2'

class InfrastructureError(RuntimeError):
    """Discard this transition; stop and rebuild the environment."""

@dataclass(frozen=True)
class Frame:
    episode_id: int
    action_id: int
    sim_time: float
    sensor_times: dict[str, float]
    observation: dict
    evaluation: dict
    odom_pose: tuple = ()
    ack: dict = field(default_factory=dict)

class GazeboBackend(Protocol):
    def reset(self, *, seed: int, scenario: str) -> Frame: ...
    def advance(self, v: float, w: float, dt: float) -> Frame: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...

def decode_action(action):
    if not hasattr(action, '__len__') or len(action) != 2 or any(not isinstance(x, Real) for x in action):
        raise ValueError('Action must have exactly two components')
    a = [float(x) for x in action]
    if not all(math.isfinite(x) for x in a):
        raise ValueError('Action must be finite')
    a = [max(-1., min(1., x)) for x in a]
    return 0.2 + 0.4*a[0], 0.8*a[1]

def validate_transition(previous: Frame, current: Frame, *, dt=0.2, tolerance=1e-6):
    """Require post-action samples; backend must separately guarantee action ACK."""
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError('dt must be positive')
    if current.episode_id != previous.episode_id:
        raise InfrastructureError('Cross-episode observation')
    if current.action_id != previous.action_id + 1:
        raise InfrastructureError('Missing or duplicate action acknowledgement')
    target = previous.sim_time + dt
    if not math.isfinite(current.sim_time) or abs(current.sim_time-target) > tolerance:
        raise InfrastructureError('Incorrect simulation advance')
    for key in ('lidar', 'odom', 'truth'):
        stamp = current.sensor_times.get(key, float('nan'))
        if not math.isfinite(stamp) or not target-tolerance <= stamp <= current.sim_time+tolerance:
            raise InfrastructureError(f'Stale or invalid {key} sample')
    if set(current.observation) != {'lidar', 'odom_velocity', 'goal_relative'}:
        raise InfrastructureError('Unexpected policy inputs; truth leakage or missing field')

def encode_lidar(ranges, *, sectors=90, range_min=0.08, range_max=10.0):
    """Each sector => [normalized nearest hit, hit_fraction, no_return_fraction].

    +inf is no return; NaN/-inf/out-of-range finite readings are invalid.
    Invalid fraction is 1-hit_fraction-no_return_fraction. Never turn NaN into free space.
    """
    if sectors < 1 or len(ranges) == 0 or len(ranges) % sectors or not 0 <= range_min < range_max:
        raise ValueError('Invalid scan geometry')
    width = len(ranges)//sectors
    output=[]
    for start in range(0,len(ranges),width):
        block=[float(x) for x in ranges[start:start+width]]
        hits=[x for x in block if math.isfinite(x) and range_min <= x <= range_max]
        output.extend([min(hits)/range_max if hits else 1., len(hits)/width,
                       sum(x == math.inf for x in block)/width])
    return output
