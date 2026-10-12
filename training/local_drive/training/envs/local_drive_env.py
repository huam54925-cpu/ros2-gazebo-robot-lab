"""295-dimensional policy observations; this is the only history owner."""
from collections import deque
import math
import gymnasium as gym
import numpy as np
from .contracts import InfrastructureError, decode_action, encode_lidar
from .gazebo_backend import GazeboBackendImpl

class LocalDriveEnv(gym.Env):
    metadata={'render_modes':[]}
    def __init__(self,backend=None):
        super().__init__()
        self.backend=backend if backend is not None else GazeboBackendImpl()
        self.action_space=gym.spaces.Box(-1.,1.,(2,),np.float32)
        low=np.concatenate([np.zeros(270),-np.ones(2),[0,-1,-1],-np.ones(20)]).astype(np.float32)
        self.observation_space=gym.spaces.Box(low,np.ones(295,np.float32),dtype=np.float32)
        self.history=deque(maxlen=4);self.motion=deque();self.prev_action=np.zeros(2,dtype=np.float32)
        self.last_frame=None;self.episode_start_time=None;self.flip_since=None;self.done=True
    def reset(self,*,seed=None,options=None):
        super().reset(seed=seed)
        options=options or {}
        if set(options)-{'scenario'}:raise ValueError('Unknown reset option')
        # Gym's RNG is retained and advanced when seed=None. Explicit seeds restart it.
        backend_seed=int(self.np_random.integers(0,2**31,dtype=np.uint32))
        self.done=True;self.history.clear();self.motion.clear();self.prev_action=np.zeros(2,dtype=np.float32);self.flip_since=None
        try:
            frame=self.backend.reset(seed=backend_seed,scenario=options.get('scenario','indoor_fixed'))
            self._check_evaluation(frame)
            self.last_frame=frame;self.episode_start_time=frame.sim_time
            obs=self._observation(frame);self.done=False
            return obs,{'episode_id':frame.episode_id,'backend_seed':backend_seed,'sim_time':frame.sim_time}
        except InfrastructureError as e:self._fault(e)
    @staticmethod
    def _check_evaluation(frame):
        try:
            if frame.evaluation['truth_dist']<0 or not all(math.isfinite(x) for x in [frame.evaluation['truth_dist'],frame.evaluation['roll'],frame.evaluation['pitch'],*frame.evaluation['truth_pose']]):
                raise ValueError('invalid evaluation values')
        except (KeyError,TypeError,ValueError) as e:raise InfrastructureError('Missing/invalid required truth data') from e
    def _observation(self,frame):
        try:
            lidar=frame.observation['lidar']
            encoded=encode_lidar(lidar['ranges'],range_min=lidar['range_min'],range_max=lidar['range_max'])
            vel=np.clip(np.asarray(frame.observation['odom_velocity'])/[.6,.8],-1,1)
            hist=np.zeros((4,5),dtype=np.float32)
            if self.history:hist[-len(self.history):]=list(self.history)
            obs=np.concatenate([encoded,vel,frame.observation['goal_relative'],hist.ravel()]).astype(np.float32)
            if not self.observation_space.contains(obs):raise ValueError('outside observation space')
            return obs
        except (KeyError,ValueError,TypeError) as e:raise InfrastructureError('Invalid policy observation: '+str(e)) from e
    def step(self,action):
        if self.done or self.last_frame is None:raise RuntimeError('reset required before step')
        v,w=decode_action(action);action=np.clip(np.asarray(action,dtype=np.float32),-1.,1.)
        before=self.last_frame
        try:
            frame=self.backend.advance(v,w,dt=.2);self._check_evaluation(frame)
            px,py,pyaw=before.odom_pose;x,y,yaw=frame.odom_pose
            dx,dy=x-px,y-py; c,s=math.cos(pyaw),math.sin(pyaw)
            dyaw=math.atan2(math.sin(yaw-pyaw),math.cos(yaw-pyaw))
            self.history.append(np.clip([action[0],action[1],(c*dx+s*dy)/.12,(-s*dx+c*dy)/.12,dyaw/.16],-1,1))
            truth=frame.evaluation['truth_pose'];old_truth=before.evaluation['truth_pose']
            distance=math.hypot(truth[0]-old_truth[0],truth[1]-old_truth[1])
            self.motion.append((frame.sim_time,distance,abs(v)))
            while self.motion and self.motion[0][0]<=frame.sim_time-2.+1e-8:self.motion.popleft()
            stalled=len(self.motion)>=10 and sum(x[1] for x in self.motion)<.05 and sum(x[2] for x in self.motion)/len(self.motion)>.05
            flipped=max(abs(frame.evaluation[k]) for k in ('roll','pitch'))>.8
            if flipped:
                if self.flip_since is None:self.flip_since=frame.sim_time
            else:self.flip_since=None
            arrival=frame.evaluation['truth_dist']<=.35
            terminated=arrival or (self.flip_since is not None and frame.sim_time-self.flip_since>=.5-1e-8)
            truncated=frame.sim_time-self.episode_start_time>=120.-1e-8
            reward=2*(before.evaluation['truth_dist']-frame.evaluation['truth_dist'])-.05*.2-.2*stalled-.01*float(np.sum((action-self.prev_action)**2))+10*arrival
            obs=self._observation(frame) # history already includes this transition
            self.last_frame=frame;self.prev_action=action.copy();self.done=bool(terminated or truncated)
            if self.done:self.backend.stop()
            return obs,float(reward),bool(terminated),bool(truncated),{'action_id':frame.action_id,'sim_time':frame.sim_time,'elapsed':frame.sim_time-self.episode_start_time,'stalled':bool(stalled)}
        except InfrastructureError as e:self._fault(e)
    def _fault(self,error):
        self.done=True
        try:self.backend.close()
        except Exception as cleanup:error.add_note('cleanup: '+repr(cleanup))
        raise error
    def close(self):
        self.done=True
        self.backend.close()
