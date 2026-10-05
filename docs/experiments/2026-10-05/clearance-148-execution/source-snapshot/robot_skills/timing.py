"""Exclusive executor phase intervals; ROS and wall clocks are never mixed."""
import time


class PhaseTimer:
    def __init__(self, clock, wall=time.monotonic):
        self.clock,self.wall=clock,wall
        self.rows=[]
        self.phase=None

    def mark(self, phase):
        wall,sim=self.wall(),self.clock()
        if self.phase is not None:
            self.rows.append({'phase':self.phase,'wall_s':wall-self.start_wall,
                              'sim_s':sim-self.start_sim if sim is not None and self.start_sim is not None and sim>=self.start_sim else None,
                              'start_sim_s':self.start_sim,'end_sim_s':sim})
        self.phase=phase
        self.start_wall,self.start_sim=wall,sim
