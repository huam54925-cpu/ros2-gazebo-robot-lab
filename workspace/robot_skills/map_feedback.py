"""Map snapshots and measured information gain shared by investigation skills."""
import time
from explore import yaw
from observation import map_change


class MapFeedbackMixin:
    def grid_snapshot(self):
        m=self.latest['map']; o=m.info.origin
        return {'data':self.latest['data'].copy(),'resolution':m.info.resolution,
                'origin':(o.position.x,o.position.y,yaw(o.orientation)), 'frame':m.header.frame_id,
                'stamp_sim_s':m.header.stamp.sec+m.header.stamp.nanosec*1e-9}


    def sensor_offset(self):
        from safety_contract import BASE_FRAME, SCAN_FRAME
        from explore import rclpy
        try:
            t=self.buffer.lookup_transform(BASE_FRAME,SCAN_FRAME,rclpy.time.Time())
        except Exception as error:
            raise RuntimeError('lidar_transform_unavailable') from error
        return (t.transform.translation.x,t.transform.translation.y)

    def observation_feedback(self):
        at_stop=self.grid_snapshot(); self.save_map('gain_stopped')
        sim_start=self.node.get_clock().now().nanoseconds*1e-9; wall=time.monotonic(); fresh=True
        while time.monotonic()-wall<20 and self.node.get_clock().now().nanoseconds*1e-9-sim_start<4:
            self.spin_once()
            try: self.health()
            except RuntimeError: fresh=False; break
            if self.store.meta('stop_latched',False): break
            if self.task.get('mission_id'):
                m=self.store.meta('mission:'+self.task['mission_id'])
                if m['status']!='running' or time.monotonic()>m['deadline_monotonic_s']: break
        after=self.grid_snapshot(); self.save_map('gain_settled')
        metrics=map_change(self.metric_before,after,allow_fractional=True)
        metrics.update(at_stop=map_change(self.metric_before,at_stop,allow_fractional=True),settling=map_change(at_stop,after,allow_fractional=True),
                       feedback_wait_wall_s=time.monotonic()-wall,sensor_feedback_fresh=fresh,
                       map_message_advanced=after['stamp_sim_s']>self.metric_before['stamp_sim_s'])
        metrics['usable_for_trend']=bool(fresh and metrics['map_message_advanced'] and
            metrics.get('registration')=='integer_grid' and metrics.get('observed_lost_known_area_m2',float('inf'))<.05)
        metrics['causal_action_gain_valid']=False
        metrics['trend_scope']='registered_map_window_only_not_causal_action_gain'
        metrics['before_map_stamp_sim_s']=self.metric_before['stamp_sim_s']
        metrics['after_map_stamp_sim_s']=after['stamp_sim_s']
        metrics['before_geometry']={k:v for k,v in self.metric_before.items() if k!='data'}
        metrics['after_geometry']={k:v for k,v in after.items() if k!='data'}
        return metrics
