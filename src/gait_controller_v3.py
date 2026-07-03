import numpy as np
from src.spider_ik import leg_fk, leg_ik, L1, L2, L3, Z_OFFSET, LEG_ORIGINS, LEG_ROTATIONS

class GaitControllerV3:
    def __init__(self, step_length=0.05, step_height=0.020, frequency=1.5, body_z=-0.028, shift_gain=0.6):
        """
        GaitControllerV3: Crawl Gait (statically stable, 3-leg support, 1-leg swing).
        Includes dynamic CoM (Center of Mass) body shifting to keep the robot balanced.
        
        step_length: Stride length
        step_height: Swing height
        frequency: Crawling frequency (suggested 1.0 - 2.0 Hz for smooth motion)
        body_z: Target body height
        shift_gain: How aggressively to shift the body towards the support triangle (0.0 - 1.0)
        """
        self.step_length = step_length
        self.step_height = step_height
        self.frequency = frequency
        self.body_z = body_z
        self.shift_gain = shift_gain
        self.duty_factor = 0.75  # 75% stance, 25% swing per leg
        
        # Standing pose with vertical tibia
        self.standing_pose = {
            'fl': (-0.5, -1.2, 2.7708),
            'rl': (0.5, -1.2, 2.7708),
            'fr': (0.5, -1.2, 2.7708),
            'rr': (-0.5, -1.2, 2.7708)
        }
        
        # Compute neutral foot positions
        self.neutrals = {}
        for leg, angles in self.standing_pose.items():
            self.neutrals[leg] = leg_fk(leg, *angles)
            
        # State tracker for smoothed CoM body shift
        self.body_shift = np.zeros(2)
        
    def get_joint_targets(self, t, dt=1.0/240.0, direction=(0.0, -1.0), yaw_rate=0.0):
        """
        Computes joint targets for all 12 joints at time t.
        dt: Timestep size for smoothing body shifts.
        """
        dx, dy = direction
        norm = np.hypot(dx, dy)
        speed_factor = max(norm, abs(yaw_rate))
        
        # Wave sequence phase offsets (non-overlapping swing phases):
        # RL: [0.00, 0.25)
        # FR: [0.25, 0.50)
        # RR: [0.50, 0.75)
        # FL: [0.75, 1.00)
        leg_phases = {
            'fl': 0.75,
            'rr': 0.50,
            'fr': 0.25,
            'rl': 0.00
        }
        
        period = 1.0 / self.frequency
        cycle_time = t % period
        phase_frac = cycle_time / period
        
        targets = {}
        
        if norm > 1e-5:
            dx /= norm
            dy /= norm
        else:
            dx, dy = 0.0, 0.0
            
        # 1. Dynamically calculate vertical tibia footprint radius r
        z_proj = self.body_z - Z_OFFSET
        arg = L2**2 - (z_proj + L3)**2
        r_dynamic = np.sqrt(arg) if arg >= 0 else 0.0
        
        # 2. Determine which leg is currently swinging and calculate target CoM shift
        swinging_leg = None
        for leg, phase_offset in leg_phases.items():
            leg_phase = (phase_frac - phase_offset + 1.0) % 1.0
            if leg_phase >= self.duty_factor:  # in swing phase [0.75, 1.0)
                swinging_leg = leg
                break
                
        # Calculate target CoM shift based on support triangle centroid
        target_shift = np.zeros(2)
        if swinging_leg is not None and speed_factor > 0.01:
            support_legs = [l for l in leg_phases.keys() if l != swinging_leg]
            
            # Sum up neutral positions of the 3 support legs
            sum_pos = np.zeros(2)
            for leg in support_legs:
                # Dynamic splay horizontal neutral point
                standing_coxa = self.standing_pose[leg][0]
                x_local = L1 + r_dynamic
                y_local = 0.0
                
                # Apply coxa rotation (around local Z axis)
                x_rot = x_local * np.cos(standing_coxa) - y_local * np.sin(standing_coxa)
                y_rot = x_local * np.sin(standing_coxa) + y_local * np.cos(standing_coxa)
                
                # Apply leg base rotation to body frame
                rot = LEG_ROTATIONS[leg]
                x_body_local = x_rot * np.cos(rot) - y_rot * np.sin(rot)
                y_body_local = x_rot * np.sin(rot) + y_rot * np.cos(rot)
                
                # Dynamic neutral position in body frame
                body_neutral = LEG_ORIGINS[leg][:2] + np.array([x_body_local, y_body_local])
                sum_pos += body_neutral
                
            centroid = sum_pos / 3.0
            # Target shift is towards the support centroid scaled by shift_gain
            target_shift = centroid * self.shift_gain
            
        # Smoothly update current body shift using a first-order low-pass filter (tau = 0.08s)
        tau = 0.08
        self.body_shift += (target_shift - self.body_shift) * (dt / (tau + dt))
        
        # 3. Calculate foot positions and joint targets
        for leg, phase_offset in leg_phases.items():
            # Phase goes from 0.0 to 1.0 (relative to its own step start)
            leg_phase = (phase_frac - phase_offset + 1.0) % 1.0
            
            standing_coxa = self.standing_pose[leg][0]
            x_local = L1 + r_dynamic
            y_local = 0.0
            z_local = self.body_z
            
            # Apply coxa rotation (around local Z axis)
            x_rot = x_local * np.cos(standing_coxa) - y_local * np.sin(standing_coxa)
            y_rot = x_local * np.sin(standing_coxa) + y_local * np.cos(standing_coxa)
            z_rot = z_local
            
            # Apply leg base rotation to body frame
            rot = LEG_ROTATIONS[leg]
            x_body_local = x_rot * np.cos(rot) - y_rot * np.sin(rot)
            y_body_local = x_rot * np.sin(rot) + y_rot * np.cos(rot)
            z_body_local = z_rot
            
            neutral_pos = LEG_ORIGINS[leg] + np.array([x_body_local, y_body_local, z_body_local])
            
            # Shift body in opposite direction of the feet (feet shift by -body_shift)
            neutral_pos[0] -= self.body_shift[0]
            neutral_pos[1] -= self.body_shift[1]
            
            offset = np.zeros(3)
            
            if speed_factor > 0.01:
                # Dynamic gait scaling based on height (to ensure stability)
                height_val = -self.body_z
                height_ratio = np.clip((height_val - 0.028) / (0.075 - 0.028), 0.0, 1.0)
                
                active_step_length = self.step_length * speed_factor * (1.0 - 0.3 * height_ratio)
                active_step_height = self.step_height * speed_factor * (1.0 - 0.2 * height_ratio)
                
                if leg_phase < self.duty_factor:
                    # Stance Phase (other 3 legs moving backward relative to body)
                    s = leg_phase / self.duty_factor
                    offset[0] = -dx * active_step_length * (s - 0.5)
                    offset[1] = -dy * active_step_length * (s - 0.5)
                    offset[2] = 0.0
                else:
                    # Swing Phase (currently swinging leg moving forward in a smooth cosine curve)
                    s = (leg_phase - self.duty_factor) / (1.0 - self.duty_factor)
                    offset[0] = -dx * active_step_length * 0.5 * np.cos(np.pi * s)
                    offset[1] = -dy * active_step_length * 0.5 * np.cos(np.pi * s)
                    offset[2] = active_step_height * np.sin(np.pi * s)
                    
                # Apply turning yaw rotation
                if abs(yaw_rate) > 0.01:
                    if leg_phase < self.duty_factor:
                        rot_angle = -yaw_rate * (leg_phase / self.duty_factor - 0.5) * self.step_length
                    else:
                        rot_angle = yaw_rate * 0.5 * np.cos(np.pi * s) * self.step_length
                    
                    cos_r = np.cos(rot_angle)
                    sin_r = np.sin(rot_angle)
                    nx, ny = neutral_pos[0], neutral_pos[1]
                    neutral_pos[0] = nx * cos_r - ny * sin_r
                    neutral_pos[1] = nx * sin_r + ny * cos_r
            else:
                # Stand still
                pass
            
            foot_target = neutral_pos + offset
            c_target, f_target, t_target = leg_ik(leg, foot_target)
            
            targets[f'{leg}_coxa_joint'] = c_target
            targets[f'{leg}_femur_joint'] = f_target
            targets[f'{leg}_tibia_joint'] = t_target
            
        return targets
