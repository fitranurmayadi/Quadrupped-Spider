import numpy as np
from src.spider_ik import leg_fk, leg_ik, L1, L2, L3, Z_OFFSET, LEG_ORIGINS, LEG_ROTATIONS

class GaitControllerV2:
    def __init__(self, step_length=0.06, step_height=0.025, frequency=2.5, body_z=-0.028, duty_factor=0.55):
        """
        GaitControllerV2: Diagonal trot gait generator where the neutral standing pose
        has the tibia exactly vertical (90 degrees relative to horizontal).
        
        duty_factor: Ratio of cycle time spent in stance. >0.5 creates a stable overlap phase.
        """
        self.step_length = step_length
        self.step_height = step_height
        self.frequency = frequency
        self.body_z = body_z
        self.duty_factor = duty_factor
        
        # Standing pose with EXACTLY vertical tibia (90 degrees)
        self.standing_pose = {
            'fl': (-0.5, -1.2, 2.7708),
            'rl': (0.5, -1.2, 2.7708),
            'fr': (0.5, -1.2, 2.7708),
            'rr': (-0.5, -1.2, 2.7708)
        }
        
        # Compute neutral foot positions (fallback / initial print)
        self.neutrals = {}
        for leg, angles in self.standing_pose.items():
            self.neutrals[leg] = leg_fk(leg, *angles)
            
        print("[GaitControllerV2] Neutral Foot Positions in Body Frame (Vertical Tibia):")
        for leg, pos in self.neutrals.items():
            print(f" - Leg {leg.upper()}: {pos}")

    def get_joint_targets(self, t, direction=(0.0, -1.0), yaw_rate=0.0):
        """
        Computes joint targets for all 12 joints at time t.
        """
        dx, dy = direction
        norm = np.hypot(dx, dy)
        speed_factor = max(norm, abs(yaw_rate))
        
        # Trot phase offsets for alternating leg groups
        leg_phases = {
            'fl': 0.0,
            'rr': 0.0,
            'rl': 0.5,
            'fr': 0.5
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
            
        # Dynamically calculate the horizontal distance r required to keep tibia perfectly vertical
        z_proj = self.body_z - Z_OFFSET
        arg = L2**2 - (z_proj + L3)**2
        r_dynamic = np.sqrt(arg) if arg >= 0 else 0.0
            
        for leg, phase_offset in leg_phases.items():
            # Calculate leg specific phase normalized to [0.0, 1.0]
            leg_phase = (phase_frac + phase_offset) % 1.0
            
            # Reconstruct local leg frame position with vertical tibia
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
            
            offset = np.zeros(3)
            
            if speed_factor > 0.01:
                # Dynamic gait scaling based on height (taller body = smaller steps, more overlap)
                # height goes from 2.8cm (body_z = -0.028) to 7.5cm (body_z = -0.075)
                height_val = -self.body_z
                height_ratio = np.clip((height_val - 0.028) / (0.075 - 0.028), 0.0, 1.0)
                
                # Scale step length and height down when taller to reduce roll/pitch perturbation
                active_step_length = self.step_length * speed_factor * (1.0 - 0.5 * height_ratio)
                active_step_height = self.step_height * speed_factor * (1.0 - 0.4 * height_ratio)
                
                # Increase duty factor (stance overlap) up to 0.60 when taller to increase support time
                active_duty_factor = self.duty_factor + 0.05 * height_ratio
                
                # Check if leg is in Stance or Swing phase based on active duty factor
                if leg_phase < active_duty_factor:
                    # 1. Stance Phase (leg on ground pushing body)
                    s = leg_phase / active_duty_factor  # 0.0 to 1.0
                    offset[0] = -dx * active_step_length * (s - 0.5)
                    offset[1] = -dy * active_step_length * (s - 0.5)
                    offset[2] = 0.0
                else:
                    # 2. Swing Phase (leg lifted and moving forward)
                    s = (leg_phase - active_duty_factor) / (1.0 - active_duty_factor)  # 0.0 to 1.0
                    # Cosine profile for smooth horizontal acceleration & deceleration
                    offset[0] = -dx * active_step_length * 0.5 * np.cos(np.pi * s)
                    offset[1] = -dy * active_step_length * 0.5 * np.cos(np.pi * s)
                    # Parabolic foot lift
                    offset[2] = active_step_height * np.sin(np.pi * s)
                    
                # Apply turning yaw rotation
                if abs(yaw_rate) > 0.01:
                    if leg_phase < active_duty_factor:
                        rot_angle = -yaw_rate * (leg_phase / active_duty_factor - 0.5) * self.step_length
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
