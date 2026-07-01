import numpy as np
from src.spider_ik import leg_fk, leg_ik

class GaitController:
    def __init__(self, step_length=0.06, step_height=0.030, frequency=2.0, body_z=-0.0177):
        """
        gait_controller: Generates foot trajectories and joint targets for the quadruped.
        
        step_length: Length of each stride in meters
        step_height: Height of foot lift during swing in meters
        frequency: Walking frequency in Hz
        body_z: Target vertical position of feet relative to the body center (neutral is -0.0177)
        """
        self.step_length = step_length
        self.step_height = step_height
        self.frequency = frequency
        self.body_z = body_z
        
        # Default joint angles for standing pose
        self.standing_pose = {
            'fl': (-0.5, -1.2, 2.8),
            'rl': (0.5, -1.2, 2.8),
            'fr': (0.5, -1.2, 2.8),
            'rr': (-0.5, -1.2, 2.8)
        }
        
        # Compute neutral foot positions from default standing angles
        self.neutrals = {}
        for leg, angles in self.standing_pose.items():
            self.neutrals[leg] = leg_fk(leg, *angles)
            
        print("[GaitController] Neutral Foot Positions in Body Frame:")
        for leg, pos in self.neutrals.items():
            print(f" - Leg {leg.upper()}: {pos}")

    def get_joint_targets(self, t, direction=(0.0, -1.0), yaw_rate=0.0):
        """
        Computes joint targets for all 12 joints at time t.
        direction: (dx, dy) direction vector in local frame.
        yaw_rate: turning rate.
        """
        dx, dy = direction
        norm = np.hypot(dx, dy)
        
        # Determine speed factor based on translation and rotation commands
        speed_factor = max(norm, abs(yaw_rate))
        
        # Trot gait groups:
        # Group 1: FL, RR (phase 0)
        # Group 2: RL, FR (phase 0.5)
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
        
        # Normalize direction vector if moving
        if norm > 1e-5:
            dx /= norm
            dy /= norm
        else:
            dx, dy = 0.0, 0.0
            
        for leg, phase_offset in leg_phases.items():
            # Leg specific phase
            leg_phase = (phase_frac + phase_offset) % 1.0
            
            neutral_pos = self.neutrals[leg].copy()
            # We want to override the Z target to our target body height
            neutral_pos[2] = self.body_z
            
            # Trajectory offset
            offset = np.zeros(3)
            
            if speed_factor > 0.01:
                # Calculate active step parameters based on speed factor
                active_step_length = self.step_length * speed_factor
                active_step_height = self.step_height * speed_factor
                
                if leg_phase < 0.5:
                    # 1. Stance Phase (contact with ground)
                    # Moves backward relative to body to push body forward
                    s = leg_phase / 0.5  # 0.0 to 1.0
                    offset[0] = -dx * active_step_length * (s - 0.5)
                    offset[1] = -dy * active_step_length * (s - 0.5)
                    offset[2] = 0.0
                else:
                    # 2. Swing Phase (lifted)
                    # Moves forward relative to body to prepare for next step
                    s = (leg_phase - 0.5) / 0.5  # 0.0 to 1.0
                    offset[0] = dx * active_step_length * (s - 0.5)
                    offset[1] = dy * active_step_length * (s - 0.5)
                    # Parabolic foot lift
                    offset[2] = active_step_height * np.sin(np.pi * s)
                    
                # Apply turning yaw rotation to foot positions if turning
                if abs(yaw_rate) > 0.01:
                    # Yaw rotation differential angle based on phase
                    if leg_phase < 0.5:
                        rot_angle = -yaw_rate * (leg_phase / 0.5 - 0.5) * self.step_length
                    else:
                        rot_angle = yaw_rate * ((leg_phase - 0.5) / 0.5 - 0.5) * self.step_length
                    
                    # Apply rotation around body Z axis to the neutral point
                    cos_r = np.cos(rot_angle)
                    sin_r = np.sin(rot_angle)
                    nx, ny = neutral_pos[0], neutral_pos[1]
                    neutral_pos[0] = nx * cos_r - ny * sin_r
                    neutral_pos[1] = nx * sin_r + ny * cos_r
            else:
                # Stand still
                pass
            
            foot_target = neutral_pos + offset
            
            # Compute IK
            c_target, f_target, t_target = leg_ik(leg, foot_target)
            
            # Store in target map
            targets[f'{leg}_coxa_joint'] = c_target
            targets[f'{leg}_femur_joint'] = f_target
            targets[f'{leg}_tibia_joint'] = t_target
            
        return targets
