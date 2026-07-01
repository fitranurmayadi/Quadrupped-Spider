import os
import sys
import time
import argparse
import numpy as np
import pybullet as p
import pybullet_data

# Ensure we can import spider_ik and gait_controller
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from gait_controller import GaitController

def run_walk_demo(gui=True, target_distance=1.0, max_duration=30.0):
    print("==================================================")
    print(f"   DEMO JALAN 1 METER (IK-Based Gait Locomotion)  ")
    print("==================================================")

    # 1. Initialize PyBullet
    connection_mode = p.GUI if gui else p.DIRECT
    client = p.connect(connection_mode)
    
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    p.setGravity(0, 0, -9.81, physicsClientId=client)
    
    # Physics Timestep
    time_step = 1.0 / 240.0
    p.setTimeStep(time_step, physicsClientId=client)

    # 2. Load Plane & URDF
    plane_id = p.loadURDF("plane.urdf", physicsClientId=client)
    
    urdf_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "assets", "spider.urdf"))
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

    # Start position: splayed standing height is about 0.028, so spawn body at 0.05
    start_pos = [0, 0, 0.05]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    
    robot_id = p.loadURDF(
        urdf_path,
        basePosition=start_pos,
        baseOrientation=start_orientation,
        flags=p.URDF_USE_INERTIA_FROM_FILE,
        physicsClientId=client
    )

    # Configure Realistic Physics Parameters
    p.changeDynamics(plane_id, -1,
                     lateralFriction=0.8,
                     spinningFriction=0.05,
                     rollingFriction=0.02,
                     restitution=0.0,
                     physicsClientId=client)
                     
    num_joints = p.getNumJoints(robot_id, physicsClientId=client)
    joint_map = {}
    revolute_joints = []

    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i, physicsClientId=client)
        j_name = info[1].decode('utf-8')
        link_name = info[12].decode('utf-8')
        joint_type = info[2]
        
        joint_map[j_name] = i
        if joint_type == p.JOINT_REVOLUTE:
            revolute_joints.append(i)
            
        # Friction setting
        if 'foot' in link_name:
            p.changeDynamics(robot_id, i,
                             lateralFriction=1.5,
                             spinningFriction=0.3,
                             rollingFriction=0.1,
                             restitution=0.0,
                             contactStiffness=5000,
                             contactDamping=100,
                             physicsClientId=client)
        else:
            p.changeDynamics(robot_id, i,
                             lateralFriction=0.1,
                             spinningFriction=0.01,
                             rollingFriction=0.01,
                             physicsClientId=client)

    if gui:
        p.resetDebugVisualizerCamera(
            cameraDistance=0.5,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0.05],
            physicsClientId=client
        )

    # 3. Initialize Gait Controller
    # We want a stable walking height: body_z = -0.028 relative to body center.
    # Step length: 0.05 m, Height: 0.020 m, Freq: 2.5 Hz for smooth fast walk.
    gait = GaitController(step_length=0.05, step_height=0.020, frequency=2.5, body_z=-0.028)

    # Reset joint motors to default standing angles first
    for j_name, j_idx in joint_map.items():
        if j_idx in revolute_joints:
            # Neutral angles
            if 'fr_coxa_joint' in j_name or 'rl_coxa_joint' in j_name:
                angle = 0.5
            elif 'fl_coxa_joint' in j_name or 'rr_coxa_joint' in j_name:
                angle = -0.5
            elif 'femur' in j_name:
                angle = -1.2
            elif 'tibia' in j_name:
                angle = 2.8
            else:
                angle = 0.0
            p.resetJointState(robot_id, j_idx, targetValue=angle, targetVelocity=0, physicsClientId=client)

    # Let physics settle
    settle_steps = int(0.5 / time_step)
    for _ in range(settle_steps):
        p.stepSimulation(physicsClientId=client)

    print("[INFO] Starting walk loop...")
    start_time = time.time()
    sim_time = 0.0
    
    initial_pos, _ = p.getBasePositionAndOrientation(robot_id, physicsClientId=client)
    initial_y = initial_pos[1]
    
    last_print_dist = 0.0
    reached = False
    
    # Proportional controller gain for Yaw direction correction (to keep path straight)
    Kp_yaw = 0.8
    
    try:
        while sim_time < max_duration:
            pos, orn = p.getBasePositionAndOrientation(robot_id, physicsClientId=client)
            euler = p.getEulerFromQuaternion(orn)
            current_yaw = euler[2]
            
            # Distance walked (forward is -Y direction)
            dist_walked = initial_y - pos[1]
            
            # Progress print every 0.1m
            if dist_walked - last_print_dist >= 0.1:
                print(f"Progress: Walked {dist_walked:.2f} m | Yaw: {current_yaw:+.3f} rad | Height: {pos[2]:.3f} m")
                last_print_dist = dist_walked
                
            # Check target condition
            if dist_walked >= target_distance:
                reached = True
                print(f"\n[SUCCESS] Target distance of {target_distance}m reached!")
                break
                
            # Yaw correction rate: if yaw drifts right (negative), turn left (positive)
            yaw_corr = -Kp_yaw * current_yaw
            
            # Get joint targets from gait controller
            # Target direction is forward (0, -1) in local/world frame
            joint_targets = gait.get_joint_targets(sim_time, direction=(0.0, -1.0), yaw_rate=yaw_corr)
            
            # Apply to motors
            for j_name, target_angle in joint_targets.items():
                if j_name in joint_map:
                    p.setJointMotorControl2(
                        bodyUniqueId=robot_id,
                        jointIndex=joint_map[j_name],
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=target_angle,
                        force=8.0,
                        maxVelocity=4.0,
                        physicsClientId=client
                    )
            
            p.stepSimulation(physicsClientId=client)
            sim_time += time_step
            
            if gui:
                # Keep camera tracking the robot
                p.resetDebugVisualizerCamera(
                    cameraDistance=0.5,
                    cameraYaw=45,
                    cameraPitch=-30,
                    cameraTargetPosition=[pos[0], pos[1], pos[2]],
                    physicsClientId=client
                )
                time.sleep(time_step)

    except KeyboardInterrupt:
        print("\n[INFO] Walk demo stopped by user.")
    
    # Final stats
    end_pos, end_orn = p.getBasePositionAndOrientation(robot_id, physicsClientId=client)
    end_euler = p.getEulerFromQuaternion(end_orn)
    elapsed_time = time.time() - start_time
    total_dist = initial_y - end_pos[1]
    
    print("\n--------------------------------------------------")
    print("                WALK DEMO RESULTS                 ")
    print("--------------------------------------------------")
    print(f"Status               : {'SUCCESS' if reached else 'TIMEOUT/FAILED'}")
    print(f"Total Distance Walked: {total_dist:.4f} m")
    print(f"Simulation Time      : {sim_time:.2f} s")
    print(f"Real Time Elapsed    : {elapsed_time:.2f} s")
    print(f"Average Speed        : {total_dist / sim_time:.4f} m/s")
    print(f"Final Body Position  : [{end_pos[0]:.4f}, {end_pos[1]:.4f}, {end_pos[2]:.4f}]")
    print(f"Final Body Yaw       : {end_euler[2]:.4f} rad")
    print("--------------------------------------------------\n")
    
    p.disconnect(physicsClientId=client)
    return reached

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="IK-Based Gait walking 1 meter demo.")
    parser.add_argument("--headless", action="store_true", help="Run without GUI")
    parser.add_argument("--distance", type=float, default=1.0, help="Target walking distance in meters")
    args = parser.parse_args()
    
    run_walk_demo(gui=not args.headless, target_distance=args.distance)
