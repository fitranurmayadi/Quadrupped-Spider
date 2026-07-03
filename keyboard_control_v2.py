import os
import sys
import time
import numpy as np
import pybullet as p
import pybullet_data

# Ensure we can import spider_ik and gait_controller_v2
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from src.gait_controller_v2 import GaitControllerV2

def main():
    print("==================================================")
    print("   INTERACTIVE KEYBOARD CONTROL V2 (VERT. TIBIA)  ")
    print("==================================================")
    print("Control Instructions:")
    print("  - W / UP ARROW    : Move Forward")
    print("  - S / DOWN ARROW  : Move Backward")
    print("  - A / LEFT ARROW  : Move Left (Lateral)")
    print("  - D / RIGHT ARROW : Move Right (Lateral)")
    print("  - Q               : Turn Left")
    print("  - E               : Turn Right")
    print("  - Z / X           : Raise / Lower Body Height")
    print("  - R               : Reset position")
    print("  - ESC             : Exit simulation")
    print("==================================================")

    # 1. Initialize PyBullet GUI
    client = p.connect(p.GUI)
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

    # 3. Initialize Gait Controller V2 (Tibia standing angle = 2.7708)
    gait = GaitControllerV2(step_length=0.06, step_height=0.025, frequency=3.5, body_z=-0.028)

    # Function to reset joints to splayed standing pose (with exactly vertical tibia)
    def reset_pose():
        p.resetBasePositionAndOrientation(robot_id, start_pos, start_orientation, physicsClientId=client)
        p.resetBaseVelocity(robot_id, [0, 0, 0], [0, 0, 0], physicsClientId=client)
        for j_name, j_idx in joint_map.items():
            if j_idx in revolute_joints:
                if 'fr_coxa_joint' in j_name or 'rl_coxa_joint' in j_name:
                    angle = 0.5
                elif 'fl_coxa_joint' in j_name or 'rr_coxa_joint' in j_name:
                    angle = -0.5
                elif 'femur' in j_name:
                    angle = -1.2
                elif 'tibia' in j_name:
                    angle = 2.7708  # 90 degrees vertical tibia
                else:
                    angle = 0.0
                p.resetJointState(robot_id, j_idx, targetValue=angle, targetVelocity=0, physicsClientId=client)

    reset_pose()

    # Add instructions text overlay in PyBullet GUI
    instruction_text = (
        "Controls (V2 - Vertical Tibia):\n"
        "  W / S / A / D or Arrows : Translate\n"
        "  Q / E : Turn Left / Right (Fast)\n"
        "  Z / X : Raise / Lower Body Height\n"
        "  R : Reset Robot Position\n"
        "  ESC : Quit"
    )
    p.addUserDebugText(
        text=instruction_text,
        textPosition=[-0.18, -0.18, 0.15],
        textColorRGB=[1.0, 1.0, 1.0],
        textSize=1.2,
        lifeTime=0,
        physicsClientId=client
    )

    # Dynamic status overlay for height
    status_text_id = p.addUserDebugText(
        text=f"Body Height: {-gait.body_z * 100:.1f} cm",
        textPosition=[-0.18, -0.18, 0.13],
        textColorRGB=[0.0, 1.0, 1.0],
        textSize=1.3,
        lifeTime=0,
        physicsClientId=client
    )

    # Add a visual ground grid for orientation reference
    for i in range(-5, 6):
        p.addUserDebugLine([i, -5, 0.001], [i, 5, 0.001], [0.5, 0.5, 0.5], 1, 0, physicsClientId=client)
        p.addUserDebugLine([-5, i, 0.001], [5, i, 0.001], [0.5, 0.5, 0.5], 1, 0, physicsClientId=client)

    sim_time = 0.0
    
    # Camera settings
    cam_distance = 0.5
    cam_pitch = -89.9  # Top-down view
    
    print("[INFO] Simulation active! Focus the PyBullet window to control the robot.")

    try:
        while p.isConnected(physicsClientId=client):
            keys = p.getKeyboardEvents(physicsClientId=client)
            
            if 27 in keys and keys[27] & 1:
                break
                
            if ord('r') in keys and keys[ord('r')] & 1:
                reset_pose()
                gait.body_z = -0.028
                sim_time = 0.0
                status_text_id = p.addUserDebugText(
                    text=f"Body Height: {-gait.body_z * 100:.1f} cm",
                    textPosition=[-0.18, -0.18, 0.13],
                    textColorRGB=[0.0, 1.0, 1.0],
                    textSize=1.3,
                    replaceItemUniqueId=status_text_id,
                    physicsClientId=client
                )
                continue
                
            height_changed = False
            if ord('z') in keys and keys[ord('z')] & 3:
                gait.body_z -= 0.0004
                height_changed = True
            if ord('x') in keys and keys[ord('x')] & 3:
                gait.body_z += 0.0004
                height_changed = True
                
            if height_changed:
                gait.body_z = np.clip(gait.body_z, -0.075, -0.015)
                status_text_id = p.addUserDebugText(
                    text=f"Body Height: {-gait.body_z * 100:.1f} cm",
                    textPosition=[-0.18, -0.18, 0.13],
                    textColorRGB=[0.0, 1.0, 1.0],
                    textSize=1.3,
                    replaceItemUniqueId=status_text_id,
                    physicsClientId=client
                )
                
            dx, dy = 0.0, 0.0
            yaw_rate = 0.0
            
            if ord('w') in keys and keys[ord('w')] & 3:
                dy = -1.0
            elif p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & 3:
                dy = -1.0
                
            if ord('s') in keys and keys[ord('s')] & 3:
                dy = 1.0
            elif p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & 3:
                dy = 1.0
                
            if ord('a') in keys and keys[ord('a')] & 3:
                dx = 1.0
            elif p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & 3:
                dx = 1.0
                
            if ord('d') in keys and keys[ord('d')] & 3:
                dx = -1.0
            elif p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & 3:
                dx = -1.0
                
            if ord('q') in keys and keys[ord('q')] & 3:
                yaw_rate = 1.5
            elif ord('e') in keys and keys[ord('e')] & 3:
                yaw_rate = -1.5
                
            joint_targets = gait.get_joint_targets(sim_time, direction=(dx, dy), yaw_rate=yaw_rate)
            
            for j_name, target_angle in joint_targets.items():
                if j_name in joint_map:
                    p.setJointMotorControl2(
                        bodyUniqueId=robot_id,
                        jointIndex=joint_map[j_name],
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=target_angle,
                        force=12.0,
                        maxVelocity=5.0,
                        physicsClientId=client
                    )
            
            p.stepSimulation(physicsClientId=client)
            sim_time += time_step
            
            # Third-person top-down follow camera
            pos, orn = p.getBasePositionAndOrientation(robot_id, physicsClientId=client)
            euler = p.getEulerFromQuaternion(orn)
            yaw_robot = euler[2]
            cam_yaw = -90.0
            
            p.resetDebugVisualizerCamera(
                cameraDistance=cam_distance,
                cameraYaw=cam_yaw,
                cameraPitch= -10,
                cameraTargetPosition=[pos[0], pos[1], pos[2] + 0.02],
                physicsClientId=client
            )
            
            time.sleep(time_step)

    except KeyboardInterrupt:
        pass
    finally:
        if p.isConnected(physicsClientId=client):
            p.disconnect(physicsClientId=client)
            print("[INFO] PyBullet disconnected.")

if __name__ == "__main__":
    main()
