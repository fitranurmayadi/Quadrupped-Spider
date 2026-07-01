import os
import sys
import time
import numpy as np
import pybullet as p
import pybullet_data

# Ensure we can import spider_ik and gait_controller
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from gait_controller import GaitController

def main():
    print("==================================================")
    print("      INTERACTIVE KEYBOARD CONTROL (WASD/ARROWS)  ")
    print("==================================================")
    print("Control Instructions:")
    print("  - W / UP ARROW    : Move Forward")
    print("  - S / DOWN ARROW  : Move Backward")
    print("  - A / LEFT ARROW  : Move Left (Lateral)")
    print("  - D / RIGHT ARROW : Move Right (Lateral)")
    print("  - Q               : Turn Left")
    print("  - E               : Turn Right")
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

    # 3. Initialize Gait Controller
    gait = GaitController(step_length=0.06, step_height=0.025, frequency=2.5, body_z=-0.028)

    # Function to reset joints to splayed standing pose
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
                    angle = 2.8
                else:
                    angle = 0.0
                p.resetJointState(robot_id, j_idx, targetValue=angle, targetVelocity=0, physicsClientId=client)

    reset_pose()

    # Add instructions text overlay in PyBullet GUI
    instruction_text = (
        "Controls:\n"
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
        textColorRGB=[0.0, 1.0, 0.0],
        textSize=1.3,
        lifeTime=0,
        physicsClientId=client
    )

    # Add a visual ground grid for orientation reference
    for i in range(-5, 6):
        p.addUserDebugLine([i, -5, 0.001], [i, 5, 0.001], [0.5, 0.5, 0.5], 1, 0, physicsClientId=client)
        p.addUserDebugLine([-5, i, 0.001], [5, i, 0.001], [0.5, 0.5, 0.5], 1, 0, physicsClientId=client)

    sim_time = 0.0
    
    # Camera settings for third-person follow view
    cam_distance = 0.5
    cam_pitch = -25.0
    
    print("[INFO] Simulation active! Focus the PyBullet window to control the robot.")

    try:
        while p.isConnected(physicsClientId=client):
            # Capture keyboard events
            keys = p.getKeyboardEvents(physicsClientId=client)
            
            # Check ESC to quit
            if 27 in keys and keys[27] & 1:
                break
                
            # Check R to reset
            if ord('r') in keys and keys[ord('r')] & 1:
                reset_pose()
                gait.body_z = -0.028 # reset height
                sim_time = 0.0
                # Update status overlay
                status_text_id = p.addUserDebugText(
                    text=f"Body Height: {-gait.body_z * 100:.1f} cm",
                    textPosition=[-0.18, -0.18, 0.13],
                    textColorRGB=[0.0, 1.0, 0.0],
                    textSize=1.3,
                    replaceItemUniqueId=status_text_id,
                    physicsClientId=client
                )
                continue
                
            # Check height adjustments (Z to raise, X to lower)
            height_changed = False
            # Z key: raise body (make body_z more negative)
            if ord('z') in keys and keys[ord('z')] & 3:
                gait.body_z -= 0.0004
                height_changed = True
            # X key: lower body (make body_z less negative)
            if ord('x') in keys and keys[ord('x')] & 3:
                gait.body_z += 0.0004
                height_changed = True
                
            if height_changed:
                # Bound height to [-0.015, -0.075] (1.5 cm to 7.5 cm)
                gait.body_z = np.clip(gait.body_z, -0.075, -0.015)
                # Update status overlay
                status_text_id = p.addUserDebugText(
                    text=f"Body Height: {-gait.body_z * 100:.1f} cm",
                    textPosition=[-0.18, -0.18, 0.13],
                    textColorRGB=[0.0, 1.0, 0.0],
                    textSize=1.3,
                    replaceItemUniqueId=status_text_id,
                    physicsClientId=client
                )
                
            # Direction and rotation commands
            dx, dy = 0.0, 0.0
            yaw_rate = 0.0
            
            # Translation keys
            # W / Up Arrow
            if ord('w') in keys and keys[ord('w')] & 3:
                dy = -1.0
            elif p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & 3:
                dy = -1.0
                
            # S / Down Arrow
            if ord('s') in keys and keys[ord('s')] & 3:
                dy = 1.0
            elif p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & 3:
                dy = 1.0
                
            # A / Left Arrow
            if ord('a') in keys and keys[ord('a')] & 3:
                dx = 1.0
            elif p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & 3:
                dx = 1.0
                
            # D / Right Arrow
            if ord('d') in keys and keys[ord('d')] & 3:
                dx = -1.0
            elif p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & 3:
                dx = -1.0
                
            # Rotation keys (Q / E) - Turned rate increased to 1.5
            if ord('q') in keys and keys[ord('q')] & 3:
                yaw_rate = 1.5
            elif ord('e') in keys and keys[ord('e')] & 3:
                yaw_rate = -1.5
                
            # Get joint targets
            joint_targets = gait.get_joint_targets(sim_time, direction=(dx, dy), yaw_rate=yaw_rate)
            
            # Apply to motors
            for j_name, target_angle in joint_targets.items():
                if j_name in joint_map:
                    p.setJointMotorControl2(
                        bodyUniqueId=robot_id,
                        jointIndex=joint_map[j_name],
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=target_angle,
                        force=10.0,
                        maxVelocity=5.0,
                        physicsClientId=client
                    )
            
            p.stepSimulation(physicsClientId=client)
            sim_time += time_step
            
            # Third-person follow camera
            pos, orn = p.getBasePositionAndOrientation(robot_id, physicsClientId=client)
            euler = p.getEulerFromQuaternion(orn)
            yaw_robot = euler[2]
            
            # In PyBullet:
            # camYaw = 0 points along -Y.
            # As robot yaw_robot increases (CCW), we must subtract it from -90 degrees
            # to make the camera follow behind.
            cam_yaw = -90.0 - np.degrees(yaw_robot)
            
            # Top-down follow camera (pitch = -89.9 degrees to avoid gimbal lock)
            cam_pitch = 0
            p.resetDebugVisualizerCamera(
                cameraDistance=cam_distance,
                cameraYaw=0,
                cameraPitch=cam_pitch,
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
