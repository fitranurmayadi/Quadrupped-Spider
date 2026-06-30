import os
import time
import numpy as np
import pybullet as p
import pybullet_data


def main():
    print("==================================================")
    print("   Tahap 1: Visualisasi Robot Spider di PyBullet   ")
    print("   Mode: GUI / Human Render                       ")
    print("==================================================")

    # 1. Hubungkan ke PyBullet GUI
    client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    
    # Set path pencarian bawaan pybullet (untuk plane.urdf)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 50.0)

    # 2. Muat Lantai / Plane
    plane_id = p.loadURDF("plane.urdf")

    # 3. Muat URDF Quadruped Spider
    urdf_path = os.path.join(os.path.dirname(__file__), "assets", "spider.urdf")
    urdf_path = os.path.abspath(urdf_path)
    
    start_pos = [0, 0, 0.35]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    
    print(f"Loading URDF from: {urdf_path}")
    robot_id = p.loadURDF(
        urdf_path,
        basePosition=start_pos,
        baseOrientation=start_orientation,
        flags=p.URDF_USE_INERTIA_FROM_FILE
    )

    # Adjust kamera PyBullet agar fokus ke robot
    p.resetDebugVisualizerCamera(
        cameraDistance=0.5,
        cameraYaw=0,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.1]
    )

    # 4. Deteksi Joint dan Buat Slider GUI untuk Kontrol Konvensional / Manual
    num_joints = p.getNumJoints(robot_id)
    revolute_joints = []
    sliders = {}

    print(f"\nTotal Joints terdeteksi: {num_joints}")
    print("Membuat Debug Sliders untuk kontrol 12 sendi (Coxa, Femur, Tibia)...")

    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_info[2]
        lower_limit = joint_info[8]
        upper_limit = joint_info[9]

        if joint_type == p.JOINT_REVOLUTE:
            revolute_joints.append(i)
            # Atur posisi awal slider ke 0.0 jika di dalam limit
            default_val = 0.0 if lower_limit <= 0.0 <= upper_limit else (lower_limit + upper_limit) / 2.0
            slider_id = p.addUserDebugParameter(joint_name, lower_limit, upper_limit, default_val)
            sliders[i] = slider_id
            print(f" - Joint [{i}]: {joint_name} (Limits: {lower_limit:.2f} rad s/d {upper_limit:.2f} rad)")

    print("\n[INFO] Simulasi berjalan! Geser slider pada panel kanan PyBullet untuk menggerakkan sendi robot.")
    print("[INFO] Tekan Ctrl+C di terminal ini jika ingin menghentikan simulasi.")

    # 5. Simulation Loop
    try:
        while p.isConnected():
            # Baca nilai slider dan terapkan ke joint motor
            for joint_idx, slider_id in sliders.items():
                target_pos = p.readUserDebugParameter(slider_id)
                p.setJointMotorControl2(
                    bodyUniqueId=robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target_pos,
                    force=3.0,
                    maxVelocity=5.0
                )

            # Step fisika simulasi
            p.stepSimulation()
            time.sleep(1.0 / 50.0)

    except KeyboardInterrupt:
        print("\nSimulasi dihentikan oleh pengguna.")
    finally:
        if p.isConnected():
            p.disconnect()
            print("Koneksi PyBullet ditutup.")

if __name__ == "__main__":
    main()
