import os
import json
import time
import numpy as np
import pybullet as p
import pybullet_data

def main():
    print("==================================================")
    # Welcoming the user to the interactive calibration tool
    print("   INTERACTIVE CALIBRATION & KINEMATICS TOOL      ")
    print("==================================================")
    print("Petunjuk Penggunaan:")
    print("1. Gunakan Debug Sliders di panel kanan PyBullet untuk mengontrol:")
    # Instructions for sliders
    print("   - Posisi & Orientasi Body (Z, Roll, Pitch, Yaw)")
    print("   - Sudut 12 sendi (Coxa, Femur, Tibia) masing-masing kaki")
    print("   - Mode Fisika/Gravitasi (0 = OFF/Base Melayang, 1 = ON/Fisika Aktif)")
    print("2. Tekan tombol keyboard 'S' untuk MENYIMPAN sudut joint saat ini.")
    print("   Sudut yang disimpan akan dicetak ke terminal ini dan disimpan ke")
    print("   file 'calibration_results.json'.")
    print("==================================================\n")

    # 1. Hubungkan ke PyBullet GUI
    client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    # Sembunyikan panel eksplorasi kamera di sebelah kiri
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 50.0)

    # 2. Muat Lantai / Plane
    plane_id = p.loadURDF("plane.urdf")
    p.changeDynamics(plane_id, -1, lateralFriction=1.5)

    # 3. Muat Robot Spider
    urdf_path = os.path.join(os.path.dirname(__file__), "..", "assets", "spider.urdf")
    urdf_path = os.path.abspath(urdf_path)
    
    start_pos = [0, 0, 0.21]
    start_orn = p.getQuaternionFromEuler([0, 0, 0])
    
    robot_id = p.loadURDF(
        urdf_path,
        basePosition=start_pos,
        baseOrientation=start_orn,
        flags=p.URDF_USE_INERTIA_FROM_FILE
    )

    # Set high friction for feet
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        link_name = info[12].decode('utf-8')
        if 'foot' in link_name or 'tibia' in link_name:
            p.changeDynamics(robot_id, i, lateralFriction=2.0)

    # Adjust camera
    p.resetDebugVisualizerCamera(
        cameraDistance=0.6,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.15]
    )

    # 4. Buat Debug Sliders untuk Body
    print("[INFO] Membuat debug sliders untuk kontrol bodi...")
    slider_body_z = p.addUserDebugParameter("Body Z (Height)", 0.05, 0.30, 0.21)
    slider_body_roll = p.addUserDebugParameter("Body Roll (Phys)", -0.5, 0.5, 0.0)
    slider_body_pitch = p.addUserDebugParameter("Body Pitch (Phys)", -0.5, 0.5, 0.0)
    slider_body_yaw = p.addUserDebugParameter("Body Yaw", -3.14, 3.14, 0.0)
    slider_physics = p.addUserDebugParameter("Physics Mode (0=OFF, 1=ON)", 0, 1, 0)

    # Define joint sliders mapping and default values
    joint_map = {}
    sliders_joints = {}
    
    # Standing splayed stance defaults
    defaults = {
        'fr_coxa_joint': 0.0, 'fr_femur_joint': 0.0, 'fr_tibia_joint': 0.0,
        'fl_coxa_joint': 0.0, 'fl_femur_joint': 0.0, 'fl_tibia_joint': 0.0,
        'rr_coxa_joint': 0.0, 'rr_femur_joint': 0.0, 'rr_tibia_joint': 0.0,
        'rl_coxa_joint': 0.0, 'rl_femur_joint': 0.0, 'rl_tibia_joint': 0.0,
    }

    print("[INFO] Membuat debug sliders untuk 12 sendi...")
    # Group by leg to display sliders in logical order
    legs = ['fr', 'fl', 'rr', 'rl']
    joint_types = ['coxa', 'femur', 'tibia']
    
    for leg in legs:
        for j_type in joint_types:
            j_name = f"{leg}_{j_type}_joint"
            # Find joint index in pybullet
            for i in range(num_joints):
                info = p.getJointInfo(robot_id, i)
                name = info[1].decode('utf-8')
                if name == j_name:
                    joint_map[j_name] = i
                    lower_limit, upper_limit = info[8], info[9]
                    default_val = defaults.get(j_name, 0.0)
                    slider_id = p.addUserDebugParameter(f"  {j_name}", lower_limit, upper_limit, default_val)
                    sliders_joints[j_name] = slider_id
                    break

    # 5. Simulation & Control Loop
    last_physics_val = 0
    save_msg_id = None

    try:
        while p.isConnected():
            # Baca status mode fisika
            physics_mode = int(p.readUserDebugParameter(slider_physics))
            
            # Jika baru beralih ke Physics ON
            if physics_mode == 1 and last_physics_val == 0:
                print("\n[STATUS] Physics Mode: ON (Gravitasi & kontak berjalan)")
                # Berikan gaya motor penuh untuk menopang pose saat ini
                for j_name, j_idx in joint_map.items():
                    target_pos = p.readUserDebugParameter(sliders_joints[j_name])
                    p.setJointMotorControl2(
                        bodyUniqueId=robot_id,
                        jointIndex=j_idx,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=target_pos,
                        force=4.0
                    )
            # Jika baru beralih ke Physics OFF (Base melayang)
            elif physics_mode == 0 and last_physics_val == 1:
                print("\n[STATUS] Physics Mode: OFF (Base terkunci pada posisi slider)")
                p.resetBaseVelocity(robot_id, [0, 0, 0], [0, 0, 0])
                
            last_physics_val = physics_mode

            # Ambil target dari sendi
            current_joint_targets = {}
            for j_name, j_idx in joint_map.items():
                target_pos = p.readUserDebugParameter(sliders_joints[j_name])
                current_joint_targets[j_name] = target_pos
                
                if physics_mode == 0:
                    # Posisi langsung di-reset (kinematik melayang)
                    p.resetJointState(robot_id, j_idx, targetValue=target_pos)
                else:
                    # Terapkan via motor controller
                    p.setJointMotorControl2(
                        bodyUniqueId=robot_id,
                        jointIndex=j_idx,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=target_pos,
                        force=4.0
                    )

            # Kontrol Base jika Physics OFF
            if physics_mode == 0:
                body_z = p.readUserDebugParameter(slider_body_z)
                body_roll = p.readUserDebugParameter(slider_body_roll)
                body_pitch = p.readUserDebugParameter(slider_body_pitch)
                body_yaw = p.readUserDebugParameter(slider_body_yaw)
                
                # Konversi orientasi fisik ke PyBullet World Euler
                # physical_roll = -euler[1] => euler[1] = -physical_roll
                # physical_pitch = -euler[0] => euler[0] = -physical_pitch
                world_roll = -body_pitch
                world_pitch = -body_roll
                world_yaw = body_yaw
                
                target_orn = p.getQuaternionFromEuler([world_roll, world_pitch, world_yaw])
                p.resetBasePositionAndOrientation(robot_id, [0, 0, body_z], target_orn)
                p.resetBaseVelocity(robot_id, [0, 0, 0], [0, 0, 0])

            # Jalankan simulation step
            p.stepSimulation()
            time.sleep(1.0 / 50.0)

            # Baca keyboard event
            keys = p.getKeyboardEvents()
            # Pengecekan tombol 'S' (case-insensitive)
            s_pressed = ord('s') in keys or ord('S') in keys
            s_triggered = False
            
            if s_pressed:
                # Periksa apakah itu event penekanan baru
                s_key = ord('s') if ord('s') in keys else ord('S')
                if keys[s_key] & p.KEY_WAS_TRIGGERED:
                    s_triggered = True

            if s_triggered:
                # 6. Ambil nama pose dari terminal
                print("\n[INPUT] Terdeteksi penekanan tombol 'S'.")
                pose_name = input("Masukkan nama/label pose ini (contoh: standing, pitch_up, dll): ").strip()
                if not pose_name:
                    pose_name = f"pose_{time.strftime('%H%M%S')}"

                pos, orn = p.getBasePositionAndOrientation(robot_id)
                euler = p.getEulerFromQuaternion(orn)
                phys_roll = -euler[1]
                phys_pitch = -euler[0]
                phys_yaw = euler[2]

                data_to_save = {
                    "pose_name": pose_name,
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "physics_mode": "ON" if physics_mode == 1 else "OFF",
                    "base_position": {"x": pos[0], "y": pos[1], "z": pos[2]},
                    "physical_attitude": {"roll": phys_roll, "pitch": phys_pitch, "yaw": phys_yaw},
                    "world_euler": {"roll": euler[0], "pitch": euler[1], "yaw": euler[2]},
                    "joint_angles": current_joint_targets
                }

                # Simpan ke calibration_results.json di folder tools
                calib_file = os.path.join(os.path.dirname(__file__), "calibration_results.json")
                existing_data = []
                if os.path.exists(calib_file):
                    try:
                        with open(calib_file, "r") as f:
                            existing_data = json.load(f)
                    except Exception:
                        pass
                
                existing_data.append(data_to_save)
                with open(calib_file, "w") as f:
                    json.dump(existing_data, f, indent=2)

                print("\n==================================================")
                print(f"   POSE TERSIMPAN! (POSE: {pose_name.upper()})    ")
                print("==================================================")
                print(f"Data ditambahkan ke '{calib_file}'")
                print(f"Base Z Height     : {pos[2]:.4f} m")
                print(f"Phys Euler (R,P,Y): [{phys_roll:+.4f}, {phys_pitch:+.4f}, {phys_yaw:+.4f}] rad")
                print(f"World Euler       : [{euler[0]:+.4f}, {euler[1]:+.4f}, {euler[2]:+.4f}] rad")
                print("Sudut Sendi:")
                for name, val in current_joint_targets.items():
                    print(f"  '{name}': {val:.4f},")
                print("==================================================\n")

                # Tambahkan notasi visual di PyBullet GUI
                if save_msg_id is not None:
                    p.removeUserDebugItem(save_msg_id)
                
                save_msg_id = p.addUserDebugText(
                    f"Pose '{pose_name}' Saved! Z={pos[2]:.3f} R={phys_roll:+.2f} P={phys_pitch:+.2f}",
                    [0, 0, pos[2] + 0.1],
                    textColorRGB=[0, 0.8, 0],
                    textSize=1.5
                )

    except KeyboardInterrupt:
        print("\nAlat kalibrasi dihentikan.")
    finally:
        if p.isConnected():
            p.disconnect()
            print("Koneksi PyBullet ditutup.")

if __name__ == "__main__":
    main()
