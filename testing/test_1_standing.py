import os
import sys
import time
import argparse
import numpy as np
import pybullet as p
import pybullet_data

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))


def run_standing_test(gui=True, duration_sec=10.0):
    """
    Test 1: Menguji kemampuan robot spider untuk berdiri tegak secara stabil.
    """
    print("==================================================")
    print("   TEST 1: Pengujian Robot Berdiri Tegak (Standing Test)   ")
    print("==================================================")

    # 1. Inisialisasi PyBullet
    connection_mode = p.GUI if gui else p.DIRECT
    client = p.connect(connection_mode)
    
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    # Timestep 1/240s: standar PyBullet, jauh lebih stabil dari 1/50
    time_step = 1.0 / 240.0
    p.setTimeStep(time_step)

    # 2. Muat Lantai & Robot URDF
    plane_id = p.loadURDF("plane.urdf")
    
    urdf_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "assets", "spider.urdf"))
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file tidak ditemukan di: {urdf_path}")

    start_pos = [0, 0, 0.08]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    
    robot_id = p.loadURDF(
        urdf_path,
        basePosition=start_pos,
        baseOrientation=start_orientation,
        flags=p.URDF_USE_INERTIA_FROM_FILE
    )

    # -------------------------------------------------------
    # Pengaturan Fisika: Friction Realistis
    # Lantai: karet/beton (lateral=0.8, spin=0.05, roll=0.02)
    # Kaki  : ujung karet (lateral=1.5, spin=0.3,  roll=0.1)
    # Semua link lain: default rendah agar tidak drag berlebih
    # -------------------------------------------------------
    p.changeDynamics(plane_id, -1,
                     lateralFriction=0.8,
                     spinningFriction=0.05,
                     rollingFriction=0.02,
                     restitution=0.0)
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        link_name = info[12].decode('utf-8')
        if 'foot' in link_name:
            # Ujung kaki: friction tinggi (grip karet)
            p.changeDynamics(robot_id, i,
                             lateralFriction=1.5,
                             spinningFriction=0.3,
                             rollingFriction=0.1,
                             restitution=0.0,
                             contactStiffness=5000,
                             contactDamping=100)
        else:
            # Link lain: friction rendah, tidak mengganggu kinematika
            p.changeDynamics(robot_id, i,
                             lateralFriction=0.1,
                             spinningFriction=0.01,
                             rollingFriction=0.01)

    if gui:
        p.resetDebugVisualizerCamera(
            cameraDistance=0.6,
            cameraYaw=0,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0.15]
        )

    # 3. Deteksi Joint dan Pengelompokan Kaki (4 Leg Groups: fr, fl, rr, rl)
    num_joints = p.getNumJoints(robot_id)
    revolute_joints = []
    leg_links = {'fr': [], 'fl': [], 'rr': [], 'rl': []}

    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8')
        link_name = joint_info[12].decode('utf-8')
        joint_type = joint_info[2]
        
        if joint_type == p.JOINT_REVOLUTE:
            revolute_joints.append(i)
        
        for leg_prefix in leg_links.keys():
            if leg_prefix in link_name or leg_prefix in joint_name:
                if 'tibia' in link_name or 'foot' in link_name:
                    leg_links[leg_prefix].append(i)

    print(f"[INFO] Terdeteksi {len(revolute_joints)} revolute joints.")

    # Target sudut berdiri tegak 
    standing_targets = {}
    for j_idx in revolute_joints:
        joint_name = p.getJointInfo(robot_id, j_idx)[1].decode('utf-8')
        if 'fr_coxa_joint' in joint_name:
            standing_targets[j_idx] = 0.5
        elif 'fl_coxa_joint' in joint_name:
            standing_targets[j_idx] = -0.5
        elif 'rr_coxa_joint' in joint_name:
            standing_targets[j_idx] = -0.5
        elif 'rl_coxa_joint' in joint_name:
            standing_targets[j_idx] = 0.5
        elif 'femur' in joint_name:
            standing_targets[j_idx] = -1.2
        elif 'tibia' in joint_name:
            standing_targets[j_idx] = 2.8
        else:
            standing_targets[j_idx] = 0.0

    # Reset posisi joint awal ke target berdiri
    # Nonaktifkan motor dulu agar tidak ada gaya saat reset
    for j_idx in revolute_joints:
        p.setJointMotorControl2(robot_id, j_idx, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
    for j_idx, target_angle in standing_targets.items():
        p.resetJointState(robot_id, j_idx, targetValue=target_angle, targetVelocity=0)

    # Settling: beri waktu 0.5 detik agar fisika stabil sebelum mulai kontrol
    settle_steps = int(0.5 / time_step)
    for _ in range(settle_steps):
        for j_idx, target_angle in standing_targets.items():
            p.setJointMotorControl2(
                bodyUniqueId=robot_id,
                jointIndex=j_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_angle,
                force=12.0,
                maxVelocity=2.0
            )
        p.stepSimulation()

    print("[INFO] Memulai simulasi berdiri tegak...")

    total_steps = int(duration_sec / time_step)
    height_records = []
    contacts_records = []
    report_interval = int(1.0 / time_step)  # laporan tiap 1 detik sim

    for step in range(total_steps):
        for j_idx, target_angle in standing_targets.items():
            p.setJointMotorControl2(
                bodyUniqueId=robot_id,
                jointIndex=j_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_angle,
                force=12.0,       # sesuai effort URDF (femur/tibia=6N, coxa=5N)
                maxVelocity=3.0  # batas kecepatan wajar servo kecil
            )

        p.stepSimulation()
        if gui:
            time.sleep(time_step)

        base_pos, base_orn = p.getBasePositionAndOrientation(robot_id)
        euler_orn = p.getEulerFromQuaternion(base_orn)
        
        # Hitung berapa banyak dari 4 kaki yang menapak lantai
        active_legs = 0
        for leg_prefix, indices in leg_links.items():
            leg_contact = False
            for idx in indices:
                pts = p.getContactPoints(robot_id, plane_id, linkIndexA=idx)
                if len(pts) > 0:
                    leg_contact = True
                    break
            if leg_contact:
                active_legs += 1

        height_records.append(base_pos[2])
        contacts_records.append(active_legs)

        if (step + 1) % report_interval == 0 or step == total_steps - 1:
            print(f"t={((step+1)*time_step):.1f}s | Posisi (X,Y,Z): [{base_pos[0]:.3f}, {base_pos[1]:.3f}, {base_pos[2]:.3f}] m | Orientasi (R,P,Y): [{euler_orn[0]:.3f}, {euler_orn[1]:.3f}, {euler_orn[2]:.3f}] rad | Kaki Menapak: {active_legs}/4")

    # Evaluasi: ambil rata-rata 2 detik terakhir
    tail = int(2.0 / time_step)
    avg_height = np.mean(height_records[-tail:]) if len(height_records) >= tail else np.mean(height_records)
    final_pos, final_orn = p.getBasePositionAndOrientation(robot_id)
    final_euler = p.getEulerFromQuaternion(final_orn)
    final_contacts = contacts_records[-1]

    print("\n--------------------------------------------------")
    print("   HASIL EVALUASI PENGUJIAN BERDIRI TEGAK   ")
    print("--------------------------------------------------")
    print(f"Posisi Akhir Badan (X, Y, Z)    : [{final_pos[0]:.4f}, {final_pos[1]:.4f}, {final_pos[2]:.4f}] m")
    print(f"Orientasi Akhir (Roll, Pitch, Yaw): [{final_euler[0]:.4f}, {final_euler[1]:.4f}, {final_euler[2]:.4f}] rad")
    print(f"Tinggi Rata-rata Akhir (2s)      : {avg_height:.4f} m")
    print(f"Jumlah Kaki Menapak             : {final_contacts} dari 4 kaki")

    is_height_ok = avg_height > 0.02      # ada clearance tubuh dari lantai
    is_upright_ok = abs(final_euler[0]) < 0.3 and abs(final_euler[1]) < 0.3  # tidak terlalu miring
    is_contacts_ok = final_contacts >= 3  # minimal 3 kaki menapak

    passed = is_height_ok and is_upright_ok and is_contacts_ok

    if passed:
        print("\n[STATUS] >>> TEST BERDIRI TEGAK: PASSED / LULUS <<<")
        print("Robot berhasil berdiri tegak dengan stabil dan menapak lantai secara seimbang.")
    else:
        print("\n[STATUS] >>> TEST BERDIRI TEGAK: FAILED / GAGAL <<<")

    p.disconnect()
    return passed


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test 1: Standing upright test for Quadruped Spider Robot")
    parser.add_argument("--headless", action="store_true", help="Jalankan simulasi tanpa GUI")
    parser.add_argument("--duration", type=float, default=10.0, help="Durasi simulasi dalam detik (default: 10.0)")
    args = parser.parse_args()

    run_standing_test(gui=not args.headless, duration_sec=args.duration)
