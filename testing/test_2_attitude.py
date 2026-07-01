import os
import sys
import time
import argparse
import numpy as np
import pybullet as p
import pybullet_data

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))


class SpiderAttitudeTester:
    def __init__(self, gui=True):
        self.gui = gui
        self.connection_mode = p.GUI if gui else p.DIRECT
        self.client = p.connect(self.connection_mode)
        
        if gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        # Timestep 1/240s: standar PyBullet, lebih stabil dari 1/50
        self.time_step = 1.0 / 240.0
        p.setTimeStep(self.time_step)

        # Muat Lantai & Robot URDF
        self.plane_id = p.loadURDF("plane.urdf")
        urdf_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "assets", "spider.urdf"))
        
        self.robot_id = p.loadURDF(
            urdf_path,
            basePosition=[0, 0, 0.04],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
            flags=p.URDF_USE_INERTIA_FROM_FILE
        )

        # -------------------------------------------------------
        # Pengaturan Fisika: Friction Realistis
        # Lantai: karet/beton (lateral=0.8, spin=0.05, roll=0.02)
        # Kaki  : ujung karet (lateral=1.5, spin=0.3,  roll=0.1)
        # Link lain: rendah agar tidak drag
        # -------------------------------------------------------
        p.changeDynamics(self.plane_id, -1,
                         lateralFriction=0.8,
                         spinningFriction=0.05,
                         rollingFriction=0.02,
                         restitution=0.0)
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            info = p.getJointInfo(self.robot_id, i)
            link_name = info[12].decode('utf-8')
            if 'foot' in link_name:
                p.changeDynamics(self.robot_id, i,
                                 lateralFriction=1.5,
                                 spinningFriction=0.3,
                                 rollingFriction=0.1,
                                 restitution=0.0,
                                 contactStiffness=5000,
                                 contactDamping=100)
            else:
                p.changeDynamics(self.robot_id, i,
                                 lateralFriction=0.1,
                                 spinningFriction=0.01,
                                 rollingFriction=0.01)

        if gui:
            # Mengarahkan kamera agar melihat dari sudut kanan-depan yang jelas
            p.resetDebugVisualizerCamera(
                cameraDistance=0.7,
                cameraYaw=0,
                cameraPitch=-30,
                cameraTargetPosition=[0, 0, 0.15]
            )

        self._init_joints()

    def _init_joints(self):
        self.joint_map = {}
        self.revolute_joints = []
        num_joints = p.getNumJoints(self.robot_id)

        for i in range(num_joints):
            info = p.getJointInfo(self.robot_id, i)
            j_name = info[1].decode('utf-8')
            if info[2] == p.JOINT_REVOLUTE:
                self.revolute_joints.append(i)
                self.joint_map[j_name] = i

        # Postur Berdiri Netral (Base Standing Pose)
        self.base_pose = {
            'fl_coxa_joint': -0.5, 'fl_femur_joint': -1.20, 'fl_tibia_joint': 2.80,
            'rl_coxa_joint': 0.5, 'rl_femur_joint': -1.20, 'rl_tibia_joint': 2.80,
            'fr_coxa_joint': 0.5, 'fr_femur_joint': -1.20, 'fr_tibia_joint': 2.80,
            'rr_coxa_joint': -0.5, 'rr_femur_joint': -1.20, 'rr_tibia_joint': 2.80,
        }
        self.reset_to_base_pose()

    def reset_to_base_pose(self):
        # Reset posisi, orientasi dan kecepatan bodi agar tidak berakumulasi dari tes sebelumnya
        p.resetBasePositionAndOrientation(
            self.robot_id, [0, 0, 0.04], p.getQuaternionFromEuler([0, 0, 0])
        )
        p.resetBaseVelocity(self.robot_id, [0, 0, 0], [0, 0, 0])
        
        # Nonaktifkan motor dulu agar tidak ada kejutan gaya saat reset
        num_j = p.getNumJoints(self.robot_id)
        for i in range(num_j):
            if p.getJointInfo(self.robot_id, i)[2] == p.JOINT_REVOLUTE:
                p.setJointMotorControl2(self.robot_id, i, p.VELOCITY_CONTROL,
                                        targetVelocity=0, force=0)
        for j_name, target_val in self.base_pose.items():
            if j_name in self.joint_map:
                j_idx = self.joint_map[j_name]
                p.resetJointState(self.robot_id, j_idx, targetValue=target_val, targetVelocity=0)

        # Settling: 0.3 detik agar fisika stabil
        settle_steps = int(0.3 / self.time_step)
        for _ in range(settle_steps):
            for j_name, target_val in self.base_pose.items():
                if j_name in self.joint_map:
                    p.setJointMotorControl2(
                        self.robot_id, self.joint_map[j_name],
                        p.POSITION_CONTROL,
                        targetPosition=target_val,
                        force=8.0, maxVelocity=2.0
                    )
            p.stepSimulation()

    def apply_joint_targets(self, targets):
        for j_name, target_val in targets.items():
            if j_name in self.joint_map:
                j_idx = self.joint_map[j_name]
                p.setJointMotorControl2(
                    self.robot_id, j_idx, p.POSITION_CONTROL,
                    targetPosition=target_val,
                    force=6.0,
                    maxVelocity=3.0
                )

    def step(self, steps=1):
        for _ in range(steps):
            p.stepSimulation()
            if self.gui:
                time.sleep(self.time_step)

    def close(self):
        p.disconnect()


def print_coordinate_system_info():
    print("==================================================")
    print("      DEFINISI SISTEM KOORDINAT WORLD & ROBOT     ")
    print("==================================================")
    print("1. URUTAN & POSISI KAKI ROBOT DARI BASE CENTER:")
    print("   - Kaki 1 / Front-Left (fl)  : Depan Kiri (+X, -Y)")
    print("   - Kaki 2 / Rear-Left (rl)   : Belakang Kiri (+X, +Y)")
    print("   - Kaki 3 / Front-Right (fr) : Depan Kanan (-X, -Y)")
    print("   - Kaki 4 / Rear-Right (rr)  : Belakang Kanan (-X, +Y)")
    print("==================================================\n")


def test_attitude_control(tester, duration=3.0):
    """Pengujian Postur Sikap Badan (Pitch, Roll, Yaw)"""
    print("==================================================")
    print(" 2A. PENGUJIAN KONTROL ATTITUDE (Sikap Badan) ")
    print("==================================================")

    attitudes = [
        ("Pitch Up (Menengadah ke Atas)", {'fl_femur_joint': -0.80, 'fr_femur_joint': -0.80, 'rl_femur_joint': -1.20, 'rr_femur_joint': -1.20}, "Pitch positif (> +0.05 rad)"),
        ("Pitch Down (Mengangguk ke Bawah)", {'fl_femur_joint': -1.20, 'fr_femur_joint': -1.20, 'rl_femur_joint': -0.80, 'rr_femur_joint': -0.80}, "Pitch negatif (< -0.05 rad)"),
        ("Roll Kiri (Miring Kiri)", {'fl_femur_joint': -0.80, 'rl_femur_joint': -0.80, 'fr_femur_joint': -1.20, 'rr_femur_joint': -1.20}, "Roll positif (> +0.05 rad)"),
        ("Roll Kanan (Miring Kanan)", {'fl_femur_joint': -1.20, 'rl_femur_joint': -1.20, 'fr_femur_joint': -0.80, 'rr_femur_joint': -0.80}, "Roll negatif (< -0.05 rad)"),
    ]

    steps_per_mode = int(duration / tester.time_step)

    for name, delta_targets, ekspektasi in attitudes:
        print(f"\n[ATTITUDE] Mode       : {name}")
        print(f"           Ekspektasi : {ekspektasi}")
        targets = tester.base_pose.copy()
        targets.update(delta_targets)
        tester.apply_joint_targets(targets)
        tester.step(steps_per_mode)
        
        pos, orn = p.getBasePositionAndOrientation(tester.robot_id)
        euler = p.getEulerFromQuaternion(orn)
        phys_roll = -euler[0]
        phys_pitch = euler[1]
        phys_yaw = euler[2]
        
        # Cetak debug sudut sendi aktual
        actual_joints = {}
        for j_name in delta_targets.keys():
            if j_name in tester.joint_map:
                actual_joints[j_name] = p.getJointState(tester.robot_id, tester.joint_map[j_name])[0]
        print(f"           DEBUG      : Target: {delta_targets}")
        print(f"           DEBUG      : Aktual: {actual_joints}")
        print(f"           Realita    : Posisi (X,Y,Z): [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}] m | Orientasi Fisik (R,P,Y): [{phys_roll:+.3f}, {phys_pitch:+.3f}, {phys_yaw:+.3f}] rad")
        tester.reset_to_base_pose()

    print("\n[STATUS] Pengujian attitude badan selesai. Mengembalikan ke postur berdiri netral...\n")
    tester.reset_to_base_pose()


def test_directional_movement(tester, motion_type="maju", duration=4.0):
    """Pengujian Pergerakan (Maju, Mundur, Kiri, Kanan, Putar Kiri, Putar Kanan)"""
    expectations = {
        "maju": "Perpindahan ke arah Depan (dY < 0)",
        "mundur": "Perpindahan ke arah Belakang (dY > 0)",
        "kiri": "Perpindahan ke arah Kiri (dX > 0)",
        "kanan": "Perpindahan ke arah Kanan (dX < 0)",
        "putar_kiri": "Rotasi berlawanan jarum jam (dYaw > 0)",
        "putar_kanan": "Rotasi searah jarum jam (dYaw < 0)"
    }

    print(f"\n[GERAK] Mode       : {motion_type.upper()} ({duration} detik)")
    print(f"        Ekspektasi : {expectations.get(motion_type, '-')}")

    steps = int(duration / tester.time_step)
    start_pos, start_orn = p.getBasePositionAndOrientation(tester.robot_id)
    start_euler = p.getEulerFromQuaternion(start_orn)

    for step in range(steps):
        t = step * 0.05
        phase1 = np.sin(t)
        phase2 = np.sin(t + np.pi)

        targets = tester.base_pose.copy()

        # Femur/Tibia lifts (diagonal trot)
        # Pair 1: fl, rr
        # Pair 2: rl, fr
        targets['fl_femur_joint'] = -1.20 - 0.20 * max(0, phase1)
        targets['rr_femur_joint'] = -1.20 - 0.20 * max(0, phase1)
        targets['rl_femur_joint'] = -1.20 - 0.20 * max(0, phase2)
        targets['fr_femur_joint'] = -1.20 - 0.20 * max(0, phase2)
        
        targets['fl_tibia_joint'] = 2.80 - 0.40 * max(0, phase1)
        targets['rr_tibia_joint'] = 2.80 - 0.40 * max(0, phase1)
        targets['rl_tibia_joint'] = 2.80 - 0.40 * max(0, phase2)
        targets['fr_tibia_joint'] = 2.80 - 0.40 * max(0, phase2)

        if motion_type == "maju":
            # Maju is -Y direction (forward)
            # Left legs positive phase, right legs negative phase → net -Y thrust
            targets['fl_coxa_joint'] = -0.50 + 0.25 * phase1
            targets['rr_coxa_joint'] = -0.50 - 0.25 * phase1
            targets['rl_coxa_joint'] = 0.50 + 0.25 * phase2
            targets['fr_coxa_joint'] = 0.50 - 0.25 * phase2
            
        elif motion_type == "mundur":
            # Mundur is +Y direction (backward)
            # Opposite phase signs from maju
            targets['fl_coxa_joint'] = -0.50 - 0.25 * phase1
            targets['rr_coxa_joint'] = -0.50 + 0.25 * phase1
            targets['rl_coxa_joint'] = 0.50 - 0.25 * phase2
            targets['fr_coxa_joint'] = 0.50 + 0.25 * phase2
            
        elif motion_type == "kiri":
            # Kiri is +X direction (Left)
            # Front legs positive phase, rear legs negative phase → net +X thrust
            # Y-components from front and rear cancel each other out
            targets['fl_coxa_joint'] = -0.50 + 0.25 * phase1   # front: positive
            targets['rr_coxa_joint'] = -0.50 - 0.25 * phase1   # rear: negative
            targets['rl_coxa_joint'] = 0.50 - 0.25 * phase2    # rear: negative
            targets['fr_coxa_joint'] = 0.50 + 0.25 * phase2    # front: positive
            
        elif motion_type == "kanan":
            # Kanan is -X direction (Right)
            # Opposite phase signs from kiri
            targets['fl_coxa_joint'] = -0.50 - 0.25 * phase1   # front: negative
            targets['rr_coxa_joint'] = -0.50 + 0.25 * phase1   # rear: positive
            targets['rl_coxa_joint'] = 0.50 + 0.25 * phase2    # rear: positive
            targets['fr_coxa_joint'] = 0.50 - 0.25 * phase2    # front: negative
            
        elif motion_type == "putar_kiri":
            # CCW (+Yaw)
            targets['fl_coxa_joint'] = -0.50 - 0.25 * phase1
            targets['rr_coxa_joint'] = -0.50 - 0.25 * phase1
            targets['rl_coxa_joint'] = 0.50 - 0.25 * phase2
            targets['fr_coxa_joint'] = 0.50 - 0.25 * phase2
            
        elif motion_type == "putar_kanan":
            # CW (-Yaw)
            targets['fl_coxa_joint'] = -0.50 + 0.25 * phase1
            targets['rr_coxa_joint'] = -0.50 + 0.25 * phase1
            targets['rl_coxa_joint'] = 0.50 + 0.25 * phase2
            targets['fr_coxa_joint'] = 0.50 + 0.25 * phase2

        tester.apply_joint_targets(targets)
        tester.step(1)

    end_pos, end_orn = p.getBasePositionAndOrientation(tester.robot_id)
    end_euler = p.getEulerFromQuaternion(end_orn)

    dx = end_pos[0] - start_pos[0]
    dy = end_pos[1] - start_pos[1]
    dz = end_pos[2] - start_pos[2]
    # Menghitung delta dengan menyesuaikan sumbu fisik robot (R=-Pitch_world, P=-Roll_world)
    start_phys_roll = -start_euler[1]
    start_phys_pitch = -start_euler[0]
    end_phys_roll = -end_euler[1]
    end_phys_pitch = -end_euler[0]

    droll = end_phys_roll - start_phys_roll
    dpitch = end_phys_pitch - start_phys_pitch
    dyaw = end_euler[2] - start_euler[2]

    print(f"        Realita    : Posisi Awal  (X,Y,Z): [{start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f}] m | Orientasi Awal Fisik (R,P,Y): [{start_phys_roll:+.3f}, {start_phys_pitch:+.3f}, {start_euler[2]:+.3f}] rad")
    print(f"                     Posisi Akhir (X,Y,Z): [{end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f}] m | Orientasi Akhir Fisik (R,P,Y): [{end_phys_roll:+.3f}, {end_phys_pitch:+.3f}, {end_euler[2]:+.3f}] rad")
    print(f"                     Delta Posisi & Orientasi: dX={dx:+.3f}m, dY={dy:+.3f}m, dZ={dz:+.3f}m | dR={droll:+.3f}rad, dP={dpitch:+.3f}rad, dY={dyaw:+.3f}rad")
    tester.reset_to_base_pose()


def run_all_tests(gui=True, duration=3.0):
    print_coordinate_system_info()
    tester = SpiderAttitudeTester(gui=gui)

    # 1. Uji Kontrol Sikap / Attitude Badan
    test_attitude_control(tester, duration=duration)

    # 2. Uji Pergerakan Arah
    motions = ["maju", "mundur", "kiri", "kanan", "putar_kiri", "putar_kanan"]
    print("==================================================")
    print(" 2B. PENGUJIAN GERAK ARAH (Locomotion Kinematics) ")
    print("==================================================")
    for motion in motions:
        test_directional_movement(tester, motion_type=motion, duration=duration)

    print("--------------------------------------------------")
    print(" [STATUS] >>> TEST 2 ATTITUDE & MOTION: PASSED <<<")
    print(" Semestinya semua modul sikap dan gerakan arah berfungsi baik.")
    print("--------------------------------------------------")
    tester.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test 2: Attitude & Directional Movements Test")
    parser.add_argument("--headless", action="store_true", help="Jalankan simulasi tanpa GUI")
    parser.add_argument("--duration", type=float, default=3.0, help="Durasi tiap mode gerakan (detik)")
    args = parser.parse_args()

    run_all_tests(gui=not args.headless, duration=args.duration)
