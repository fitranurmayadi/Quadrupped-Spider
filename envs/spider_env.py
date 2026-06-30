import os
import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import pybullet as p
import pybullet_data


class SpiderEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(self, render_mode=None, max_steps=1000):
        super(SpiderEnv, self).__init__()

        self.render_mode = render_mode
        self.max_steps = max_steps
        self.step_counter = 0

        # Define 12 revolute joint indices
        self.revolute_joint_indices = []
        self.foot_indices = []

        # Physics client setup
        if self.render_mode == "human":
            self.client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        else:
            self.client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)
        p.setTimeStep(1.0 / 50.0, physicsClientId=self.client)

        # Action Space: 12 normalized joint position targets (-1.0 to 1.0)
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(12,), dtype=np.float32
        )

        # Observation Space: 37 values
        # 3 (Euler orientation) + 3 (Angular Vel) + 3 (Linear Vel) + 12 (Joint Pos) + 12 (Joint Vel) + 4 (Foot Contacts)
        self.observation_space = spaces.Box(
            low=-50.0, high=50.0, shape=(37,), dtype=np.float32
        )

        self.plane_id = None
        self.robot_id = None
        self._load_environment()

    def _load_environment(self):
        p.resetSimulation(physicsClientId=self.client)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)
        
        self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client)
        
        urdf_path = os.path.join(os.path.dirname(__file__), "..", "assets", "spider.urdf")
        urdf_path = os.path.abspath(urdf_path)
        
        self.robot_id = p.loadURDF(
            urdf_path,
            basePosition=[0, 0, 0.21],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
            flags=p.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=self.client
        )

        self.revolute_joint_indices = []
        self.foot_indices = []

        num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.client)
        for i in range(num_joints):
            info = p.getJointInfo(self.robot_id, i, physicsClientId=self.client)
            joint_type = info[2]
            joint_name = info[1].decode('utf-8')
            
            if joint_type == p.JOINT_REVOLUTE:
                self.revolute_joint_indices.append(i)
            if 'foot' in joint_name:
                self.foot_indices.append(i)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_counter = 0

        # Reset base position and orientation
        p.resetBasePositionAndOrientation(
            self.robot_id,
            posObj=[0, 0, 0.21],
            ornObj=p.getQuaternionFromEuler([0, 0, 0]),
            physicsClientId=self.client
        )
        p.resetBaseVelocity(
            self.robot_id,
            linearVelocity=[0, 0, 0],
            angularVelocity=[0, 0, 0],
            physicsClientId=self.client
        )

        # Reset joints to default splayed standing angles with slight initial random noise
        standing_pose = {
            'fr_coxa_joint': 0.5, 'fr_femur_joint': 1.00, 'fr_tibia_joint': 0.50,
            'fl_coxa_joint': -0.5, 'fl_femur_joint': 1.00, 'fl_tibia_joint': 0.50,
            'rr_coxa_joint': -0.5, 'rr_femur_joint': 1.00, 'rr_tibia_joint': 0.50,
            'rl_coxa_joint': 0.5, 'rl_femur_joint': 1.00, 'rl_tibia_joint': 0.50,
        }

        for joint_idx in self.revolute_joint_indices:
            joint_name = p.getJointInfo(self.robot_id, joint_idx, physicsClientId=self.client)[1].decode('utf-8')
            default_angle = standing_pose.get(joint_name, 0.0)
            init_angle = default_angle + self.np_random.uniform(-0.05, 0.05)
            p.resetJointState(self.robot_id, joint_idx, targetValue=init_angle, targetVelocity=0, physicsClientId=self.client)

        obs = self._get_observation()
        info = {}
        return obs, info

    def step(self, action):
        self.step_counter += 1

        # Scale actions to joint limits
        action = np.clip(action, -1.0, 1.0)
        
        for i, joint_idx in enumerate(self.revolute_joint_indices):
            joint_info = p.getJointInfo(self.robot_id, joint_idx, physicsClientId=self.client)
            lower_limit, upper_limit = joint_info[8], joint_info[9]
            target_angle = lower_limit + (action[i] + 1.0) * 0.5 * (upper_limit - lower_limit)
            
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_angle,
                force=2.5,
                maxVelocity=5.0,
                physicsClientId=self.client
            )

        p.stepSimulation(physicsClientId=self.client)
        if self.render_mode == "human":
            time.sleep(1.0 / 50.0)

        obs = self._get_observation()
        
        # Base state metrics
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id, physicsClientId=self.client)
        base_vel, base_ang_vel = p.getBaseVelocity(self.robot_id, physicsClientId=self.client)
        euler_orn = p.getEulerFromQuaternion(base_orn)

        # Calculate reward (forward is -Y direction, so forward velocity is -base_vel[1])
        forward_reward = -base_vel[1] * 10.0
        survival_reward = 0.5
        torque_penalty = -0.001 * np.sum(np.square(action))
        orientation_penalty = -1.0 * (abs(euler_orn[0]) + abs(euler_orn[1]))
        
        reward = forward_reward + survival_reward + torque_penalty + orientation_penalty

        # Termination conditions
        terminated = False
        if base_pos[2] < 0.05 or abs(euler_orn[0]) > 0.8 or abs(euler_orn[1]) > 0.8:
            terminated = True
            reward -= 10.0

        truncated = self.step_counter >= self.max_steps

        info = {
            "forward_velocity": -base_vel[1],
            "base_height": base_pos[2]
        }

        return obs, reward, terminated, truncated, info

    def _get_observation(self):
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id, physicsClientId=self.client)
        base_vel, base_ang_vel = p.getBaseVelocity(self.robot_id, physicsClientId=self.client)
        euler_orn = p.getEulerFromQuaternion(base_orn)

        joint_positions = []
        joint_velocities = []
        for joint_idx in self.revolute_joint_indices:
            state = p.getJointState(self.robot_id, joint_idx, physicsClientId=self.client)
            joint_positions.append(state[0])
            joint_velocities.append(state[1])

        foot_contacts = []
        for foot_idx in self.foot_indices:
            contact_points = p.getContactPoints(self.robot_id, self.plane_id, linkIndexA=foot_idx, physicsClientId=self.client)
            foot_contacts.append(1.0 if len(contact_points) > 0 else 0.0)

        obs = np.concatenate([
            euler_orn,
            base_ang_vel,
            base_vel,
            joint_positions,
            joint_velocities,
            foot_contacts
        ]).astype(np.float32)

        return obs

    def render(self):
        if self.render_mode == "rgb_array":
            base_pos, _ = p.getBasePositionAndOrientation(self.robot_id, physicsClientId=self.client)
            view_matrix = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=base_pos,
                distance=0.6,
                yaw=45,
                pitch=-30,
                roll=0,
                upAxisIndex=2,
                physicsClientId=self.client
            )
            proj_matrix = p.computeProjectionMatrixFOV(
                fov=60, aspect=1.0, nearVal=0.1, farVal=100.0, physicsClientId=self.client
            )
            (_, _, px, _, _) = p.getCameraImage(
                width=320, height=320, viewMatrix=view_matrix, projectionMatrix=proj_matrix, physicsClientId=self.client
            )
            return px

    def close(self):
        if p.isConnected(physicsClientId=self.client):
            p.disconnect(physicsClientId=self.client)
