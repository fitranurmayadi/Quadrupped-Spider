import os
import time
import argparse
import numpy as np
import gymnasium as gym
from gymnasium.envs.registration import register
from stable_baselines3 import PPO
from envs.spider_env import SpiderEnv

# Register custom environment
try:
    register(
        id="SpiderEnv-v0",
        entry_point="envs.spider_env:SpiderEnv",
        max_episode_steps=1000,
    )
except gym.error.Error:
    pass


def heuristic_controller(step_idx):
    """Sinusoidal tripod gait controller for fallback testing."""
    t = step_idx * 0.15
    # 4 legs: FR (0-2), FL (3-5), RR (6-8), RL (9-11)
    # Joint order for each leg: Coxa, Femur, Tibia
    action = np.zeros(12, dtype=np.float32)
    
    phase1 = np.sin(t)
    phase2 = np.sin(t + np.pi)
    
    # Coxa actions (swing forward during swing, backward during stance)
    # fr_coxa (0): default 0.50, +swing forward, -swing backward
    # fl_coxa (3): default -0.50, -swing forward, +swing backward
    # rr_coxa (6): default -0.50, +swing forward, -swing backward
    # rl_coxa (9): default 0.50, -swing forward, +swing backward
    action[0] = 0.50 + 0.35 * phase2
    action[3] = -0.50 - 0.35 * phase1
    action[6] = -0.50 + 0.35 * phase1
    action[9] = 0.50 - 0.35 * phase2
    
    # Femur actions (lift during swing)
    # Default is 1.00, decreases to ~0.65 to lift
    action[1] = 1.00 - 0.35 * max(0, phase2)
    action[4] = 1.00 - 0.35 * max(0, phase1)
    action[7] = 1.00 - 0.35 * max(0, phase1)
    action[10] = 1.00 - 0.35 * max(0, phase2)
    
    # Tibia actions (constant flexed stand angle)
    action[2] = 0.50
    action[5] = 0.50
    action[8] = 0.50
    action[11] = 0.50
    
    return action


def test(model_path="./models/spider_ppo_model.zip", episodes=5, render_mode="human"):
    env = SpiderEnv(render_mode=render_mode)
    
    model = None
    if os.path.exists(model_path):
        print(f"Loading trained PPO model from {model_path}...")
        model = PPO.load(model_path.replace(".zip", ""))
    else:
        print(f"No trained model found at {model_path}. Running heuristic sinusoidal gait demo...")

    for ep in range(episodes):
        obs, info = env.reset()
        done = False
        total_reward = 0
        step_count = 0

        print(f"\n--- Starting Episode {ep + 1}/{episodes} ---")

        while not done:
            if model is not None:
                action, _ = model.predict(obs, deterministic=True)
            else:
                action = heuristic_controller(step_count)

            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            step_count += 1
            done = terminated or truncated

            if step_count % 100 == 0:
                print(f"Step {step_count} | Forward Vel: {info['forward_velocity']:.3f} m/s | Height: {info['base_height']:.3f} m")

        print(f"Episode {ep + 1} finished in {step_count} steps. Total Reward: {total_reward:.2f}")

    env.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test trained model or run heuristic gait demo.")
    parser.add_argument("--model", type=str, default="./models/spider_ppo_model.zip", help="Path to trained PPO model zip.")
    parser.add_argument("--episodes", type=int, default=5, help="Number of test episodes.")
    parser.add_argument("--headless", action="store_true", help="Run in headless direct mode.")
    args = parser.parse_args()
    
    render_mode = None if args.headless else "human"
    test(model_path=args.model, episodes=args.episodes, render_mode=render_mode)
