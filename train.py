import os
import argparse
import gymnasium as gym
from gymnasium.envs.registration import register
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from envs.spider_env import SpiderEnv

# Register custom environment
register(
    id="SpiderEnv-v0",
    entry_point="envs.spider_env:SpiderEnv",
    max_episode_steps=1000,
)

def train(timesteps=100000, num_envs=4, save_dir="./models"):
    os.makedirs(save_dir, exist_ok=True)
    tensorboard_dir = "./tensorboard_logs"
    os.makedirs(tensorboard_dir, exist_ok=True)

    print(f"Creating {num_envs} vectorized environments...")
    env = make_vec_env("SpiderEnv-v0", n_envs=num_envs)

    print("Initializing PPO agent...")
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        tensorboard_log=tensorboard_dir
    )

    print(f"Starting training for {timesteps} timesteps...")
    model.learn(total_timesteps=timesteps)

    model_path = os.path.join(save_dir, "spider_ppo_model")
    model.save(model_path)
    print(f"Model saved successfully to {model_path}.zip")

    env.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train Quadruped Spider RL Policy")
    parser.add_argument("--timesteps", type=int, default=100000, help="Total training timesteps")
    parser.add_argument("--envs", type=int, default=4, help="Number of parallel environments")
    args = parser.parse_args()

    train(timesteps=args.timesteps, num_envs=args.envs)
