import os
import sys
import time
import numpy as np
import pybullet as p
import pybullet_data
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# Ensure we can import src.gait_controller_v2 and src.path_planning
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from src.gait_controller_v2 import GaitControllerV2
from src.path_planning import GridMap, a_star_search, dijkstra_search, pure_pursuit

# Recursive Raycast ignoring all robots
def cast_ray_recursive(client, start, end, robot_ids, plane_id, max_depth=5):
    current_start = list(start)
    current_end = list(end)
    original_start = np.array(start)
    original_end = np.array(end)
    original_length = np.linalg.norm(original_end - original_start)
    direction = (original_end - original_start) / original_length
    
    remaining_length = original_length
    
    for depth in range(max_depth):
        res = p.rayTest(current_start, current_end, physicsClientId=client)[0]
        hit_object = res[0]
        hit_fraction = res[2]
        hit_pos = res[3]
        
        if hit_object == -1:
            return -1, 1.0, current_end
            
        if hit_object not in robot_ids:
            # Hit actual obstacle! Calculate fraction relative to original start/end
            total_dist = np.linalg.norm(np.array(hit_pos) - original_start)
            fraction = total_dist / original_length
            return hit_object, min(fraction, 1.0), hit_pos
            
        # Hit one of the robots! Advance start point past the hit position
        dist_stepped = hit_fraction * remaining_length
        step_epsilon = 0.08  # step 8cm forward past the leg
        if dist_stepped + step_epsilon >= remaining_length:
            return -1, 1.0, end
            
        current_start = list(np.array(current_start) + (dist_stepped + step_epsilon) * direction)
        remaining_length = np.linalg.norm(original_end - np.array(current_start))
        
    return -1, 1.0, end

# Helper to color specific joints and sasis of the robot
def color_robot(robot_id, color, client):
    # Color base sasis (link index -1)
    p.changeVisualShape(robot_id, -1, rgbaColor=color, physicsClientId=client)
    # Color joints
    for i in range(p.getNumJoints(robot_id, physicsClientId=client)):
        info = p.getJointInfo(robot_id, i, physicsClientId=client)
        link_name = info[12].decode('utf-8')
        if "coxa" in link_name or "tibia" in link_name:
            p.changeVisualShape(robot_id, i, rgbaColor=color, physicsClientId=client)

# MPC Parameters
N = 4
dt = 0.6
v_max = 1.5
omega_max = 2.0

def mpc_cost_function(u, current_pos, current_yaw, target_pos, obstacles):
    cost = 0.0
    x, y, yaw = current_pos[0], current_pos[1], current_yaw
    
    for i in range(N):
        v = u[2*i]
        w = u[2*i+1]
        x += v * np.sin(yaw) * dt
        y += -v * np.cos(yaw) * dt
        yaw += w * dt
        
        # Distance to goal cost
        dist_to_goal = np.sqrt((target_pos[0] - x)**2 + (target_pos[1] - y)**2)
        cost += dist_to_goal * 10.0
        
        # Obstacle avoidance cost
        for obs in obstacles:
            obs_pos = obs[0]
            dist_to_obs = np.sqrt((obs_pos[0] - x)**2 + (obs_pos[1] - y)**2)
            if dist_to_obs < 0.6:
                cost += 800.0 / (dist_to_obs + 0.01)
                
    return cost

def solve_mpc(current_pos, current_yaw, target_pos, obstacles):
    u0 = np.zeros(2 * N)
    for i in range(N):
        u0[2*i] = 0.5
        
    bounds = [(-v_max, v_max) if i % 2 == 0 else (-omega_max, omega_max) for i in range(2 * N)]
    
    res = minimize(
        mpc_cost_function,
        u0,
        args=(current_pos, current_yaw, target_pos, obstacles),
        method='L-BFGS-B',
        bounds=bounds,
        options={'maxiter': 8}
    )
    
    optimal_u = res.x
    
    x, y, yaw = current_pos[0], current_pos[1], current_yaw
    trajectory = [(x, y)]
    for i in range(N):
        v = optimal_u[2*i]
        w = optimal_u[2*i+1]
        x += v * np.sin(yaw) * dt
        y += -v * np.cos(yaw) * dt
        yaw += w * dt
        trajectory.append((x, y))
        
    return optimal_u[0], optimal_u[1], trajectory

def main():
    import argparse
    parser = argparse.ArgumentParser(description="V8: Multi-Algorithm Comparison Simulation")
    parser.add_argument('--headless', action='store_true', help="Run without GUI")
    args = parser.parse_args()

    mode = p.DIRECT if args.headless else p.GUI
    client = p.connect(mode)
    
    if mode == p.GUI:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=client)
        p.resetDebugVisualizerCamera(cameraDistance=15.0, cameraYaw=-45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0], physicsClientId=client)

    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    p.setGravity(0, 0, -9.81, physicsClientId=client)
    time_step = 1.0 / 240.0
    p.setTimeStep(time_step, physicsClientId=client)
    plane_id = p.loadURDF("plane.urdf", physicsClientId=client)

    # 1. Spawn Arena obstacles
    np.random.seed(42)
    num_obstacles = 24
    obstacles_data = []
    
    start_point = np.array([-5.0, -5.0])
    target_point = np.array([5.0, 5.0])
    
    for i in range(num_obstacles):
        w = np.random.uniform(0.3, 0.5)
        l = np.random.uniform(0.3, 0.5)
        while True:
            x = np.random.uniform(-4.0, 4.0)
            y = np.random.uniform(-4.0, 4.0)
            pos = np.array([x, y])
            if np.linalg.norm(pos - start_point) > 2.0 and np.linalg.norm(pos - target_point) > 2.0:
                too_close = False
                for obs_pos, obs_w, obs_l in obstacles_data:
                    required_dist = 0.50 + (max(w, l) + max(obs_w, obs_l)) / 2
                    if np.linalg.norm(pos - obs_pos) < required_dist:
                        too_close = True
                        break
                if not too_close:
                    break
        h = np.random.uniform(0.18, 0.35)
        
        col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[w/2, l/2, h/2], physicsClientId=client)
        vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[w/2, l/2, h/2], rgbaColor=[0.7, 0.6, 0.5, 1], physicsClientId=client)
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col_id, baseVisualShapeIndex=vis_id, basePosition=[pos[0], pos[1], h/2], physicsClientId=client)
        obstacles_data.append((pos, w, l))

    # Spawn visual target green beacon
    target_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.15, length=1.2, rgbaColor=[0, 1, 0, 0.8], physicsClientId=client)
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=target_visual, basePosition=[target_point[0], target_point[1], 0.6], physicsClientId=client)

    # 2. Plan global paths for V5 & V6
    print("[INFO] Planning global paths for V5 (A*) and V6 (Dijkstra)...")
    grid_map = GridMap(safe_margin=0.15)
    for obs in obstacles_data:
        grid_map.add_box_obstacle(obs[0][0], obs[0][1], obs[1], obs[2])
    
    astar_path = a_star_search(grid_map, start_point, target_point)
    dijkstra_path = dijkstra_search(grid_map, start_point, target_point)

    # Draw Global Path lines in GUI
    if mode == p.GUI:
        for i in range(len(astar_path) - 1):
            p.addUserDebugLine([astar_path[i][0], astar_path[i][1], 0.05], [astar_path[i+1][0], astar_path[i+1][1], 0.05], [0, 0.8, 0], 3, physicsClientId=client)
        # Draw nominal path (straight line) for V4
        p.addUserDebugLine([start_point[0], start_point[1], 0.02], [target_point[0], target_point[1], 0.02], [0.8, 0, 0], 1, physicsClientId=client)

    # 3. Summon 4 Robots
    # V4: Red, V5: Green, V6: Blue, V7: Black
    colors = {
        'v4': [1.0, 0.0, 0.0, 1.0],
        'v5': [0.0, 0.8, 0.0, 1.0],
        'v6': [0.0, 0.0, 1.0, 1.0],
        'v7': [0.1, 0.1, 0.1, 1.0]
    }
    
    robot_ids = {}
    gaits = {}
    joint_indices = {}
    
    # Load URDFs
    urdf_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "assets", "spider.urdf"))
    for name in ['v4', 'v5', 'v6', 'v7']:
        r_id = p.loadURDF(urdf_path, [start_point[0], start_point[1], 0.15], p.getQuaternionFromEuler([0, 0, np.pi/4]), physicsClientId=client)
        robot_ids[name] = r_id
        color_robot(r_id, colors[name], client)
        
        # Configure gait
        gaits[name] = GaitControllerV2(step_length=0.07, step_height=0.025, frequency=5.0, body_z=-0.060)
        
        # Joint mapping
        joint_indices[name] = {}
        for j in range(p.getNumJoints(r_id, physicsClientId=client)):
            info = p.getJointInfo(r_id, j, physicsClientId=client)
            j_name = info[1].decode('utf-8')
            if info[2] == p.JOINT_REVOLUTE:
                joint_indices[name][j_name] = j

    # Disable collisions between the 4 robots
    r_list = list(robot_ids.values())
    for i in range(len(r_list)):
        for j in range(i+1, len(r_list)):
            for link_a in range(-1, p.getNumJoints(r_list[i], physicsClientId=client)):
                for link_b in range(-1, p.getNumJoints(r_list[j], physicsClientId=client)):
                    p.setCollisionFilterPair(r_list[i], r_list[j], link_a, link_b, 0, physicsClientId=client)

    # 4. Simulation tracking states
    ray_length = 0.45
    ray_angles = np.linspace(-np.pi, np.pi, 16, endpoint=False)
    
    # State tracking histories for comparison plotting
    history = {
        'v4': {'x': [], 'y': [], 'dist': [], 'time': []},
        'v5': {'x': [], 'y': [], 'dist': [], 'time': []},
        'v6': {'x': [], 'y': [], 'dist': [], 'time': []},
        'v7': {'x': [], 'y': [], 'dist': [], 'time': []}
    }
    
    # V5/V6 Pure Pursuit indices
    target_idx_v5 = 0
    target_idx_v6 = 0
    
    # V7 MPC variables
    last_mpc_time = -1.0
    opt_v, opt_w = 0.0, 0.0
    traj_line_ids = []
    
    # V4 APF state variables
    curl_sign = None
    last_check_pos_v4 = np.array([-5.0, -5.0])
    last_check_time_v4 = 5.0
    escape_timer_v4 = 0.0
    escape_mode_v4 = False
    escape_cooldown_v4 = 0.0

    # Completion flags
    finished = {name: False for name in ['v4', 'v5', 'v6', 'v7']}
    completion_times = {name: None for name in ['v4', 'v5', 'v6', 'v7']}

    sim_time = 0.0
    print("[INFO] Autopilot engaged. Running 4 algorithms simultaneously...")

    try:
        while sim_time < 120.0:
            # Check if all completed
            if all(finished.values()):
                print("[INFO] All robots have completed their mission!")
                break
                
            for name, r_id in robot_ids.items():
                pos, orn = p.getBasePositionAndOrientation(r_id, physicsClientId=client)
                euler = p.getEulerFromQuaternion(orn)
                yaw = euler[2]
                
                dist_to_target = np.linalg.norm(np.array(pos[:2]) - target_point)
                
                # Check for completion
                if dist_to_target < 0.4 and not finished[name]:
                    finished[name] = True
                    completion_times[name] = sim_time
                    print(f"[{name.upper()} SUCCESS] Target reached in {sim_time:.2f} seconds!")
                
                if finished[name]:
                    # Just stand still
                    joint_targets = gaits[name].get_joint_targets(sim_time, direction=(0.0, 0.0), yaw_rate=0.0)
                    for joint_name, target_angle in joint_targets.items():
                        p.setJointMotorControl2(r_id, joint_indices[name][joint_name], p.POSITION_CONTROL, targetPosition=target_angle, force=1.2, maxVelocity=3.0, physicsClientId=client)
                    continue

                # Record history
                history[name]['x'].append(pos[0])
                history[name]['y'].append(pos[1])
                history[name]['dist'].append(dist_to_target)
                history[name]['time'].append(sim_time)

                # 16-ray LiDAR Scan
                dist_front = ray_length
                dist_left = ray_length
                dist_right = ray_length
                
                for j, angle in enumerate(ray_angles):
                    r_start = [pos[0], pos[1], pos[2] + 0.08]
                    ray_yaw = yaw + angle
                    r_end = [
                        pos[0] + ray_length * np.sin(ray_yaw),
                        pos[1] - ray_length * np.cos(ray_yaw),
                        pos[2] + 0.08
                    ]
                    
                    hit_object, hit_fraction, hit_pos = cast_ray_recursive(client, r_start, r_end, list(robot_ids.values()), plane_id)
                    
                    if hit_object != -1 and hit_object != plane_id:
                        hit_dist = hit_fraction * ray_length
                        if -np.pi/4 <= angle <= np.pi/4:
                            dist_front = min(dist_front, hit_dist)
                        elif np.pi/4 < angle <= 3*np.pi/4:
                            dist_left = min(dist_left, hit_dist)
                        elif -3*np.pi/4 <= angle < -np.pi/4:
                            dist_right = min(dist_right, hit_dist)

                dx, dy = 0.0, 0.0
                yaw_rate = 0.0

                # ------------------- V4: APF -------------------
                if name == 'v4':
                    # Pure pursuit target heading
                    heading_to_target = np.arctan2(target_point[1] - pos[1], target_point[0] - pos[0])
                    heading_error = heading_to_target - (yaw - np.pi/2)
                    heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
                    
                    # V4 Stuck recovery logic
                    if sim_time - last_check_time_v4 >= 3.0:
                        displacement = np.linalg.norm(np.array(pos[:2]) - last_check_pos_v4)
                        if displacement < 0.08:
                            escape_mode_v4 = True
                            escape_timer_v4 = 1.0
                            curl_sign = -curl_sign if curl_sign is not None else 1.0
                        last_check_pos_v4 = np.array(pos[:2])
                        last_check_time_v4 = sim_time
                        
                    if escape_timer_v4 > 0.0:
                        dy = 0.1
                        yaw_rate = 3.5 * curl_sign if curl_sign is not None else 3.5
                        escape_timer_v4 -= time_step
                    else:
                        if dist_front < 0.40 or dist_left < 0.40 or dist_right < 0.40:
                            if dist_front < 0.20:
                                dy = 0.0
                                yaw_rate = 2.0 if dist_right < dist_left else -2.0
                            else:
                                active_dist = min(dist_front, dist_left, dist_right)
                                proximity_factor = np.clip((0.40 - active_dist) / 0.20, 0.0, 1.0)
                                dy = -0.5 * (1.0 - 0.3 * proximity_factor)
                                yaw_rate = 1.8 * proximity_factor * (1.0 if dist_right < dist_left else -1.0)
                        else:
                            if np.abs(heading_error) > 1.4:
                                yaw_rate = np.clip(2.0 * heading_error, -1.5, 1.5)
                                dx, dy = 0.0, 0.0
                            else:
                                yaw_rate = np.clip(1.8 * heading_error, -1.2, 1.2)
                                dy = -v_max * max(0.3, np.cos(heading_error))

                # ------------------- V5: A* -------------------
                elif name == 'v5':
                    if len(astar_path) > 0:
                        robot_heading_angle = yaw - np.pi/2
                        robot_heading_angle = np.arctan2(np.sin(robot_heading_angle), np.cos(robot_heading_angle))
                        target_point_v5, heading_error, is_last_v5 = pure_pursuit(pos[:2], robot_heading_angle, astar_path, lookahead_dist=0.45)
                        
                        if dist_front < 0.40 or dist_left < 0.40 or dist_right < 0.40:
                            if dist_front < 0.20:
                                dy = 0.0
                                yaw_rate = 2.0 if dist_right < dist_left else -2.0
                            else:
                                active_dist = min(dist_front, dist_left, dist_right)
                                proximity_factor = np.clip((0.40 - active_dist) / 0.20, 0.0, 1.0)
                                dy = -0.5 * (1.0 - 0.3 * proximity_factor)
                                yaw_rate = 1.8 * proximity_factor * (1.0 if dist_right < dist_left else -1.0)
                        else:
                            if np.abs(heading_error) > 1.4:
                                yaw_rate = np.clip(2.0 * heading_error, -1.5, 1.5)
                                dx, dy = 0.0, 0.0
                            else:
                                yaw_rate = np.clip(1.8 * heading_error, -1.2, 1.2)
                                dy = -v_max * max(0.3, np.cos(heading_error))

                # ------------------- V6: Dijkstra -------------------
                elif name == 'v6':
                    if len(dijkstra_path) > 0:
                        robot_heading_angle = yaw - np.pi/2
                        robot_heading_angle = np.arctan2(np.sin(robot_heading_angle), np.cos(robot_heading_angle))
                        target_point_v6, heading_error, is_last_v6 = pure_pursuit(pos[:2], robot_heading_angle, dijkstra_path, lookahead_dist=0.45)
                        
                        if dist_front < 0.40 or dist_left < 0.40 or dist_right < 0.40:
                            if dist_front < 0.20:
                                dy = 0.0
                                yaw_rate = 2.0 if dist_right < dist_left else -2.0
                            else:
                                active_dist = min(dist_front, dist_left, dist_right)
                                proximity_factor = np.clip((0.40 - active_dist) / 0.20, 0.0, 1.0)
                                dy = -0.5 * (1.0 - 0.3 * proximity_factor)
                                yaw_rate = 1.8 * proximity_factor * (1.0 if dist_right < dist_left else -1.0)
                        else:
                            if np.abs(heading_error) > 1.4:
                                yaw_rate = np.clip(2.0 * heading_error, -1.5, 1.5)
                                dx, dy = 0.0, 0.0
                            else:
                                yaw_rate = np.clip(1.8 * heading_error, -1.2, 1.2)
                                dy = -v_max * max(0.3, np.cos(heading_error))

                # ------------------- V7: MPC -------------------
                elif name == 'v7':
                    if sim_time - last_mpc_time >= 0.3:
                        # Hybrid MPC: Track the A* global path waypoints dynamically
                        robot_heading_angle = yaw - np.pi/2
                        robot_heading_angle = np.arctan2(np.sin(robot_heading_angle), np.cos(robot_heading_angle))
                        target_point_v7, _, _ = pure_pursuit(pos[:2], robot_heading_angle, astar_path, lookahead_dist=1.60)
                        
                        opt_v, opt_w, pred_trajectory = solve_mpc(np.array(pos[:2]), yaw, target_point_v7, obstacles_data)
                        last_mpc_time = sim_time
                        
                        # Dynamic trajectory update in GUI
                        if mode == p.GUI:
                            for l_id in traj_line_ids:
                                p.removeUserDebugItem(l_id, physicsClientId=client)
                            traj_line_ids.clear()
                            for k in range(len(pred_trajectory) - 1):
                                l_id = p.addUserDebugLine(
                                    [pred_trajectory[k][0], pred_trajectory[k][1], 0.05],
                                    [pred_trajectory[k+1][0], pred_trajectory[k+1][1], 0.05],
                                    [0, 0.8, 0.8], 3, physicsClientId=client
                                )
                                traj_line_ids.append(l_id)
                                
                    if dist_front < 0.40 or dist_left < 0.40 or dist_right < 0.40:
                        if dist_front < 0.20:
                            dy = 0.0
                            yaw_rate = 2.0 if dist_right < dist_left else -2.0
                        else:
                            active_dist = min(dist_front, dist_left, dist_right)
                            proximity_factor = np.clip((0.40 - active_dist) / 0.20, 0.0, 1.0)
                            dy = -0.5 * (1.0 - 0.3 * proximity_factor)
                            yaw_rate = 1.8 * proximity_factor * (1.0 if dist_right < dist_left else -1.0)
                    else:
                        dx, dy = 0.0, -opt_v
                        yaw_rate = opt_w

                # Set joint targets
                joint_targets = gaits[name].get_joint_targets(sim_time, direction=(dx, dy), yaw_rate=yaw_rate)
                for joint_name, target_angle in joint_targets.items():
                    if joint_name in joint_indices[name]:
                        p.setJointMotorControl2(
                            r_id,
                            joint_indices[name][joint_name],
                            p.POSITION_CONTROL,
                            targetPosition=target_angle,
                            force=1.2,
                            maxVelocity=3.0,
                            physicsClientId=client
                        )

            # Step PyBullet
            p.stepSimulation(physicsClientId=client)
            if mode == p.GUI:
                time.sleep(time_step / 2.0)
                
            sim_time += time_step

    except KeyboardInterrupt:
        print("[INFO] Simulation interrupted by user.")
    finally:
        p.disconnect(physicsClientId=client)
        print("[INFO] PyBullet disconnected.")

    # 5. Generate Comparison Plot and save it
    print("\n[INFO] Generating comparison plot...")
    plt.figure(figsize=(12, 6))
    
    # Trajectory Plot
    plt.subplot(1, 2, 1)
    # Plot obstacles
    for obs in obstacles_data:
        cx, cy = obs[0][0], obs[0][1]
        w, l = obs[1], obs[2]
        rect = plt.Rectangle((cx - w/2, cy - l/2), w, l, color='gray', alpha=0.5, label='Obstacle' if 'Obstacle' not in plt.gca().get_legend_handles_labels()[1] else "")
        plt.gca().add_patch(rect)
        
    # Plot target and start
    plt.plot(start_point[0], start_point[1], 'go', markersize=10, label='Start')
    plt.plot(target_point[0], target_point[1], 'ro', markersize=10, label='Target')
    
    # Plot paths
    labels = {'v4': 'V4 (APF) - Red', 'v5': 'V5 (A*) - Green', 'v6': 'V6 (Dijkstra) - Blue', 'v7': 'V7 (MPC) - Black'}
    plot_colors = {'v4': 'red', 'v5': 'green', 'v6': 'blue', 'v7': 'black'}
    
    for name in ['v4', 'v5', 'v6', 'v7']:
        if len(history[name]['x']) > 0:
            plt.plot(history[name]['x'], history[name]['y'], color=plot_colors[name], linewidth=2, label=labels[name])
            
    plt.xlim(-6.0, 6.0)
    plt.ylim(-6.0, 6.0)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Trajectory Comparison')
    plt.grid(True)
    plt.legend()
    
    # Distance vs Time Plot
    plt.subplot(1, 2, 2)
    for name in ['v4', 'v5', 'v6', 'v7']:
        if len(history[name]['time']) > 0:
            plt.plot(history[name]['time'], history[name]['dist'], color=plot_colors[name], linewidth=2, label=labels[name])
            
    plt.xlabel('Simulation Time (s)')
    plt.ylabel('Distance to Target (m)')
    plt.title('Distance to Target vs Time')
    plt.grid(True)
    plt.legend()
    
    plt.tight_layout()
    plot_filepath = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "docs", "trajectory_comparison.png"))
    plt.savefig(plot_filepath, dpi=150)
    plt.close()
    
    print("==================================================")
    print("             RACE RESULTS SUMMARY                 ")
    print("==================================================")
    for name in ['v4', 'v5', 'v6', 'v7']:
        time_taken = completion_times[name]
        status = f"SUCCESS in {time_taken:.2f}s" if time_taken is not None else "TIMEOUT / FAILED"
        print(f" - {labels[name]}: {status}")
    print("==================================================")
    print(f"[INFO] Comparison plot successfully saved as: {plot_filepath}")

if __name__ == "__main__":
    main()
