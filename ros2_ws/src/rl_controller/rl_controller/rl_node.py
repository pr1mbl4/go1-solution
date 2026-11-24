import os
import pickle
import io
import numpy as np
import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from unitree_go.msg import LowState, LowCmd


def class_to_dict(obj):
    if not hasattr(obj, "__dict__"):
        return obj
    result = {}
    for key in dir(obj):
        if key.startswith("_") or key == "terrain":
            continue
        val = getattr(obj, key)
        result[key] = [class_to_dict(v) for v in val] if isinstance(val, list) else class_to_dict(val)
    return result


class CPU_Unpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if module == 'torch.storage' and name == '_load_from_bytes':
            return lambda b: torch.load(io.BytesIO(b), map_location='cpu')
        return super().find_class(module, name)


class RLControllerNode(Node):
    def __init__(self):
        super().__init__('rl_controller')

        wtw_dir = os.path.join(get_package_share_directory('rl_controller'), 'wtw')

        # --- 1. Конфиг ---
        pkl_path = os.path.join(wtw_dir, 'parameters.pkl')
        self.get_logger().info(f"Загрузка конфига из {pkl_path}...")
        try:
            pkl_cfg = torch.load(pkl_path, map_location='cpu', weights_only=False)
        except Exception:
            with open(pkl_path, 'rb') as f:
                pkl_cfg = CPU_Unpickler(f).load()

        cfg = pkl_cfg['Cfg'] if isinstance(pkl_cfg, dict) and 'Cfg' in pkl_cfg else pkl_cfg
        cfg = class_to_dict(cfg)
        self.cfg = cfg

        env = cfg.get('env', {})
        ctl = cfg.get('control', {})
        norm = cfg.get('normalization', {})

        default_scales = {
            'lin_vel': 2.0, 'ang_vel': 0.25, 'dof_pos': 1.0, 'dof_vel': 0.05,
            'body_height_cmd': 1.0, 'footswing_height_cmd': 1.0,
            'body_pitch_cmd': 1.0, 'body_roll_cmd': 1.0,
            'stance_width_cmd': 1.0, 'stance_length_cmd': 1.0, 'aux_reward_cmd': 1.0
        }
        loaded_scales = cfg.get('obs_scales', norm.get('obs_scales', {}))
        self.obs_scales = {**default_scales, **loaded_scales}

        self.num_obs = env.get('num_observations', 70)
        self.hist_len = env.get('num_observation_history', 30)
        self.num_commands = cfg.get('commands', {}).get('num_commands', 15)
        self.clip = norm.get('clip_actions', 100.0)
        
        self.action_scale = 0.18 
        
        self.hip_red = ctl.get('hip_scale_reduction', 0.5)
        self.dt = ctl.get('decimation', 4) * cfg.get('sim', {}).get('dt', 0.005)

        self.two_prev = env.get('observe_two_prev_actions', True)
        self.use_clock = env.get('observe_clock_inputs', True)
        self.observe_vel = env.get('observe_vel', False)

        cs = self.obs_scales
        self.commands_scale = np.array([
            cs['lin_vel'], cs['lin_vel'], cs['ang_vel'], cs.get('body_height_cmd', 1.0),
            1, 1, 1, 1, 1,
            cs.get('footswing_height_cmd', 1.0), cs.get('body_pitch_cmd', 1.0), cs.get('body_roll_cmd', 1.0),
            cs.get('stance_width_cmd', 1.0), cs.get('stance_length_cmd', 1.0), cs.get('aux_reward_cmd', 1.0),
            1, 1, 1, 1, 1, 1
        ])[:self.num_commands]

        # --- 2. Суставы ---
        joint_names = [
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
        ]
        angles = cfg.get('init_state', {}).get('default_joint_angles', {})
        default_angles_map = {
            'FL_hip_joint': -0.1, 'FL_thigh_joint': 0.8, 'FL_calf_joint': -1.5,
            'FR_hip_joint': 0.1, 'FR_thigh_joint': 0.8, 'FR_calf_joint': -1.5,
            'RL_hip_joint': -0.1, 'RL_thigh_joint': 1.0, 'RL_calf_joint': -1.5,
            'RR_hip_joint': 0.1, 'RR_thigh_joint': 1.0, 'RR_calf_joint': -1.5
        }
        final_angles = {**default_angles_map, **angles}
        self.default_gym = np.array([final_angles[n] for n in joint_names], dtype=np.float32)

        self.p_gains = np.full(12, 40.0, dtype=np.float32)
        self.d_gains = np.full(12, 2.0, dtype=np.float32)

        # Маппинг: Gym(FL,FR,RL,RR) -> ROS(FR,FL,RR,RL)
        self.gym_to_ros = np.array([3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8])
        self.ros_to_gym = np.array([3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8])

        # --- 3. Модели ---
        self.body = torch.jit.load(os.path.join(wtw_dir, 'body_latest.jit'), map_location='cpu').eval()
        self.adapt = torch.jit.load(os.path.join(wtw_dir, 'adaptation_module_latest.jit'), map_location='cpu').eval()

        # --- 4. Состояние ---
        self.obs_history = torch.zeros(1, self.hist_len * self.num_obs)
        self.actions = torch.zeros(12)
        self.last_actions = torch.zeros(12)
        self.gait_indices = 0.0
        self.clock = torch.zeros(1, 4)

        self.commands = np.zeros(self.num_commands)
        if self.num_commands > 4: self.commands[4] = 2.5
        if self.num_commands > 5: self.commands[5] = 0.5
        if self.num_commands > 8: self.commands[8] = 0.5
        if self.num_commands > 9: self.commands[9] = 0.08

        self.last_lowstate = None
        self.vx = 0.0
        self.loop_count = 0
        
        self.vel_threshold = 0.05
        
        # Yaw Stabilization: Дифференциальный коэффициент
        # Если yaw < 0 (вправо), нужно повернуть влево.
        # Левые ноги (0, 6) +, Правые ноги (3, 9) -
        self.yaw_kp = 0.5 

        self.create_subscription(LowState, '/lowstate', self.cb_state, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.pub = self.create_publisher(LowCmd, '/lowcmd_bare', QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f'WTW Ready: obs={self.num_obs}, hist={self.hist_len}, cmds={self.num_commands}, dt={self.dt:.3f}, scale={self.action_scale}')

    def cb_cmd(self, msg):
        self.vx = float(msg.linear.x)

    def cb_state(self, msg):
        self.last_lowstate = msg

    def control_loop(self):
        if self.last_lowstate is None:
            return
        try:
            ms = self.last_lowstate.motor_state
            imu = self.last_lowstate.imu_state

            q_ros = np.array([ms[i].q for i in range(12)], dtype=np.float32)
            dq_ros = np.array([ms[i].dq for i in range(12)], dtype=np.float32)
            q = q_ros[self.ros_to_gym]
            dq = dq_ros[self.ros_to_gym]

            q4 = imu.quaternion
            if abs(q4[0]) >= abs(q4[3]):
                w, x, y, z = q4[0], q4[1], q4[2], q4[3]
            else:
                w, x, y, z = q4[3], q4[0], q4[1], q4[2]

            gravity = np.array([
                -2.0 * (x * z - w * y),
                -2.0 * (y * z + w * x),
                -(1.0 - 2.0 * (x * x + y * y))
            ], dtype=np.float32)

            # Вычисляем текущий yaw
            siny_cosp = 2.0 * (w * z + x * y)
            cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
            current_yaw = np.arctan2(siny_cosp, cosy_cosp)

            # --- ЛОГИКА "СТОЯТЬ ИЛИ ИДТИ" ---
            if abs(self.vx) < self.vel_threshold:
                target = self.default_gym.copy()
                self.actions = torch.zeros(12)
                self.last_actions = torch.zeros(12)
                
                q_ros_target = target[self.gym_to_ros]
                cmd = LowCmd()
                cmd.head = [0xFE, 0xEF]
                cmd.level_flag = 0xFF
                for i in range(12):
                    m = cmd.motor_cmd[i]
                    m.mode = 0x0A
                    m.q = float(q_ros_target[i])
                    m.dq = 0.0
                    m.tau = 0.0
                    gym_idx = self.ros_to_gym[i]
                    m.kp = float(self.p_gains[gym_idx])
                    m.kd = float(self.d_gains[gym_idx])
                self.pub.publish(cmd)
                
                self.loop_count += 1
                if self.loop_count % 100 == 1:
                    self.get_logger().info(f'debug: STANDING vx={self.vx:.2f} g_z={gravity[2]:.2f}')
                return

            # --- РЕЖИМ ДВИЖЕНИЯ ---
            self.commands[0] = self.vx
            cmd_scaled = (self.commands * self.commands_scale).reshape(1, -1)

            ob = np.concatenate((
                gravity.reshape(1, -1),
                cmd_scaled,
                ((q - self.default_gym) * self.obs_scales['dof_pos']).reshape(1, -1),
                (dq * self.obs_scales['dof_vel']).reshape(1, -1),
                torch.clip(self.actions, -self.clip, self.clip).numpy().reshape(1, -1)
            ), axis=1)

            if self.two_prev:
                ob = np.concatenate((ob, self.last_actions.numpy().reshape(1, -1)), axis=1)
            if self.use_clock:
                ob = np.concatenate((ob, self.clock.numpy()), axis=1)
            if self.observe_vel:
                ob = np.concatenate((np.zeros((1, 3)), (np.array(imu.gyroscope) * self.obs_scales['ang_vel']).reshape(1, -1), ob), axis=1)

            obs_tensor = torch.tensor(ob, dtype=torch.float32)
            if obs_tensor.shape[1] != self.num_obs:
                self.get_logger().error(f'Obs mismatch: {obs_tensor.shape[1]} vs {self.num_obs}', throttle_duration_sec=5.0)
                return

            self.obs_history = torch.cat((self.obs_history[:, self.num_obs:], obs_tensor), dim=-1)

            with torch.no_grad():
                latent = self.adapt(self.obs_history)
                act = self.body(torch.cat((self.obs_history, latent), dim=-1))

            self.last_actions = self.actions.clone()
            self.actions = torch.clip(act[0], -self.clip, self.clip)

            target = self.actions.numpy() * self.action_scale
            
            # --- DIFFERENTIAL YAW STABILIZATION ---
            # Если yaw < 0 (дрейф вправо), current_yaw отрицательный.
            # Нам нужно повернуть влево.
            # correction = kp * (-current_yaw) -> положительное значение.
            # Левые ноги (0, 6) += correction
            # Правые ноги (3, 9) -= correction
            yaw_error = -current_yaw 
            correction = self.yaw_kp * yaw_error
            
            target[0] += correction  # FL_hip
            target[6] += correction  # RL_hip
            target[3] -= correction  # FR_hip
            target[9] -= correction  # RR_hip
            
            target[[0, 3, 6, 9]] *= self.hip_red
            target = target + self.default_gym
            q_ros_target = target[self.gym_to_ros]

            cmd = LowCmd()
            cmd.head = [0xFE, 0xEF]
            cmd.level_flag = 0xFF
            for i in range(12):
                m = cmd.motor_cmd[i]
                m.mode = 0x0A
                m.q = float(q_ros_target[i])
                m.dq = 0.0
                m.tau = 0.0
                gym_idx = self.ros_to_gym[i]
                m.kp = float(self.p_gains[gym_idx])
                m.kd = float(self.d_gains[gym_idx])
            self.pub.publish(cmd)

            if self.use_clock and self.num_commands > 8:
                f, ph, off, bd = self.commands[4], self.commands[5], self.commands[6], self.commands[7]
                self.gait_indices = (self.gait_indices + self.dt * f) % 1.0
                gi = self.gait_indices
                fi = [gi + ph + off + bd, gi + off, gi + bd, gi + ph]
                self.clock = torch.sin(2 * np.pi * torch.tensor(fi)).reshape(1, 4)

            self.loop_count += 1
            if self.loop_count % 100 == 1:
                self.get_logger().info(f'debug: WALKING vx={self.vx:.2f} g_z={gravity[2]:.2f} yaw={current_yaw:.2f} corr={correction:.3f}')

        except Exception as e:
            self.get_logger().error(f'Loop error: {e}', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = RLControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
