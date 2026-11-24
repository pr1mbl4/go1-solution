import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from unitree_go.msg import LowState, LowCmd
import numpy as np
import torch
import os

class RLControllerNode(Node):
    def __init__(self):
        super().__init__('rl_controller')
        
        # --- 1. Загрузка модели ---
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(pkg_dir, 'motion.pt')
        if not os.path.exists(model_path):
            self.get_logger().error(f"Модель не найдена: {model_path}")
            raise FileNotFoundError(model_path)
            
        self.device = torch.device('cpu')
        # Используем jit.load для .pt файлов от RSL_RL/LeggedGym
        try:
            self.policy = torch.jit.load(model_path, map_location=self.device)
            self.policy.eval()
            self.get_logger().info("RL Policy загружена успешно")
        except Exception as e:
            self.get_logger().error(f"Ошибка загрузки модели: {e}")
            raise e

        # --- 2. Подписки ---
        qos_lowstate = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.lowstate_sub = self.create_subscription(
            LowState, '/lowstate', self.lowstate_callback, qos_lowstate
        )
        
        qos_cmd_vel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos_cmd_vel
        )
        
        # --- 3. Публикация ---
        qos_bare = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.lowcmd_pub = self.create_publisher(LowCmd, '/lowcmd_bare', qos_bare)
        
        # --- 4. Параметры среды (Go1 / G1 Config) ---
        # Дефолтная поза Go1 (из unitree_rl_gym / legged_gym)
        self.default_dof_pos = np.array([
            0.0, 0.8536, -1.7072,  # FR
            0.0, 0.8536, -1.7072,  # FL
            0.0, 0.8536, -1.7072,  # RR
            0.0, 0.8536, -1.7072   # RL
        ], dtype=np.float32)
        
        # Размерности (теперь 47, как требует модель)
        self.num_obs = 47 
        self.num_actions = 12
        
        # Масштаб действий (КРИТИЧЕСКИ ВАЖНО, берем из g1_config.py)
        self.action_scale = 0.25
        
        # PD коэффициенты (берем усредненные/стандартные для Go1, 
        # в g1_config они другие, но для Go1 в симуляторе нужны такие)
        self.kp = 40.0
        self.kd = 2.0
        
        # Состояние
        self.last_lowstate = None
        self.target_speed_x = 0.0
        self.prev_actions = np.zeros(self.num_actions, dtype=np.float32)
        
        # Частота управления RL (обычно 50 Гц -> 0.02 сек)
        self.timer = self.create_timer(0.02, self.control_loop) 
        self.get_logger().info("RL Controller запущен (47 obs, LSTM)")

    def cmd_vel_callback(self, msg):
        self.target_speed_x = float(msg.linear.x)

    def lowstate_callback(self, msg):
        self.last_lowstate = msg

    def control_loop(self):
        if self.last_lowstate is None:
            return
            
        try:
            # --- Сборка Observation Vector (47 элементов) ---
            obs = np.zeros(self.num_obs, dtype=np.float32)
            ms = self.last_lowstate.motor_state
            imu = self.last_lowstate.imu_state
            
            # 0-2: Угловая скорость базы (gyroscope)
            # В ROS2 LowState gyroscope - это [x, y, z]
            obs[0:3] = [imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2]]
            
            # 3-5: Проекция гравитации (из кватерниона)
            # Кватернион в LowState: [x, y, z, w]
            q = imu.quaternion 
            x, y, z, w = q[0], q[1], q[2], q[3]
            
            # R_base_world @ [0, 0, -1]
            obs[3] = 2.0 * (x * z - w * y)
            obs[4] = 2.0 * (y * z + w * x)
            obs[5] = 1.0 - 2.0 * (x * x + y * y)
            
            # 6-8: Команды скорости (нормализованные)
            # Обычно в legged_gym команды нормализуются на max_vel (2.0 для lin, 2*pi для ang)
            obs[6] = self.target_speed_x / 2.0
            obs[7] = 0.0 # linear.y
            obs[8] = 0.0 # angular.z
            
            # 9-20: Позиции суставов (dof_pos)
            for i in range(12):
                obs[9 + i] = ms[i].q
                
            # 21-32: Скорости суставов (dof_vel)
            for i in range(12):
                obs[21 + i] = ms[i].dq
                
            # 33-44: Предыдущие действия (prev_actions)
            obs[33:45] = self.prev_actions
            
            # 45-46: Дополнительные признаки (обычно base_lin_vel x, y)
            # Так как в /lowstate нет одометрии, ставим 0. 
            # Модель должна быть робастна к этому, либо это были другие признаки.
            obs[45] = 0.0 
            obs[46] = 0.0
            
            # --- Инференс ---
            with torch.no_grad():
                obs_tensor = torch.from_numpy(obs).unsqueeze(0).to(self.device)
                # Модель с LSTM внутри сама обновляет свое скрытое состояние
                actions_tensor = self.policy(obs_tensor)
                actions = actions_tensor.cpu().numpy()[0]
                
            # Сохраняем действия для следующего шага
            self.prev_actions = actions.copy()
            
            # --- Формирование LowCmd ---
            # target_q = default_pos + action * action_scale
            target_q = self.default_dof_pos + actions * self.action_scale
            
            cmd_msg = LowCmd()
            cmd_msg.head = [0xFE, 0xEF]
            cmd_msg.level_flag = 0xFF
            
            for i in range(12):
                motor = cmd_msg.motor_cmd[i]
                motor.mode = 0x0A # SERVO mode
                motor.q = float(target_q[i])
                motor.dq = 0.0
                motor.tau = 0.0
                motor.kp = self.kp
                motor.kd = self.kd
                
            self.lowcmd_pub.publish(cmd_msg)
            
        except Exception as e:
            self.get_logger().error(f"Ошибка в control_loop: {e}")

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
