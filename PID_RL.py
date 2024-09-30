import gym
from gym import spaces
import numpy as np
import matplotlib.pyplot as plt
from stable_baselines3 import DQN
from stable_baselines3.common.callbacks import BaseCallback


class PIDEnv(gym.Env):
    def __init__(self, target_position=(0, 0), threshold=5):
        super(PIDEnv, self).__init__()
        # 动作空间：kp, ki, kd（连续的浮点数）
        self.action_space = spaces.Box(low=0, high=10, shape=(3,), dtype=np.float32)
        
        # 状态空间：offset_x, offset_y（偏移量）
        self.observation_space = spaces.Box(low=-100, high=100, shape=(2,), dtype=np.float32)
        
        # 初始化Marker位置和目标位置
        self.marker_position = np.random.uniform(-100, 100, size=2)
        self.target_position = np.array(target_position)
        self.threshold = threshold  # 偏移阈值

    def reset(self):
        # 随机初始化Marker位置
        self.marker_position = np.random.uniform(-100, 100, size=2)
        return self._get_state()

    def _get_state(self):
        # 返回Marker相对于目标位置的偏移
        offset = self.marker_position - self.target_position
        return offset

    def step(self, action):
        kp, ki, kd = action

        # 计算偏移
        offset = self._get_state()
        error_x, error_y = offset
        
        # 模拟PID对偏移的调整
        if np.abs(error_x) > self.threshold or np.abs(error_y) > self.threshold:
            self.marker_position[0] -= kp * 0.1 + ki * 0.01 + kd * 0.001
            self.marker_position[1] -= kp * 0.1 + ki * 0.01 + kd * 0.001

        # 计算新的偏移量
        new_offset = self._get_state()

        # 奖励：越接近目标位置奖励越高
        reward = - (np.abs(new_offset[0]) + np.abs(new_offset[1]))

        # 如果Marker已经接近目标位置，则认为完成
        done = np.all(np.abs(new_offset) < self.threshold)

        # 返回新的状态，奖励，是否完成
        return new_offset, reward, done, {}

    def render(self, mode='human'):
        pass  # 可选：在这里实现可视化


# 自定义回调类，用于可视化训练过程
class PlottingCallback(BaseCallback):
    def __init__(self, plot_interval=1000, verbose=0):
        super(PlottingCallback, self).__init__(verbose)
        self.plot_interval = plot_interval
        self.episodes = []
        self.rewards = []
        self.losses = []

    def _on_step(self) -> bool:
        if self.n_calls % self.plot_interval == 0:
            # 记录奖励和损失
            episode_reward = np.sum(self.locals['rewards'])
            self.episodes.append(len(self.episodes) + 1)
            self.rewards.append(episode_reward)
            loss = np.mean(self.locals['loss'])
            self.losses.append(loss)

            # 绘制奖励和损失
            plt.figure(figsize=(10, 5))
            plt.subplot(121)
            plt.plot(self.episodes, self.rewards, label='Rewards')
            plt.xlabel('Episode')
            plt.ylabel('Reward')
            plt.title('Rewards Over Time')
            plt.legend()

            plt.subplot(122)
            plt.plot(self.episodes, self.losses, label='Losses')
            plt.xlabel('Episode')
            plt.ylabel('Loss')
            plt.title('Losses Over Time')
            plt.legend()

            plt.tight_layout()
            plt.show()
        return True


# 初始化环境
env = PIDEnv()

# 创建DQN模型
model = DQN("MlpPolicy", env, verbose=1, learning_rate=1e-3)

# 创建自定义回调，用于可视化训练过程
plot_callback = PlottingCallback(plot_interval=1000)

# 训练模型
model.learn(total_timesteps=10000, callback=plot_callback)

# 保存训练好的模型
model.save("pid_dqn_agent")

# 测试模型
obs = env.reset()
for _ in range(100):
    action, _states = model.predict(obs)
    obs, reward, done, info = env.step(action)
    print(f"Action: {action}, Obs: {obs}, Reward: {reward}")
    if done:
        print("Target reached!")
        break
