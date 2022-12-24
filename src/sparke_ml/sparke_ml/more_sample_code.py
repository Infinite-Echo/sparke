import numpy as np
import rclpy

from rl_algorithm import RLAlgorithm

class RobotDog(RLAlgorithm):
    def __init__(self, env):
        super().__init__(env)
        self.q_table = np.zeros((self.n_states, self.n_actions))

    def choose_action(self, state):
        # Epsilon-greedy action selection
        if np.random.uniform(0, 1) < self.epsilon:
            # Explore: choose a random action
            return self.env.action_space.sample()
        else:
            # Exploit: choose the action with the maximum expected reward
            return np.argmax(self.q_table[state, :])

    def learn(self, state, action, reward, next_state, done):
        # Update the Q-value for the current state and action
        self.q_table[state, action] = (1 - self.alpha) * self.q_table[state, action] + self.alpha * (reward + self.gamma * np.max(self.q_table[next_state, :]))
        # Decrease the exploration rate
        self.epsilon = self.epsilon * self.epsilon_decay

def main(args=None):
    rclpy.init(args=args)
    env = RobotDogEnv()
    robot_dog = RobotDog(env)
    while not rclpy.ok():
        # Run the simulation and allow the robot to take actions
        state = env.reset()
        while True:
            action = robot_dog.choose_action(state)
            next_state, reward, done, _ = env.step(action)
            robot_dog.learn(state, action, reward, next_state, done)
            state = next_state
            if done:
                break
    rclpy.shutdown()

if __name__ == '__main__':
    main()
