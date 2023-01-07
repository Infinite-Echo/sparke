import tensorflow as tf
import numpy as np

class SparkeAI:
    def __init__(self, num_joints=12, learning_rate=0.001):
        # Define the policy model
        self.policy_model = tf.keras.Sequential()
        self.policy_model.add(tf.keras.layers.Dense(64, input_shape=(25,), activation='relu'))
        self.policy_model.add(tf.keras.layers.Dense(64, activation='relu'))
        self.policy_model.add(tf.keras.layers.Dense(num_joints))
        
        # Define the critic model
        self.critic_model = tf.keras.Sequential()
        self.critic_model.add(tf.keras.layers.Dense(64, input_shape=(25,), activation='relu'))
        self.critic_model.add(tf.keras.layers.Dense(64, activation='relu'))
        self.critic_model.add(tf.keras.layers.Dense(1))
        
        # Define the optimizers and loss functions
        self.policy_optimizer = tf.keras.optimizers.Adam(learning_rate)
        self.critic_optimizer = tf.keras.optimizers.Adam(learning_rate)
        self.loss_fn = tf.keras.losses.MeanSquaredError()
        
    def predict_policy(self, inputs):
        # Use the policy model to make predictions
        return self.policy_model(inputs, training=False)
    
    def predict_value(self, inputs):
        # Use the critic model to make predictions
        return self.critic_model(inputs, training=False)

    def reward_function(self, state, action):
        # Calculate the reward based on the current state and action
        reward = 0
        
        # Check if the action caused the robot to fall
        if state[2] < 0:
            reward = -1
            
        # Check if the action caused the robot to take a step forward
        if state[0] > 0:
            reward = 1
            
        return reward
    
    @tf.function
    def train_step(self, inputs, targets):
        with tf.GradientTape() as tape:
            # Use the policy model to make predictions
            actions = self.policy_model(inputs, training=True)
            
            # Use the critic model to predict the expected return of the actions
            values = self.critic_model(inputs, training=True)
            
            # Calculate the rewards for each action
            rewards = [self.reward_function(inputs[i], actions[i]) for i in range(inputs.shape[0])]
            
            # Calculate the policy loss
            policy_loss = self.loss_fn(targets, actions)
            
            # Calculate the critic loss
            critic_loss = self.loss_fn(rewards, values)
            
            # Calculate the total loss
            total_loss = policy_loss + critic_loss

        # Backpropagate the error and update the weights
        gradients = tape.gradient(total_loss, self.policy_model.trainable_variables + self.critic_model.trainable_variables)
        self.policy_optimizer.apply_gradients(zip(gradients[:len(self.policy_model.trainable_variables)], self.policy_model.trainable_variables))
        self.critic_optimizer.apply_gradients(zip(gradients[len(self.policy_model.trainable_variables):], self.critic_model.trainable_variables))

        # Return the total loss
        return total_loss

# def main():
#     # Create an instance of the QuadrupedRobot class
#     robot = SparkeAI()

#     # Generate a batch of input states
#     input_states = np.random.rand(1, 18)

#     # Get the predicted joint angles
#     predicted_joint_angles = robot.predict_policy(input_states)

#     # Get the predicted expected returns
#     predicted_expected_returns = robot.predict_value(input_states)

#     print(predicted_joint_angles)
#     print(predicted_expected_returns)


# if __name__ == '__main__':
#     main()
