import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import tensorflow as tf
import numpy as np
from tensorflow import keras

class TensorFlowSubscriber(Node):

    def __init__(self):
        super().__init__('tensorflow_subscriber')
        self.model = tf.keras.Sequential([keras.layers.Dense(units=1, input_shape=[1])])
        self.model.compile(optimizer='sgd', loss='mean_squared_error')
        self.xs = np.array([-1.0, 0.0, 1.0, 2.0, 3.0, 4.0], dtype=float)
        self.ys = np.array([-2.0, 1.0, 4.0, 7.0, 10.0, 13.0], dtype=float)
        self.model.fit(self.xs, self.ys, epochs=500)
        self.subscription = self.create_subscription(String, 'tensorflow_input', self.tensorflow_callback, 10)

    def tensorflow_callback(self, msg):
        print(self.model.predict(np.array([[float(msg.data)]])))

def main(args=None):
    rclpy.init(args=args)

    tensorflow_subscriber = TensorFlowSubscriber()

    rclpy.spin(tensorflow_subscriber)

    tensorflow_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
