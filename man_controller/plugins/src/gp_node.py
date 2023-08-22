#!/usr/bin/env python

import rospy
import GPy
import numpy as np
from std_msgs.msg import Float64MultiArray

class GPFittingNode:
    def __init__(self):
        rospy.init_node('gp_fitting_node')

        # Subscribe to the topics for states and observations
        rospy.Subscriber('states_topic', Float64MultiArray, self.states_callback)
        rospy.Subscriber('observations_topic', Float64MultiArray, self.observations_callback)

        # Publisher for predictions
        self.prediction_pub = rospy.Publisher('gp_predictions', Float64MultiArray, queue_size=10)

        # Initialize GP model (you may need to adjust the kernel and other parameters)
        self.kernel = GPy.kern.RBF(input_dim=1)
        self.gp_model = None

        # Buffers for states and observations
        self.states_buffer = []
        self.observations_buffer = []

        # Timer for fitting GP at 10 Hz
        rospy.Timer(rospy.Duration(0.1), self.fit_gp)

    def states_callback(self, msg):
        self.states_buffer.append(msg.data)
        if self.gp_model:
            prediction = self.gp_model.predict(np.array(msg.data).reshape(-1, 1))
            prediction_msg = Float64MultiArray(data=prediction[0].flatten())
            self.prediction_pub.publish(prediction_msg)

    def observations_callback(self, msg):
        self.observations_buffer.append(msg.data)

    def fit_gp(self, event):
        if len(self.states_buffer) > 0 and len(self.observations_buffer) > 0:
            X = np.array(self.states_buffer).reshape(-1, 1)
            Y = np.array(self.observations_buffer).reshape(-1, 1)
            self.gp_model = GPy.models.GPRegression(X, Y, self.kernel)
            self.gp_model.optimize(messages=True)
            self.states_buffer = []
            self.observations_buffer = []

if __name__ == '__main__':
    try:
        node = GPFittingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
