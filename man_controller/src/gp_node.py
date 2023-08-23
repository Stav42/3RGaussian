#!/usr/bin/env python

import rospy
import gpflow
import numpy as np
from std_msgs.msg import Float64MultiArray

class GPFittingNode:
    def __init__(self):


        # Buffers for states and observations
        self.states_buffer = []
        self.observation1_buffer = []
        self.observation2_buffer = []
        self.observation3_buffer = []

        self.gp_model1 = None
        self.gp_model2 = None
        self.gp_model3 = None

        rospy.init_node('gp_fitting_node')

        # Subscribe to the topics for states and observations
        rospy.Subscriber('states_topic', Float64MultiArray, self.states_callback)
        rospy.Subscriber('observations_topic', Float64MultiArray, self.observations_callback)

        # Publisher for predictions
        self.prediction_pub = rospy.Publisher('gp_predictions', Float64MultiArray, queue_size=10)

        # Initialize GP model (you may need to adjust the kernel and other parameters)
        self.kernel = gpflow.kernels.RBF()

        

        # Timer for fitting GP at 10 Hz
        rospy.Timer(rospy.Duration(0.1), self.fit_gp)

    def states_callback(self, msg):
        if len(self.states_buffer)>=20:
            self.states_buffer.append(msg.data)
            self.states_buffer.pop(0)
        else:
            self.states_buffer.append(msg.data)

        if self.gp_model1 and self.gp_model2 and self.gp_model3:
            data = np.expand_dims(np.array(msg.data), axis=0)
             
            print(data.shape)

            mean1, var1 = self.gp_model1.predict_f(data)
            mean2, var2 = self.gp_model2.predict_f(data)
            mean3, var3 = self.gp_model3.predict_f(data)

            print("\npredicted data is: ", mean1.numpy()[0][0])

            prediction = Float64MultiArray(data=[mean1.numpy()[0][0], mean2.numpy()[0][0], mean3.numpy()[0][0]])

            self.prediction_pub.publish(prediction)

    def observations_callback(self, msg):

        if len(self.observation1_buffer) >= 20:
            self.observation1_buffer.append(msg.data[0])
            self.observation1_buffer.pop(0)
            self.observation2_buffer.append(msg.data[1])
            self.observation2_buffer.pop(0)
            self.observation3_buffer.append(msg.data[2])
            self.observation3_buffer.pop(0)
            return 
        
        self.observation1_buffer.append(msg.data[0])
        self.observation2_buffer.append(msg.data[1])
        self.observation3_buffer.append(msg.data[2])



    def fit_gp(self, event):

        print("Calling fitting function")
        if len(self.states_buffer) == 20 and len(self.observation1_buffer) > 0 and len(self.observation2_buffer) > 0 and len(self.observation3_buffer) > 0:
            # X = np.array(self.states_buffer).reshape(-1, 1)
            X = np.array(self.states_buffer)
            Y1 = np.array(self.observation1_buffer).reshape(-1, 1)
            Y2 = np.array(self.observation2_buffer).reshape(-1, 1)
            Y3 = np.array(self.observation3_buffer).reshape(-1, 1)

            self.gp_model1 = gpflow.models.GPR(data=(X, Y1), kernel=self.kernel)
            self.gp_model2 = gpflow.models.GPR(data=(X, Y2), kernel=self.kernel)
            self.gp_model3 = gpflow.models.GPR(data=(X, Y3), kernel=self.kernel)

            opt = gpflow.optimizers.Scipy()
            
            opt.minimize(self.gp_model1.training_loss, self.gp_model1.trainable_variables)
            opt.minimize(self.gp_model2.training_loss, self.gp_model2.trainable_variables)
            opt.minimize(self.gp_model3.training_loss, self.gp_model3.trainable_variables)

            self.states_buffer = []
            self.observations_buffer = []

if __name__ == '__main__':
    try:
        node = GPFittingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
