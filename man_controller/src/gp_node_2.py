#!/usr/bin/env python

import rospy
import gpflow
import numpy as np
import time as pkg_time
from std_msgs.msg import Float64MultiArray
from man_controller.msg import FloatArray
from gpflow.utilities import print_summary
from scipy.linalg import solve_continuous_are
import tensorflow as tf


class GPFittingNode:
    def __init__(self):

        self.buffer_size = 50
        self.tuning_buffer_size = 500
        self.skip = 0
        self.skip_obs = 0
        # Buffers for states and observations
        self.states_buffer = []
        self.ranked_list = []

        self.tuning_state_buffer = []
        self.tuning_obs1_buffer = []
        self.tuning_obs_time_buffer = []
        self.tuning_state_time_buffer = []
        

        self.observation1_buffer = []

        self.error_buffer = []

        self.gp_model1 = None
        self.posterior1 = None


        self.tuned = False

        rospy.init_node('gp_fitting_node')

        print("Node started")

        # Subscribe to the topics for states and observations
        rospy.Subscriber('states_topic', FloatArray, self.states_callback, queue_size=1)
        rospy.Subscriber('error_states', Float64MultiArray, self.errors_callback)
        rospy.Subscriber('observations_topic', FloatArray, self.observations_callback)
        rospy.Subscriber('train_data', FloatArray, self.training_callback, queue_size=1)

        # Publisher for predictions
        self.prediction_pub = rospy.Publisher('gp_mean3', FloatArray, queue_size=10)
        self.correction_pub = rospy.Publisher('gp_var3', FloatArray, queue_size=10)

        # Initialize GP model (you may need to adjust the kernel and other parameters)
        self.kernel1 = gpflow.kernels.RBF(lengthscales=[1, 1, 1, 1, 1, 1, 1, 1, 1])


        # Timer for fitting GP at 10 Hz
        rospy.Timer(rospy.Duration(0.2), self.fit_gp)

    def get_correction(self, mean1, mean2, mean3, var1, var2, var3, error):

        print("Obtaining correction")
        
        rho1 = max(abs(mean1 - 2.5*var1), abs(mean1 + 2.5*var1))
        rho2 = max(abs(mean2 - 2.5*var2), abs(mean2 + 2.5*var2))
        rho3 = max(abs(mean3 - 2.5*var3), abs(mean3 + 2.5*var3))

        rho = rho1 * rho1 + rho2 * rho2 + rho3 * rho3
        rho = rho**0.5

        B = np.zeros([6, 3])
        B[3:6, 0:3] = np.identity(3)

        # print(B)
        K_P = np.array([[26.5948*9*5, 0, 0], 
                        [0, 26.5948*9*5, 0], 
                        [0, 0, 26.5948*9*5]])
        
        K_D = np.array([[23.0629*3, 0, 0], 
                        [0, 23.0629*3, 0], 
                        [0, 0, 23.0629*3]])
        

        A = np.zeros([6, 6])
        A[3:6, 0:3] = -K_P
        A[3:6, 3:6] = -K_D
        A[0:3, 3:6] = np.identity(3)
        epsilon = 0.1
        Q = np.identity(6)

        P = solve_continuous_are(a = A, b = np.zeros([6, 6]), r = np.identity(6), q = Q)

        w =  B.T @ P @ error
        if np.linalg.norm(w)>epsilon:
            r = -rho * w/np.linalg.norm(w)
        else:
            r = -rho * w/epsilon

        # print("r is: \n", r)

        return r

    def training_callback(self, msg):

        # print("Training callback, size: ", len(msg.data))
        if len(self.tuning_state_buffer)<self.tuning_buffer_size:
            if self.skip%20 == 0:
                # print("Tuning happening for: ", msg.data[0])
                self.tuning_state_buffer.append([msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4],msg.data[5],msg.data[6],msg.data[7],msg.data[8] ] )
                self.tuning_obs1_buffer.append(msg.data[11])
                # self.tuning_state_time_buffer.append(msg.header.stamp)
            self.skip+=1
        else:
            if self.skip%20 == 0:
                # print("Tuning happening for: ", msg.data[0])
                self.tuning_state_buffer.append([msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4],msg.data[5],msg.data[6],msg.data[7],msg.data[8] ] )
                self.tuning_obs1_buffer.append(msg.data[9])
                self.tuning_obs1_buffer.pop(0)
                self.tuning_state_buffer.pop(0)

                # self.tuning_state_time_buffer.append(msg.header.stamp)
            self.skip+=1
        
        if len(self.observation1_buffer)>=self.buffer_size:
            self.observation1_buffer.append(msg.data[11])
            self.observation1_buffer.pop(0)
            self.states_buffer.append([msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4],msg.data[5],msg.data[6],msg.data[7],msg.data[8] ] )
            self.states_buffer.pop(0)
        else:
            self.observation1_buffer.append(msg.data[11])
            self.states_buffer.append([msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4],msg.data[5],msg.data[6],msg.data[7],msg.data[8] ] )
        

    def rank_states(self, datas, state):
        
        key_val = []
        for i in range(datas.shape[0]):
            dot = np.dot(datas[i], np.array(state))
            key_val.append([dot, i])

        key_val = np.array(key_val)
        key_val = key_val[key_val[:, 0].argsort()]
        # print(key_val[:, 1])
        return key_val[:, 1]


    def states_callback(self, msg):
        
        # print("Calling state callback")

        # if len(self.states_buffer)>=self.buffer_size:
        #     self.states_buffer.append(msg.data)
        #     self.states_buffer.pop(0)
        # else:
        #     self.states_buffer.append(msg.data)
        with tf.device("/cpu:0"):

            start_time = pkg_time.time()
            data = np.expand_dims(np.array(msg.data), axis=0)
            # ranks = list(self.rank_states(np.array(self.tuning_state_buffer), np.array(msg.data)))

            if self.posterior1 and self.tuned:

                # X = np.array(self.tuning_state_buffer[ranks[0:50]])
                # Y1 = np.array(self.tuning_obs1_buffer[ranks[0:50]]).reshape(-1, 1)

                # print(Y1)
                # print("\Observation expected to learn: ", self.observation1_buffer, " ", self.observation2_buffer, " ", self.observation3_buffer)

                # self.gp_model1 = gpflow.models.GPR(data=(X, Y1), kernel=self.kernel1)
                 

                # print("Data is: ", data.shape)             
                # print(data.shape)
                time = msg.header.stamp
                # print("\nStates being used to get prediction for observation: ", data)
                # start_time = pkg_time.time()

                # mean1, var1 = self.gp_model1.predict_f(data)
                # mean2, var2 = self.gp_model2.predict_f(data)
                # mean3, var3 = self.gp_model3.predict_f(data)

                # print("Time taken to predict: ", start_time - pkg_time.time())
                mean11, var11 = self.posterior1.predict_f(data)

                # print("Time taken to predict using posterior: ", pkg_time.time()-start_time)

                # print("Time of publishing of this data is: ", time)
                # print("@@@@@@@@@@@@@@@@@@@@@@\n\n\n\n\n Prediction is: ", mean1.numpy(), mean2.numpy(), mean3.numpy(), "\Variance is: ", var1.numpy(), var2.numpy(), var3.numpy())
                # print("Posterior calculation is: ",  mean11.numpy())

                # print("\npredicted data is: ", mean1.numpy()[0][0])
                # Calculation of robust corrections

                # error = self.error_buffer[len(self.error_buffer)-1]
                # print("\nError is: ", error)

                # print("\nPrediction is: ", mean1.numpy(), mean2.numpy(), mean3.numpy())
                # print("\Variance is: ", var1.numpy(), var2.numpy(), var3.numpy())

                # correction = self.get_correction(mean1, mean2, mean3, var1, var2, var3, error)

                # correction = correction.numpy()
                # correction_val = FloatArray(data = [correction[0][0], correction[0][1], correction[0][2]])
                # correction_val.header.stamp = rospy.Time.now()
                # correction_val.header.frame_id = 'GP Correction'¡

                variance_val = FloatArray(data = [var11.numpy()[0][0]])
                variance_val.header.stamp = rospy.Time.now()
                variance_val.header.frame_id = 'GP Variance'

                prediction = FloatArray(data=[mean11.numpy()[0][0]])
                prediction.header.stamp = rospy.Time.now()
                prediction.header.frame_id = 'GP Mean'
                
                self.correction_pub.publish(variance_val)
                self.prediction_pub.publish(prediction)

    def observations_callback(self, msg):

        # print("Observation Callback")
        # print("Count is: \n", self.skip_obs)
        # print("Length of tuning_obs1_buffer: \n", len(self.tuning_obs1_buffer))
        if len(self.tuning_obs1_buffer)<self.tuning_buffer_size:
            if self.skip_obs%20 == 0:
                # self.tuning_obs1_buffer.append(msg.data[0])
                self.tuning_obs_time_buffer.append(msg.header.stamp)
                # self.tuning_obs2_buffer.append(msg.data[1])
                # self.tuning_obs3_buffer.append(msg.data[2])
            self.skip_obs+=1


    def errors_callback(self, msg):

        # print("Error state callback")
        if len(self.error_buffer) >= self.buffer_size:
            self.error_buffer.append(msg.data)
            self.error_buffer.pop(0)
            return 
        
        self.error_buffer.append(msg.data)



    def fit_gp(self, event):

        print("Calling fitting function 3")
        # print("Time stamp of latest observation for GP1 input: \n", self.tuning_obs_time_buffer[len(self.tuning_obs_time_buffer)-1])
        # print("Time stamp of latest state for GP1 input: \n", self.tuning_state_time_buffer[len(self.tuning_state_time_buffer)-1])
        
        print(len(self.tuning_obs1_buffer), len(self.tuning_state_buffer), self.tuned)
        with tf.device("/gpu:0"):

            if len(self.tuning_obs1_buffer) == self.tuning_buffer_size and len(self.tuning_state_buffer) == self.tuning_buffer_size and not self.tuned:
                # X = np.array(self.states_buffer).reshape(-1, 1)
                tuning_states = self.tuning_state_buffer[0:self.tuning_buffer_size-1]
                X = np.array(tuning_states)
                tuning_vals = self.tuning_obs1_buffer[0:self.tuning_buffer_size-1]
                Y1 = np.array(tuning_vals).reshape(-1, 1)

                # print(Y1)
                # print("\Observation expected to learn: ", self.observation1_buffer, " ", self.observation2_buffer, " ", self.observation3_buffer)
                # print("Length Scales before GP 3 optimization:\n")
                print(len(tuning_states), len(tuning_vals))

                self.gp_model1 = gpflow.models.GPR(data=(X, Y1), kernel=self.kernel1)

                # print_summary(self.gp_model1)
                # print_summary(self.kernel1)

                opt1 = gpflow.optimizers.Scipy()
                opt2 = gpflow.optimizers.Scipy()
                opt3 = gpflow.optimizers.Scipy()
                
                start_time = pkg_time.time()

                opt1.minimize(self.gp_model1.training_loss, self.gp_model1.trainable_variables)

                print("Time to optimize one gp model: ", start_time - pkg_time.time())


                self.tuned = True

                # print("\nTrainable parameters after optimization GP3 are: \n")
                # print_summary(self.gp_model1)
                # print_summary(self.kernel1)

                # self.posterior1 = self.gp_model1.posterior()

                print("YAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAYYYYYYYY \n\n\n\n TUNNNNNNNNNNEEEEEDDDDD 333333\n\n\n\n")

                # self.states_buffer = []
                # self.observations_buffer = []

        if self.tuned:

            X = np.array(self.states_buffer)
            Y1 = np.array(self.observation1_buffer).reshape(-1, 1)
        #     Y2 = np.array(self.observation2_buffer).reshape(-1, 1)
        #     Y3 = np.array(self.observation3_buffer).reshape(-1, 1)

        #     # print(Y1)
        #     # print("\Observation expected to learn: ", self.observation1_buffer, " ", self.observation2_buffer, " ", self.observation3_buffer)

            self.gp_model1 = gpflow.models.GPR(data=(X, Y1), kernel=self.kernel1)
            self.posterior1 = self.gp_model1.posterior()
        #     self.gp_model2 = gpflow.models.GPR(data=(X, Y2), kernel=self.kernel)
        #     self.gp_model3 = gpflow.models.GPR(data=(X, Y3), kernel=self.kernel)            

if __name__ == '__main__':
    try:
        node = GPFittingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
