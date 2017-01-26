#!/usr/bin/env python
import numpy as np
import rosbag
import rospy
import math
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from numpy.linalg import inv
import matplotlib.pyplot as plt

def gps_to_cartesian(latitude,longitude):
    r = 6371000
    x = r*math.cos(math.radians(latitude))*math.cos(math.radians(longitude))
    y = r*math.cos(math.radians(latitude))*math.sin(math.radians(longitude))
    position = [x, y]
    return position

bag = rosbag.Bag('/home/sidharth/Downloads/AD_1/16_Hz.bag')
zindagi_mein_dusri_bar = 0
iteration = 0
position = []
first_time = 1
expected_freq = 25
real_freq = 16
update_after = int(math.ceil((expected_freq - real_freq)/real_freq ))
delta = 0.04#float(1/expected_freq)
t = []
t.append(0.0)
x = []
y = []
newx = []
newy = []
sensor_noise_covariance_matrix = np.matrix([[0.05, 0], [0, 0.05]])
velocity = []
F = np.matrix([[1, delta], [0, 1]])

current_state = np.zeros(shape = (2,2))
previous_state = np.zeros(shape = (2,2))
# matrix_product = array.dot(vector)
temp = 0
for topic, msg, t in bag.read_messages(topics=['/mavros/global_position/global', '/mavros/global_position/raw/gps_vel']):
    if topic == '/mavros/global_position/global':
        #  print msg
        past_secs = msg.header.stamp.secs
        past_nsecs = msg.header.stamp.nsecs
        position = gps_to_cartesian(msg.latitude,msg.longitude)
        cov1 = [msg.position_covariance[0], 0]
        cov2 = [0, msg.position_covariance[4]]
        current_covariance_matrix = np.matrix([cov1,cov2])
        #print(" ")
        #print("New Covariance Matrix")
        #print(current_covariance_matrix)
        #print("Time : secs %d,  nsecs : %d" %(past_secs, past_nsecs))
        #print (" ")
        if first_time:
            first_x = position[0]
            first_y = position[1]
            first_time = 0
        position[0] = position[0] - first_x
        position[1] = position[1] - first_y
        #print("Positon : x %f,  y : %f" %(position[0], position[1]))

        x.append(float(position[0]))
        y.append(float(position[1]))
        '''
        plt.figure(1)
        plt.subplot(211)
        plt.plot(x, t, 'ro')

        plt.subplot(212)
        plt.plot(y, t, 'bo')

        plt.show()
        '''
        iteration = iteration + 1     
        new_value = 1       
        

    if topic == '/mavros/global_position/raw/gps_vel':
        velocity = []
        vel_x = msg.twist.linear.x
        vel_y = msg.twist.linear.y
        velocity.append(vel_x)
        velocity.append(vel_y)        
        #print (" ")
        #print("velocity : x %f,  y : %f" %(velocity[0], velocity[1]))
        current_state = np.matrix([position,velocity])
        #print("New State")
        #print(current_state)
        #print (" ")
        iteration = iteration + 1
        new_value = 1


    if iteration == 2:
        updated_state = current_state
        updated_covariance_matrix = current_covariance_matrix

        previous_state = current_state
        previous_covariane_matrix = current_covariance_matrix

        for i in range(1,update_after+2):

            #print("Estimate loop no. : %d" %i)
            '''
            print("Delta")
            print(delta)
            
            print("Prediction Matrix")
            print(F)
            '''
            current_state = np.dot(F,previous_state)
            #print("Current State")
            #print(current_state)

            F_transpose = F.transpose()
            temp_matrix = np.dot(F,previous_covariane_matrix)
            current_covariance_matrix = np.dot(temp_matrix,F_transpose)
            #print("Current Covariance Matrix")
            #print(current_covariance_matrix)

            previous_state = current_state
            previous_covariane_matrix = current_covariance_matrix
            newx.append(float(current_state.item(0,0)))
            newy.append(float(current_state.item(0,1)))

            if zindagi_mein_dusri_bar:
                if new_value:
                    temp_matrix = inv(current_covariance_matrix + sensor_noise_covariance_matrix)
                    kalman_gain = np.dot(current_covariance_matrix,temp_matrix)
                    
                    new_state = current_state + np.dot(kalman_gain,(updated_state - current_state))
                    new_covariance_matrix = current_covariance_matrix - np.dot(kalman_gain,current_covariance_matrix)                    
                    
                    previous_covariane_matrix = new_covariance_matrix
                    previous_state = new_state
                    new_value = 0


        iteration = 0
        zindagi_mein_dusri_bar = 1
                
    
bag.close()

testimate = [0.0]
tacutal = [0.0]
for i in range(1,len(newx)):
    testimate.append(testimate[i-1] + 0.05)
for i in range(1,len(x)):
    tacutal.append(tacutal[i-1] + 0.33)
plt.plot(newx,newy,'bs',x, y, 'ro',)
#plt.axis([3, 3.8, 3.2, 5])
plt.show()