#Milestone 1
import numpy as np
import math
import modern_robotics as mr
import os
import csv
import Kuka_youBot_parameters

def NextState(configuration_vector, controls_vector, dt, velocity_limit_arm, velocity_limit_wheel):
    '''
    Input:
    configuration_vector - A 12-vector representing the current configuration of the robot (3 variables for the chassis configuration, 5 variables for the arm configuration, and 4 variables for the wheel angles).
    controls_vector - A 9-vector of controls indicating the arm joint speeds theta_dot (5 variables) and the wheel speeds u (4 variables).
    dt - A timestep delta_t.
    max_ang_speed - A positive real value indicating the maximum angular speed of the arm joints and the wheels.
    
    Output:
    A 12-vector representing the configuration of the robot time delta_t later (chassis, arm, wheels)
    '''
    phi = configuration_vector[0] #mobile base orientation (angle)
    x = configuration_vector[1] #mobile base x coordinate
    y = configuration_vector[2] #mobile base y coordinate
    q_k = np.array([phi, x, y]) #configurations vector phi, x, y
    
    #call first-order Euler step
    new_joint_angles, delta_theta_list = euler_step(configuration_vector, controls_vector, dt, velocity_limit_arm, velocity_limit_wheel)

    #create a numpy vector of delta theta values
    delta_theta_list_np = np.zeros((4, 1))
    i = 0
    for d in delta_theta_list[5:]:
        np.put(delta_theta_list_np, i, d)
        i = i + 1 

    Vb = np.matmul(KukaYoubot.F,delta_theta_list_np) #calculate body twist Vb using the formula 13.33 from the book
    Vb6 = np.array([[0], [0], Vb[0], Vb[1], Vb[2], [0]]) #expand two dimentional twist Vb to three dimentional twist Vb6
    #extract the change in coordinates relative to the body frame b using the formula 13.35
    if Vb6[2][0] == 0:
        delta_qb = np.array([0, Vb6[3][0], Vb6[4][0]]) #w_bz, v_bx, v_by
    else:
        delta_qb = np.array([Vb6[2][0], 
                             (Vb6[3][0] * np.sin(Vb6[2][0]) + Vb6[4][0] * (np.cos(Vb6[2][0]) - 1))/Vb6[2][0], 
                             (Vb6[4][0] * np.sin(Vb6[2][0]) + Vb6[3][0] * (1 - np.cos(Vb6[2][0])))/Vb6[2][0]])
    matrix_for_delta_q = np.array([[1, 0, 0], [0, np.cos(phi), -np.sin(phi)], [0, np.sin(phi), np.cos(phi)]])
    #Transforming delta_qb in b frame to delta_q in the fixed frame s using the chassis angle phi_k using the formula 13.36
    delta_q = np.matmul(matrix_for_delta_q, delta_qb)
    #the updated odometry estimate of the chassis configuration
    q_k_plus_1 = np.add(q_k, delta_q)
    new_configuration = [q_k_plus_1[0], q_k_plus_1[1], q_k_plus_1[2]]
    new_configuration.extend(new_joint_angles)
    return new_configuration #nextstate returns x, y, theta, arm joint angles, wheel angles

#this function calculates new joint angles for a certain timestep
def euler_step(configuration_vector, controls_vector, dt, velocity_limit_arm, velocity_limit_wheel):
    joint_angles = configuration_vector[3:] #exclude phi, x and y
    delta_theta = 9 * [0]
    new_joint_angles = []
    delta_theta_list = []
    controls_vector_limited_speeds = []
    for speed in controls_vector[0:5]: #part for limiting arm joints speed
        if abs(speed) > velocity_limit_arm:
            if speed > 0:
                controls_vector_limited_speeds.append(velocity_limit_arm)
            else:
                controls_vector_limited_speeds.append(-velocity_limit_arm)
        else:
            controls_vector_limited_speeds.append(speed)
    for speed in controls_vector[5:]: #part for limiting wheels speed
        if abs(speed) > velocity_limit_wheel:
            if speed > 0:
                controls_vector_limited_speeds.append(velocity_limit_wheel)
            else:
                controls_vector_limited_speeds.append(-velocity_limit_wheel)
        else:
            controls_vector_limited_speeds.append(speed)
    
    i = 0
    #calculate new angles for joints
    for joint_angle in joint_angles[0:5]:
        new_arm_joint_angle = joint_angle + controls_vector_limited_speeds[i] * dt
        new_joint_angles.append(new_arm_joint_angle)
        i = i + 1
    for wheel_angle in joint_angles[5:]:
        new_wheel_angle = wheel_angle + controls_vector_limited_speeds[i] * dt
        new_joint_angles.append(new_wheel_angle)
        i = i + 1
    index = 0
    for delta in joint_angles:
        delta_theta = new_joint_angles[index] - joint_angles[index]
        delta_theta_list.append(delta_theta)
        index = index + 1

    return new_joint_angles, delta_theta_list

#function to write a csv file
def write_next_state(row, path_next_states):
    with open(path_next_states, mode='a', newline='') as path_file:
        path_writer = csv.writer(path_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        path_writer.writerow(row)

KukaYoubot = Kuka_youBot_parameters.Robot #initialization of robot parameters

#If you want to test kinematics simulator separately, you need to uncomment the code below

'''
#-------------------code for testing the kinematics simulator----------------
#define paths to the test file to write
cur_path = os.path.dirname('Milestone1_Kinematics_Simulator.py')
path_next_states = os.path.relpath('next_states_test.csv', cur_path)

configuration_vector = (0,0,0,0,0,0,0,0,0,0,0,0)#phi, x, y, theta1, theta2, theta3, theta4, theta5, wheel_1, wheel_2, wheel_3, wheel_4
controls_vector = [0,0,0,0,0,10,10,10,10]
dt = 0.01

velocity_limit_arm = KukaYoubot.joint_velocity_limit # joints speed limit (radians per second)
velocity_limit_wheel = KukaYoubot.joint_velocity_limit # wheels speed limit (radians per second)

#create the csv file 
with open(path_next_states, mode='w', newline='') as path_file:
    path_writer = csv.writer(path_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    path_writer.writerow(configuration_vector)

print('Start calculating next states. \nConfiguration vector is:', configuration_vector, '\nControl vector is:', controls_vector, '\nVelocity limit is:', KukaYoubot.joint_velocity_limit, '\ndt is:', dt)
for i in range(100): #simulating the control input for 1 second (100/0.01)
    configuration_vector = NextState(configuration_vector, controls_vector, dt, velocity_limit_arm, velocity_limit_wheel)
    configuration_vector = configuration_vector[:12]
    write_next_state(configuration_vector, path_next_states)
print('Calculation is done, check the csv file')
print('Final configuration vector is:\n', configuration_vector)
'''