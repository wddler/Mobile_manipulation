#Milestone 2

import math
import numpy as np
import modern_robotics as mr
import os
import csv
import Kuka_youBot_parameters
import Milestone3_control as control

#this function writes a row with reference configuration into a csv file
def write_files(row, path_trajectory):
    with open(path_trajectory, mode='a', newline='') as path_file:
        path_writer = csv.writer(path_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        path_writer.writerow(row)

#this function goes through the whole trajectory and creates rows that will be written into a csv file
def write_row(traj, grip, path_trajectory): #grip shoud be 0 or 1
    for i in traj:
        #r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state
        traj_row = [i[0][0], i[0][1], i[0][2],
                i[1][0], i[1][1], i[1][2],
                i[2][0], i[2][1], i[2][2], 
                i[0][3], i[1][3], i[2][3]]
        traj_row.append(grip) #add grippere state
        write_files(traj_row, path_trajectory)

#this function calculates time duration for trajectory segment. Based on linear and angular distance between the start and the finish transformation matrix
def calc_Tf(T_start, T_finish, k):
    '''
    Input:
    T_start - start transformation matrix
    T_finish - finish transformation matrix
    k - The number of trajectory reference configurations per 0.01 seconds

    Output:
    Tf - duration of the movement within a trajectory segment
    number_of_configurations - number of configurations within a trajectory segment
    '''
    max_linear_velocity = 0.1 #meters per 0.01 second
    max_angular_velocity = 0.1 #radians per 0.01 second
    T_distance = np.matmul(np.linalg.inv(T_start),T_finish) #matrix transform from start to finish
    linear_distance = math.hypot(T_distance[0,-1], T_distance[1,-1]) #shortest distance between start and finish points
    max_angular_distance = max(Rotation_matrix_to_euler_angles(T_distance[:-1,:-1])) #choose the biggest rotation
    max_motion_duration = max(linear_distance/max_linear_velocity, max_angular_distance/max_angular_velocity) #choose what is bigger: linear or angular motion
    number_of_configurations = int((max_motion_duration * k)/0.01) #calculate the number of configurations within a trajectory segment
    Tf = max_motion_duration
    return Tf, number_of_configurations


#this function map transformation matrix to Euler angles 
#based on "Computing Euler angles from a rotation matrix" method by Gregory G. Slabaugh https://www.gregslabaugh.net/publications/euler.pdf
def Rotation_matrix_to_euler_angles(Rotation_matrix):
    #as XYZ Euler angles notation is used in the publication, I named rotations as roll, pitch, yaw to make it more visually compelling
    def isclose(x, y, rtolerance=1.e-5, atolerance=1.e-8):
        return abs(x-y) <= atolerance + rtolerance * abs(y) 
    yaw = 0.0
    if isclose(Rotation_matrix[2,0],-1.0):
        pitch = math.pi/2.0
        roll = math.atan2(Rotation_matrix[0,1],Rotation_matrix[0,2])
    elif isclose(Rotation_matrix[2,0],1.0):
        pitch = -math.pi/2.0
        roll = math.atan2(-Rotation_matrix[0,1],-Rotation_matrix[0,2])
    else:
        pitch = -math.asin(Rotation_matrix[2,0])
        pitch_cos = math.cos(pitch)
        roll = math.atan2(Rotation_matrix[2,1]/pitch_cos, Rotation_matrix[2,2]/pitch_cos)
        yaw = math.atan2(Rotation_matrix[1,0]/pitch_cos, Rotation_matrix[0,0]/pitch_cos)
        return roll, pitch, yaw 

#this function generates trajectory for all eight segments
def TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k, path_trajectory):
    '''
    Input:
    Tse_initial - The initial configuration of the end-effector in the reference trajectory
    Tsc_initial - The cube's initial configuration
    Tsc_final - The cube's desired final configuration
    Tce_grasp - The end-effector's configuration relative to the cube when it is grasping the cube
    Tce_standoff - The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube
    k - The number of trajectory reference configurations per 0.01 second
    
    Output:
    csv file with trajectory references

    For all the segments I calculate Tf and N first, then employ "ScrewTrajectory" function from the MR library. 
    For segments 3 and 7 (grasping and releasing) I simply write 70 equal configurations to make end-effector grip/release the cube 
    '''
    method = 5 #The time-scaling method: 5 indicates quintic (5fth-order polynomial) time scaling.

    #create csv file
    traj_row = [Tse_initial[0][0], Tse_initial[0][1], Tse_initial[0][2],
                Tse_initial[1][0], Tse_initial[1][1], Tse_initial[1][2], 
                Tse_initial[2][0], Tse_initial[2][1], Tse_initial[2][2], 
                Tse_initial[0][3], Tse_initial[1][3], Tse_initial[2][3]]
    traj_row.append(0)
    with open(path_trajectory, mode='w', newline='') as path_file:
        path_writer = csv.writer(path_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        path_writer.writerow(traj_row)

    #first segment
    Tse_standoff = np.matmul(Tsc_initial, Tce_standoff)
    Tf, N = calc_Tf(Tse_initial, Tse_standoff, k)
    traj = mr.ScrewTrajectory(Tse_initial,Tse_standoff,Tf,N,method)
    write_row(traj, 0, path_trajectory) #add gripper state 0
    
    #second segment
    #Xstart = Tse_standoff
    Tse_grasp = np.matmul(Tsc_initial, Tce_grasp)
    Tf, N = calc_Tf(Tse_standoff, Tse_grasp, k)
    traj = mr.ScrewTrajectory(Tse_standoff,Tse_grasp,Tf,N,method)
    write_row(traj, 0, path_trajectory) #add gripper state 0

    #third segment
    i = Tse_grasp
    traj_row = [i[0][0], i[0][1], i[0][2],
                i[1][0], i[1][1], i[1][2],
                i[2][0], i[2][1], i[2][2], 
                i[0][3], i[1][3], i[2][3]]
    traj_row.append(1) #add gripper state 1
    for i in range(70):
        write_files(traj_row, path_trajectory)

    #fourth segment
    Tf, N = calc_Tf(Tse_grasp, Tse_standoff, k)
    traj = mr.ScrewTrajectory(Tse_grasp,Tse_standoff,Tf,N,method)
    write_row(traj, 1, path_trajectory) #add gripper state 1

    #fifth segment
    Tse_standoff_final = np.matmul(Tsc_final, Tce_standoff)
    Tf, N = calc_Tf(Tse_standoff, Tse_standoff_final, k)
    traj = mr.ScrewTrajectory(Tse_standoff,Tse_standoff_final,Tf,N,method)
    write_row(traj, 1, path_trajectory) #add gripper state 1

    #sixth segment
    Tse_grasp_final = np.matmul(Tsc_final, Tce_grasp)
    Tf, N = calc_Tf(Tse_standoff_final, Tse_grasp_final, k)
    traj = mr.ScrewTrajectory(Tse_standoff_final,Tse_grasp_final,Tf,N,method)
    write_row(traj, 1, path_trajectory) #add gripper state 1

    #seventh segment
    i = Tse_grasp_final
    traj_row = [i[0][0], i[0][1], i[0][2],
                i[1][0], i[1][1], i[1][2],
                i[2][0], i[2][1], i[2][2], 
                i[0][3], i[1][3], i[2][3]]
    traj_row.append(0) #add gripper state 0
    for i in range(70):
        write_files(traj_row, path_trajectory)

    #eighth segment
    Tf, N = calc_Tf(Tse_grasp_final, Tse_standoff_final, k)
    traj = mr.ScrewTrajectory(Tse_grasp_final,Tse_standoff_final,Tf,N,method)
    write_row(traj, 0, path_trajectory) #add gripper state 0


#If you want to test trajectory generator separately, you need to uncomment the code below
'''
#define paths to the file
cur_path = os.path.dirname('Milestone2_Reference_Trajectory_Generation.py')
path_trajectory = os.path.relpath('trajectory_test.csv', cur_path)

Tse_initial = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]]) #The initial configuration of the end-effector in the reference trajectory
Tsc_initial = np.array([[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]]) #The cube's initial configuration
Tsc_final = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]]) #The cube's desired final configuration
Tce_grasp = np.array([[-0.7071, 0 , 0.7071, 0],[0,1,0,0],[-0.7071, 0, -0.7071, 0],[0,0,0,1]]) #The end-effector's configuration relative to the cube when it is grasping the cube
Tce_standoff =np.array([[-0.7071, 0 , 0.7071, 0],[0,1,0,0],[-0.7071, 0, -0.7071, 0.06],[0,0,0,1]]) #The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube
k = 1 #The number of trajectory reference configurations per 0.01 seconds

print('Start calculating reference trajectory with the following parameters:')
print('Tse_initial:', Tse_initial, '\nTsc_initial:', Tsc_initial, '\nTsc_final:', Tsc_final, '\nTce_grasp:', Tce_grasp, '\nTce_standoff:', Tce_standoff)

TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k, path_trajectory)

print('Calculating is done, check the csv file')
'''