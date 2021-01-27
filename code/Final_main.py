#main program with 3 cases.
import os
import csv
import numpy as np
import Kuka_youBot_parameters
import Milestone1_Kinematics_Simulator as ks
import Milestone2_Reference_Trajectory_Generation as tr
import Milestone3_control as control

'''
This script is organazied as follows:
First, I define functions: to read reference configurations from the trajectory file,
to create csv files with actual configurations and twist errors,
to employ the controller from Milestone 3 and kinematics simulator from Milestone 1,
Second, I initialize common parameters for all of the three cases: robot's parameters, Tce_grasp and Tce_standoff, timesteps for trajectory and control
Third, for all of the three cases (best, overshoot, newTask) I generate trajectories, loop through them calculating control commands and using kinematics simulator to produce "next_states.csv" file for CoppeliaSim

Parameters for three cases:
Robot configuration: 0.5 meter offset by Y axis and -0.7 rad in arm joint #3 which is 40 degrees along Y axis
Controller type: Feedworward + PI

            Kp      Ki      Cube_initial (x, y, theta)      Cube_final (x, y, theta)
Best:       2       0.01    1, 0, 0                         0, -1, -pi/2
Overshoot:  4.5     0.01    1, 0, 0                         0, -1, -pi/2
newTask:    2       0.01    0, -0.5, 0                      0, 1, pi/2
'''

#this function reads reference configuration from a trajectory file and makes a list for more convenient usage
def read_reference_configurations():
    reference_configurations = []
    with open(path_trajectory, newline='') as traj:
        config_reader = csv.reader(traj)
        for config in config_reader:
            reference_configurations.append([float(config[0]), float(config[1]), float(config[2]), 
                                             float(config[3]), float(config[4]), float(config[5]), 
                                             float(config[6]), float(config[7]), float(config[8]), 
                                             float(config[9]), float(config[10]), float(config[11]), float(config[12])])
    print('Trajectory contains', len(reference_configurations), 'configurations')
    return reference_configurations

#this function creates a file with robot configurations made by kinematics simulator
def create_file_with_next_states():
    with open(path_next_states, mode='w', newline='') as path_file:
        path_writer = csv.writer(path_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        row_to_write = configuration
        row_to_write.append(0)
        path_writer.writerow(row_to_write)
#this function creates a file with twist errors
def create_file_with_errors():
    with open(error_csv, mode='w', newline='') as error_file:
        error_writer = csv.writer(error_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        error_writer.writerow('')
#this function employ the controller from Milestone 3 and kinematics simulator from Milestone 1
def calculate_control_and_simulate(reference_configurations, configuration):
    i = 0
    for reference in reference_configurations: #for every reference configuration in the trajectory
        if i+1 == len(reference_configurations):
            break
        Xd = np.array([[reference[0], reference[1], reference[2], reference[9]], #create Tse
                       [reference[3], reference[4], reference[5], reference[10]],
                       [reference[6], reference[7], reference[8], reference[11]],
                       [0,0,0,1]])
        #create Tse_next
        Xd_next = np.array([[reference_configurations[i+1][0], reference_configurations[i+1][1], reference_configurations[i+1][2], reference_configurations[i+1][9]], 
                            [reference_configurations[i+1][3], reference_configurations[i+1][4], reference_configurations[i+1][5], reference_configurations[i+1][10]], 
                            [reference_configurations[i+1][6], reference_configurations[i+1][7], reference_configurations[i+1][8], reference_configurations[i+1][11]], 
                            [0,0,0,1]]) 
        V, u = controller.FeedbackControl(configuration[:8], Xd, Xd_next, error_csv) #calculate twist and commanded velocities
        controls_vector = [u[4], u[5], u[6], u[7], u[8], u[0], u[1], u[2], u[3]] #make control vector
        #calculate configuration based on commanded velocities using kinematics simulator
        configuration = ks.NextState(configuration[:12], controls_vector, dt, KukaYoubot.joint_velocity_limit, KukaYoubot.joint_velocity_limit)
        #write configuration to the file
        i = i+1
        row_to_write = configuration
        #append gripper state
        if reference[12] == 0:
            row_to_write.append(0) 
        else:
            row_to_write.append(1)
        ks.write_next_state(row_to_write, path_next_states)
    print('Calculation of controls and next states is done. Files with configurations and twist errors are created')

#common parameters for all 3 cases
KukaYoubot = Kuka_youBot_parameters.Robot #initialization of robot parameters
Tce_grasp = np.array([[-0.7071, 0 , 0.7071, 0.01],[0, 1, 0, 0],[-0.7071, 0, -0.7071, 0],[0, 0, 0, 1]]) #The end-effector's configuration relative to the cube when it is grasping the cube
Tce_standoff =np.array([[-0.7071, 0 , 0.7071, 0.01],[0,1,0,0],[-0.7071, 0, -0.7071, 0.06],[0,0,0,1]]) #The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube
k = 1 #The number of trajectory reference configurations per 0.01 seconds
dt = 0.01 #for trajectrory
timestep = 0.01 #for controller
cur_path = os.path.dirname('Final_main.py')
configuration = [0,0,0.5,0,0,-0.7,0,0,0,0,0,0] #0.5 meter offset by Y axis and -0.7 rad in arm joint #3 which is 40 degrees along Y axis

#----------------------------Best case-------------------------------------------------
print('----------------------Best case----------------------')
#calculate reference trajectory
Tse_initial = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]]) #The initial configuration of the end-effector in the reference trajectory
Tsc_initial = np.array([[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]]) #The cube's initial configuration
Tsc_final = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]]) #The cube's final desired configuration
print('Calculating reference trajectory with the following parameters:')
print('Initial cube configuration:','x, y, theta', Tsc_initial[0][3], Tsc_initial[1][3], '0', '\nFinal cube configuration:','x, y, theta', Tsc_final[0][3], Tsc_final[1][3], '-pi/2', )
print('Initial robot configuration is: theta, x, y, arm joints 1-5, wheels 1-4', configuration)
path_trajectory = os.path.relpath('..\\results\\best\\trajectory.csv', cur_path) 
tr.TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k, path_trajectory) #generate trajectory
print('Reference trajectory is calculated')

#loop through the reference configuration, calculate required controls and employ kinematics simulator
reference_configurations = read_reference_configurations()
path_next_states = os.path.relpath('..\\results\\best\\next_states.csv', cur_path)
error_csv = os.path.relpath('..\\results\\best\\error.csv', cur_path)
create_file_with_next_states()
create_file_with_errors()
Kp = 0.5
Ki = 0.01
controller = control.FeedbackController(Kp, Ki, timestep)
configuration = [0,0,0.5,0,0,-0.7,0,0,0,0,0,0] #0.5 meter offset by Y axis and -0.7 rad in arm joint #3 which is 40 degrees along Y axis
calculate_control_and_simulate(reference_configurations, configuration)

#----------------------------Overshoot case-------------------------------------------------
print('----------------------Overshoot case----------------------')
#calculate reference trajectory
Tse_initial = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]]) #The initial configuration of the end-effector in the reference trajectory
Tsc_initial = np.array([[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]]) #The cube's initial configuration
Tsc_final = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]]) #The cube's final desired configuration
print('Calculating reference trajectory with the following parameters:')
print('Initial cube configuration:','x, y, theta', Tsc_initial[0][3], Tsc_initial[1][3], '0', '\nFinal cube configuration:','x, y, theta', Tsc_final[0][3], Tsc_final[1][3], '-pi/2', )
print('Initial robot configuration is: theta, x, y, arm joints 1-5, wheels 1-4', configuration)
path_trajectory = os.path.relpath('..\\results\\overshoot\\trajectory.csv', cur_path)
tr.TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k, path_trajectory) #generate trajectory
print('Reference trajectory is calculated')

#loop through the reference configuration, calculate required controls and employ kinematics simulator
reference_configurations = read_reference_configurations()
path_next_states = os.path.relpath('..\\results\\overshoot\\next_states.csv', cur_path)
error_csv = os.path.relpath('..\\results\\overshoot\\error.csv', cur_path)
create_file_with_next_states()
create_file_with_errors()
Kp = 4.5
Ki = 0.01
controller = control.FeedbackController(Kp, Ki, timestep)
#configuration = [1,0,0.5,0,0,-0.7,0,0,0,0,0,0]
configuration = [0,0,0.5,0,0,-0.7,0,0,0,0,0,0] #0.5 meter offset by Y axis and -0.7 rad in arm joint #3 which is 40 degrees along Y axis
calculate_control_and_simulate(reference_configurations, configuration)

#----------------------------newTask case-------------------------------------------------
print('----------------------newTask case----------------------')
#calculate reference trajectory
Tse_initial = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]]) #The initial configuration of the end-effector in the reference trajectory
Tsc_initial = np.array([[0,1,0,0],[-1,0,0,-0.5],[0,0,1,0.025],[0,0,0,1]]) #The cube's initial configuration
Tsc_final = np.array([[1, 0, 0, 0], [0, 1, 0, 1], [0, 0, 1, 0.025], [0, 0, 0, 1]]) #The cube's final desired configuration
print('Calculating reference trajectory with the following parameters:')
print('Initial cube configuration:','x, y, theta', Tsc_initial[0][3], Tsc_initial[1][3], '0', '\nFinal cube configuration:','x, y, theta', Tsc_final[0][3], Tsc_final[1][3], 'pi/2', )
print('Initial robot configuration is: theta, x, y, arm joints 1-5, wheels 1-4', configuration)
path_trajectory = os.path.relpath('..\\results\\newTask\\trajectory.csv', cur_path)
tr.TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k, path_trajectory) #generate trajectory
print('Reference trajectory is calculated')

#loop through the reference configuration, calculate required controls and employ kinematics simulator
reference_configurations = read_reference_configurations()
path_next_states = os.path.relpath('..\\results\\newTask\\next_states.csv', cur_path)
error_csv = os.path.relpath('..\\results\\newTask\\error.csv', cur_path)
create_file_with_next_states()
create_file_with_errors()
Kp = 2
Ki = 0.01
controller = control.FeedbackController(Kp, Ki, timestep)
#configuration = [1,0,0.5,0,0,-0.7,0,0,0,0,0,0]
configuration = [0,0,0.5,0,0,-0.7,0,0,0,0,0,0] #0.5 meter offset by Y axis and -0.7 rad in arm joint #3 which is 40 degrees along Y axis
calculate_control_and_simulate(reference_configurations, configuration)