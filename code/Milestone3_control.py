#Milestone 3 feedforward control
import numpy as np
import modern_robotics as mr
import math
from math import cos, sin, fabs, copysign
import os
import csv
import Kuka_youBot_parameters 

class FeedbackController:
    #init function keeps controller's parameters
    def __init__(self, Kp, Ki, timestep):
        self.Kp = np.eye(6) * Kp #identity matrix * proportional gain
        self.Ki = np.eye(6) * Ki #identity matrix * integral gain
        self.integral = np.zeros(6) #cumulative twist error
        self.timestep = timestep
        
    def FeedbackControl(self, configuration, Xd, Xd_next, error_csv):
        '''Input:
        #configuration - phi, x, y, theta1, theta2, theta3, theta4, theta5
        Xd - The end-effector reference configuration at the next timestep in the reference trajectory (Tse_des)
        Xd_next - The end-effector reference configuration at a time delta_t later (Tse_des_next)
        error_csv - path to the csv file

        Output:
        V - The commanded end-effector twist V expressed in the end-effector frame {e}.
        velocities - commanded velocities (wheels 1-4, arm_joints 1-5)
        
        Joint limits are implemented to avoid self-collisions and tolerance in Jacobian to avoid singularities  
        '''
        
        thetalist = np.array(configuration[3:8]) #find thetalist from a configuration vector
        phi = configuration[0]
        x = configuration[1]
        y = configuration[2]

        Tsb = KukaYoubot.calc_Tsb(phi, x, y) # find Tsb (chassis base in the space frame)
        T0e = mr.FKinBody(KukaYoubot.M0e, KukaYoubot.Blist, thetalist) # find T0e (end-effector in the arm base frame)
        Tse_current = np.dot(np.dot(Tsb,KukaYoubot.Tb0),T0e) # current end-effector position in the space frame
        Tse_current_desired = np.dot(mr.TransInv(Tse_current),Xd) # transformation from the current Tse to Tse_des
        Tse_desired_next_desired = np.dot(mr.TransInv(Xd), Xd_next) # transformation from the desired position Tse_des to the next desired position Tse_des_next

        Vd_matrix_form = mr.MatrixLog6(Tse_desired_next_desired) / self.timestep # The feedforward reference twist in adjoint matrix form
        Vd = mr.se3ToVec(Vd_matrix_form) # The feedforward reference twist as 6 elements vector

        Xerr_matrix_form = mr.MatrixLog6(Tse_current_desired) # error twist in the 4x4 matrix form
        Xerr = mr.se3ToVec(Xerr_matrix_form) #error twist as 6 elements vector
        #here we save this error twist to the csv file
        with open(error_csv, mode='a', newline='') as error_file: 
            error_writer = csv.writer(error_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            error_writer.writerow(Xerr)
        
        feedforward = np.dot(mr.Adjoint(Tse_current_desired), Vd) # feedforward component calculated using formula 13.17 from the book
        P = np.dot(self.Kp, Xerr) #proportional component calculated using formula 13.17 from the book
        self.integral = self.integral + Xerr #calculation of the cumulative integral error
        I = np.dot(self.Ki,self.integral*self.timestep) #integral component calculated using formula 13.17 from the book

        V = feedforward + P + I # The commanded end-effector twist V expressed in the end-effector frame {e}.
        
        Teb = np.dot(mr.TransInv(T0e), mr.TransInv(KukaYoubot.Tb0)) # chassis base in the end-effector frame
        
        Jbase = np.dot(mr.Adjoint(Teb),KukaYoubot.F6) #Jbase expresses the contribution of the wheel velocities u to the end-effector's velocity. Calculated using the formula from the page 552 in the book
        Jarm = mr.JacobianBody(KukaYoubot.Blist, thetalist) #Body Jacobian of the arm expresses the contribution of the joint velocities to the end-effector's velocity
        Je = np.hstack((Jbase, Jarm)) #resulting Jacobian
        
        #Implementing joint limits to avoid self-collisions and singularities 
        while(True):
            velocities = np.dot(np.linalg.pinv(Je,1e-3),V) # The tolerance option allows you to specify how close to zero a singular value must be to be treated as zero. By treating small singular values (that are greater than the default tolerance) as zero, you will avoid having pseudoinverse matrices with unreasonably large entries
            joints_with_constraints = self.FindJointLimits(velocities[4:], thetalist)
            if(len(joints_with_constraints)): #if a list with constrained joints is contains elements:
                for joint_constraint in joints_with_constraints: 
                    Je[:,Jbase.shape[1] + joint_constraint] = 0.0 #change corresponding column in the Jacobian to zeros
            else:
                break
            
        return V, velocities
    
    def find_X(self, configuration): #X is equal to Tse
        #input - configuration (phi, x, y, theta1, theta2, theta3, theta4, theta5)
        phi = configuration[0]
        x = configuration[1]
        y = configuration[2]
        Tsb = KukaYoubot.calc_Tsb(phi, x, y)
        thetalist = np.array([configuration[3], configuration[4], configuration[5], configuration[6], configuration[7]])
        T0e = mr.FKinBody(KukaYoubot.M0e,KukaYoubot.Blist,thetalist)
        Ts0 = np.matmul(Tsb, KukaYoubot.Tb0)
        Tse = np.round(np.matmul(Ts0, T0e),5)
        return Tse
    
    #this function check if arm joints violate limits for velocity and position that are defined in the Robot class to avoid self-collision
    def FindJointLimits(self, joint_velocities, current_joint_positions):
        '''
        Input:
        joint_velocities - vector of 5 elements
        current_joint_positions - thetalist

        Output:
        joints_with_constraints - a list with indication of what joints violate limits
        '''
        joint_positions = len(joint_velocities) * [0] # list with zeros
        joints_with_constraints = [] #empty list
        for joint_number in range(len(joint_velocities)):
            if math.fabs(joint_velocities[joint_number]) > math.fabs(KukaYoubot.joint_velocity_limit): #if joint velocity is bigger than the limit:
                joint_velocities[joint_number] = math.copysign(KukaYoubot.joint_velocity_limit,joint_velocities[joint_number]) #change current joint velocity to the maximal (limited)
            joint_positions[joint_number] = current_joint_positions[joint_number] + (joint_velocities[joint_number] * self.timestep) #calculate new joint position
            if(joint_positions[joint_number] > KukaYoubot.joint_position_limits[joint_number][1] or joint_positions[joint_number] < KukaYoubot.joint_position_limits[joint_number][0]): #if joint position is bigger than the limit
                joints_with_constraints.append(joint_number) #add this joint to the list which will be returned to recalculate Jacobian
        return joints_with_constraints
KukaYoubot = Kuka_youBot_parameters.Robot #initialization of robot parameters

#If you want to test the controller separately, you need to uncomment the code below
'''
configuration = [0,0,0,0,0,0.2,-1.6,0] #current configuration (phi, x, y, theta1, theta2, theta3, theta4, theta5)
Xd = np.array([[0,0,1,0.5],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]]) #current position (Tse)
Xd_next = np.array([[0,0,1,0.6],[0,1,0,0],[-1,0,0,0.3],[0,0,0,1]])
Kp = 0
Ki = 0
timestep = 0.01

controller = FeedbackController(Kp, Ki, timestep)
cur_path = os.path.dirname('Milestone3_control.py')
error_csv = os.path.relpath('error_test.csv', cur_path)
with open(error_csv, mode='w', newline='') as error_file:
    error_writer = csv.writer(error_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    error_writer.writerow('')

print('Start calculation of the body twist and commanded velocity for the following parameters:')
print('Kp =', Kp, '\nKi =', Ki, '\nTimestep =', timestep)
print('Current configuration:', configuration, '\nXd:', Xd, '\nXd_next:', Xd_next)

res = controller.FeedbackControl(configuration, Xd, Xd_next, error_csv)

print('Calculation is done')
print('Body Twist is:', res[0], '\nCommanded velocities are:', res[1])
'''