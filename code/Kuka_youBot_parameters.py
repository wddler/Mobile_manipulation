import numpy as np
import math

#This is the script for initializing Kuka youBot robot with its parameters.

class Robot:
    l = 0.47/2 #forward-backward distance between the wheels, meters
    w = 0.3/2 #side-to-side distance between wheels, meters
    r = 0.0475 #wheel radius, meters
    h = 0.0963 #height
    Tb0 = np.array([[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026], [0, 0, 0, 1]]) #The fixed offset from the chassis frame {b} to the base frame of the arm {0}
    #When the arm is at its home configuration (all joint angles zero, as shown in the figure), the end-effector frame {e} relative to the arm base frame {0} is:
    M0e = np.array([[1, 0, 0, 0.033], [0, 1, 0, 0], [0, 0, 1, 0.6546], [0, 0, 0, 1]])
    #When the arm is at its home configuration, the screw axes B for the five joints are expressed in the end-effector frame {e} as:
    Blist = np.array([[0,  0, 1,       0, 0.033, 0],
                      [0, -1, 0, -0.5076,     0, 0],
                      [0, -1, 0, -0.3526,     0, 0],
                      [0, -1, 0, -0.2176,     0, 0],
                      [0,  0, 1,       0,     0, 0]]).T
    joint_position_limits = [[-1,1],[-1.8,1],[-2,1.5],[-1.78,1.78],[-2.89,2.89]] # joint limits to avoid self-collisions: 1, 2, 3, 4, 5
    joint_velocity_limit = 10 #radians per second. The same for arm joints and for the wheels
    F = (r/4) * np.array([[-1.0/(l + w), 1.0/(l + w), 1.0/(l + w), -1.0/(l + w)],[1,1,1,1],[-1,1,-1,1]]) #formula 13.33 from the book
    F6 = np.array([[0,0,0,0],[0,0,0,0],F[0],F[1],F[2],[0,0,0,0]]) # for calculation body twist from wheel angles
    def calc_Tsb(phi, x, y): #ths function calculates transformation matrix Tsb from mobile base configuration
        Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, x], 
                        [np.sin(phi), np.cos(phi), 0, y], 
                        [0, 0, 1, 0.0963], 
                        [0, 0, 0, 1]])
        return Tsb
    
#initialization of the robot
KukaYoubot = Robot