The software plans a trajectory for the end-effector of the youBot mobile manipulator (a mobile base with four mecanum wheels and a 5R robot arm), performs odometry as the chassis moves, and performs feedback control to drive the youBot to pick up a block at a specified location, carry it to the desired location, and put it down.
The final output is a CSV text file that specifies the configurations of the chassis and the arm, the angles of the four wheels, and the state of the gripper (open or closed) as a function of time. This specification of the position-controlled youBot can be "played" on the CoppeliaSim simulator to see if the trajectory succeeds in solving the task.

The code consist of the following scripts:

-------------Milestone1_Kinematics_Simulator.py-------------
contains the "NextState" function that takes an initial configuration of the youBot and simulates (using EulerStep) 
constant controls for the specified time step. I use the velocity limit for wheels and arm joints 10 radians per second. 
Also contains a test part corresponding to the one in the assignment description.

-------------Milestone2_Reference_Trajectory_Generation.py-------------
contains the "TrajectoryGenerator" function to generate the reference trajectory for the end-effector frame {e}. 
This trajectory consists of eight concatenated trajectory segments. I use the ScrewTrajectory function from MRlib and 
quintic time-scaling factor. The movement duration of each trajectory segment is calculated from the maximal linear or 
angular distance (what is bigger) between the start and end configuration. Maximal linear velocity is 10 meters per second and 
maximal angular velocity is 10 radians per second. For grasping and releasing I use 70 timesteps. 
Also contains a test part corresponding to the one in the assignment description.

-------------Milestone3_control.py-------------
contains the "FeedbackControl" to calculate the kinematic task-space feedforward plus feedback control law. 
I use tolerance 1e-3 for the Jacobian pseudoinverse to avoid singularities. Also, I use the "testJointLimits" function 
to avoid arm self-collision: using Scene 3 (Interactive youBot) I found approximate joint-angle combinations that avoid 
self-collision (those angles specified in the Kuka_youBot_parameters.py). First I check if the joint velocity violates 
the limit (10 radians per second), if no, I keep the velocity, if yes - I replace it with 10. 
Then I calculate the position of the arm joint with indicated velocity and check if it is outside the limit or not. 
If outside - I recalculate Jacobian, substituting column corresponding to the joint by zeros. 
Also contains a test part corresponding to the one in the assignment description.

-------------Kuka_youBot_parameters.py-------------
contains robot parameters such as kinematics, joint limits (position, velocity).

-------------Final_main.py-------------
Main script that employs all of the mentioned above to create 3 cases: best, overshoot, and newTask. 
For all of the three cases, I generate trajectories, loop through them calculating control commands, 
and using a kinematics simulator to produce a "next_states.csv" file for CoppeliaSim.

Parameters for three cases:
Robot configuration: 0.5 meter offset by Y axis and -0.7 rad in arm joint #3 which is 40 degrees along Y axis
Controller type: Feedworward + PI

            Kp      Ki      Cube_initial (x, y, theta)      Cube_final (x, y, theta)
Best:       1       0.01    1, 0, 0                         0, -1, -pi/2
Overshoot:  4.5     0.01    1, 0, 0                         0, -1, -pi/2
newTask:    1       0.01    0, -0.5, 0                      0, 1, pi/2
