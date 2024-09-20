# Stweart Platform Simulation with Trapezoidal Velocity Trajectory Planner and Local Singularity Avoidance
This Colab notebook simulates a Stewart Platform with a Trapezoidal Velocity Trajectory Planner and Local Singularity Avoidance, using four key classes:

* The **Robot** class models the Stewart Platform, handling forward/inverse kinematics, actuator forces, and singularity detection.
* The **Trapezoidal Velocity** class manages trajectory planning, generating desired joint trajectories.
* The **Controller** class handles control logic, issuing joint commands and tracking desired trajectories.
* The **Simulation** class ties it all together, running dynamic simulations, visualizing real-time behavior, and analyzing performance.
  
## Table of Contents

- [Libraries](#libraries)
- [Classes Description](#classes-description)
  - [Stewart Platform Class](#stewart-platform-class)
  - [Trapezoidal Velocity Class](#trapezoidal-velocity-class)
  - [Controller Class](#controller-class)
  - [Simulation Class](#simulation-class)

- [Code Execution](#code-execution)
  - [Variables Definition](#variables-definition)
  - [Initialization](#initialization)
  - [Simulation](#simulation)
  - [Video Generation](#video-generation)

## Libraries

Needed libraries:

```bash
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from IPython.display import HTML
```
## Classes Description

A short description of the key features of the four classes used in this repository is made here below.

### Stewart Platform Class

This Stewart Platform Class has multiple uses that we will not fully cover here. The main function of the class that will be used here are the *Inverse Kinematics*, *Forward Kinematics* , *Load Singularities*  and *Find Closest singularity*. Inverse and Forward kinematics are used to move from joint space to task space and vice versa when needed. The main focus of this class will be around *Load Singularities* and *Find Closest singularity* functions. The first one is used to upload the singularity positions in task space found offline, the second one is used to find the closest singularity given a pose.

Key functions description:

In the  *__find_closest_singularity_*  function:

1.   Using the workspace boundaries, the pose of the robot and the singularities get normalized.
2.   The euclidean distance between the pose and all the singularities is calculated
3.   The minimum distance is found and the closest singularity is selected.

### Trapezoidal Velocity Class
The Trapezoidal class is designed to generate a trapezoidal velocity profile for controlling the movement of a robotic system. This class functions as a path and trajectory planner which is generated in joint space and then converted to task space for the controller. Note : for each second of simulation 5 frames are needed, the correction factor is used to convert seconds into ticks and vice versa.

Key functions description:

In the _ _init_ _  function:
1.   A copy of the robot is initialized to use IK and FK
2.   Initial and final pose are saved
3.   The time vector is defined
4.   The function _calculate_q_profiles_ is called
5.   The function calculate_q_vel_profiles is called
6.   The function calculate_x_profiles is called


In the  _calculate_q_profiles_  function:

1.   For every every joint q :
  *   Limits of the trapezoidal trajectory are checked in the _check_limits_ function. In case the minimum  or maximum velocity constraints are not met, new values of velocities are proposed.
  *  For every t in the time vector: the time profile relative to the joint q is built.

In the  _calculate_q_vel_profiles_  function:

1.   For every every joint q :
  *  the velocity profile relative to the joint q is built.

In the  _calculate_x_profiles_  function:

1.  Using FK of the robot and the time profiles from _calculate_q_profiles_, time profiles in task space are calculated.
   
### Controller Class

This controller stands in the middle between the trapezoidal velocity trajectory generation and the simulation environment. The objective is to maintain the robot onto the trapezoidal path while locally avoiding singularities if needed. Note : The controls are generated in task space and then converted into joint space for the motors.

Key functions description:

In the _ _init_ _  function:
1.   The robot class is passed.
2.   The trapezoidal trajectory is initialized and generated.
3.   The time vector, initial and final position are found.
4.   Max joint velocity is defined.
5.   Standard deviation for gaussian obstacle profile generation is defined.(to be tuned)

In the  _calculate_cmd_   function:

1.   For every t (instant) in the time vector :
  *   The trajectory part of the control action is found (u=K*error). The error is calculated as the difference of the reference coming from the trapezoidal trajectory and the position of the robot in time.
  *   If the LocalConditionIndex is too low, the singularity part of the control action is found (u=K*closeness to singularity * direction). The closest singularity is found with the robot function _getClosestSingularity_. The closeness to the singularity (gaussian distance) to each component is found with the function _findGaussianDistFromSingularity_. K needs to be tuned.
  *   Velocity command is calculated and is checked for saturation.


  In the  _findGaussianDistFromSingularity_   function:

1.   The pose of the robot and the closest singularity get normalized.
2.   For each pose component (x,y,z,roll,pitch,yaw), the gaussian distance from the singularity is found.
3. The whole gaussian vector is then normalized.

### Simulation Class

The simulation class is used to simulate the robot side of the system. Commands arrive from the controller at every timestep, the position of the robot is updated taking into consideration random noise, feedback of the new configuration is provided to the controller. Using the library *FuncAnimation* it is possible to generate a video at 5 frames per second showing : the robot, the robot closest singularity, the desired trajectory, the real trajectory and the table of the forces experienced by the legs under the effect of gravity.

Key functions description:

In the _ _init_ _  function:
1.   The controller and the robot are passed to the simulation class.
2.   The time vector, initial and final position are retrieved.
3.   The force to win gravity is defined (x direction).

In the  _start_   function

1.   For every t (instant) in the time vector :
  *   A new velocity command for the linear actuators is give by the controller.
  *   The simulation gets updated through the _updateSimulation_ function.
  *   Feedback is given to the controller through the controller function _updatePose_.
2.  Multiple functions for plotting are then called.

In the  _updateSimulation_   function

1. The theoretical displacement of the actuator is found.
2. Random noise is added to the new displacement of the actuator to simulate real case scenario.
3. New lenghts of actuators are found.
4. Robot pose is updated.



## Code Execution

In this section, the steps necessary to use the various elements is shown

### Variables Definition
1.   Mount the google drive.
2.   Define the Platform parameters.
3.   Upload the singularities (calculated offline with the *__find_singularity_workspace* stewart platform function).

   
```bash
# access drive to load singularities
drive.mount('/content/drive')

# Platform definition
r_b = 0.5  # Radius of base
phi_b = 50  # Angle between base joints
r_p = 0.3  # Radius of platform
phi_p = 80  # Angle between platform joints

# Workspace Boundaries definition
workspace_limits = [-0.5, 0.5, -0.5, 0.5, 0.1, 0.6]
orientation_limits = [-10, 10, -10, 10, -10, 10]

# load singularities into google colab
with open('/content/drive/My Drive/Github/filtered_singularities_task_space_2.txt', 'r') as f:
    singularities_task_space = np.loadtxt(f)

# singularities_task_space = np.array([[0,0,0,0,0,0],[0,0,0,0,0,0]]) # uncomment if no singularities are available
```
### Initialization

1.   Initialize robot class.
2.   Load the singularities into the robot object.
3.   Define the controller's parameters and  initialize the controller. By initializing the controller, the trapezoidal velocity profile class is automatically constructed. Warning will pop up if the chosen set of speed and time does not meet the trapezoidal profile constraints.
4.   Initialize the simulation.
   
```bash
# Platform initialization
platform = StewartPlatform(r_b , phi_b , r_p, phi_p)
# Loading Singularities into robot.
platform.loadSingularitiesTaskSpace(singularities_task_space,workspace_limits,orientation_limits) # be coherent with the limits used for getSingularityWorkspace() function

# Define initial and final position
pose_i=np.array([0.3,0.1,0.5,0,-10,0]) # Note: FK algorithm does not like high values for angles.
pose_f=singularities_task_space[230]

# Define final time
tf_seconds=20 # defines the time in which you would like to reach the final pose

# Define controller parameters
max_joint_vel=0.03 # [m/s] # define max joint velocities
K=np.array([1,1,1,0.03,0.03,0.03])*0.5 # define proportional gain for the controller

# Initialize controller
controller = Controller(platform,pose_i,pose_f,tf_seconds,max_joint_vel,K)

# Initialize Simulation
sim = Simulation(controller)
```
### Simulation
Now that the robot, the controller with trapezoidal trajectory and the simulation are setted up.

Start the simulation

If, during the trajectory, the robot passes too close to a singularity, warnings will pop up.

A series of plots is then generated to show the simulation's result.

```bash
# Start Simulation
sim.start()
```
### Video Generation
Generate the video from the simulation.

 Note: this may take some time as the animation is generated with matplotlib. In each second, five frames are generated.

Elements:


*   In blue the Stewart Platform.
*   In black the closest singular configuration.
*   The static series of frames is the trajectory coming from the trapezoidal class.
*   The dynamic series of frames is the real trajectory of the robot.
*   In the top right corner is the table of the forces felt by the actuators under gravity (enumaration at the base of the platform).
  
```bash
 # Generate video
HTML(sim.anim.to_html5_video())
```

