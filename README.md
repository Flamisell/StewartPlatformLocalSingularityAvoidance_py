# Stweart Platform Simulation with Trapezoidal Velocity Trajectory Planner and Local Singularity Avoidance
This Colab notebook simulates a Stewart Platform with a Trapezoidal Velocity Trajectory Planner and Local Singularity Avoidance, using four key classes:

* The **Robot** class models the Stewart Platform, handling forward/inverse kinematics, actuator forces, and singularity detection.
* The **Trapezoidal Velocity** class manages trajectory planning, generating desired joint trajectories.
* The **Controller** class handles control logic, issuing joint commands and tracking desired trajectories.
* The **Simulation** class ties it all together, running dynamic simulations, visualizing real-time behavior, and analyzing performance.
* 
## Table of Contents

- [Libraries](#libraries)
- [Classes Description](#classes-description)
  - [Stewart Platform Class](#stewart-platform-class)
  - [Trapezoidal Velocity Class](#trapezoidal-velocity-class)
  - [Controller Class](#controller-class)
  - [Simulation Class](#simulation-class)

- [Usage](#usage)
  - [Singularity Search](#singularity-search)
    - [Platform Initialization](#platform-initialization)
    - [Find Singularities](find-singularities)
    - [Filter Singularities](filter-singularities)
  - [Closest Singularity Search and Visualization](#closest-singularity-search-and-visualization)
- [Class Methods Overview](#class-methods-overview)

## Libraries

Needed libraries:

```bash
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
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



## Usage

### Singularity Search
The steps to obtain an offline set of singular configurations is presented here below.

#### Platform Initialization
This class has multiple purposes which will not be discussed here but can be found in my Stewart Platform Class repository, the key function that is used in this notebook is the *getSingularityWorkspace* function.
By defining the workspace boundaries (in terms of position and orientation), the local condition index of the transposed Jacobian matrix is calculated for each pose telling us when Jabobian becomes singular.

To create an instance of the StewartPlatform class, you need to provide the following parameters:
- r_b: Radius of the base.
- phi_b: Angle between base joints.
- r_p: Radius of the platform.
- phi_p: Angle between platform joints.

It is also important to define limits on workspace position and orientation, this limits will define the boundaries of the search.
```
# Define parameters
r_b = 0.5  # Radius of base
phi_b = 50  # Angle between base joints
r_p = 0.3  # Radius of platform
phi_p = 80  # Angle between platform joints

# Create Stewart Platform instance
platform = StewartPlatform(r_b, phi_b, r_p, phi_p) # Initialize platform
pose=np.array([0,0,0.5,10,0,0]) # Define pose
lengths= platform.getIK(pose) # Find Joint position
k=platform.getLocalConditionIndexT() # Tells you how close you are to a singularity

# Define workspace and orientation limints
workspace_limits = [-0.5, 0.5, -0.5, 0.5, 0.1, 0.6]
orientation_limits = [-10, 10, -10, 10, -10, 10]
x_min, x_max, y_min, y_max, z_min, z_max = workspace_limits
roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max = orientation_limits
```
Mount the drive
```
from google.colab import drive
drive.mount('/content/drive')
```
#### Find Singularities
Use *getSingularityWorkspace* to search for any pose which has a low local condition index. Note: this can be computationally expensive.
```
N_pos=10 # Discretization for workspace coordinates
N_orient=10 # Discretization for orientation coordinates
Holder=platform.getSingularityWorkspace(workspace_limits,orientation_limits,N_pos,N_orient) # find singularities in all space
```
Save the singularities onto drive for safety.
```
 with open('/content/drive/My Drive/Github/singularities_2.txt', 'w') as f:
     np.savetxt(f, Holder)
```
Load the whole singularity file.
```
with open('/content/drive/My Drive/Github/singularities_2.txt', 'r') as f:
    singularities = np.loadtxt(f)
```
#### Filter Singularities
*First Filtering*

Slicing the matrix, removing the borders of the workspace (external singularities) and selecting only the poses with low local condition index.
```
keep_mask = (singularities[:, 6] < 0.001) & (singularities[:, 2] > z_min)& (singularities[:, 2] < z_max) & (singularities[:, 0] < x_max)& (singularities[:, 0] > x_min)& (singularities[:, 1] < y_max) & (singularities[:, 1] > y_min)
interest_points_with_index=singularities[keep_mask]
```
*Second filtering*

We normalize the singularities between the borders of the workspace, we then calculate the distance matrix between all of them and remove the configurations which are too close to each other.

```
from scipy.spatial.distance import cdist

# take only poses
interest_points=interest_points_with_index[:,:6]
#normalize the singularities poses
mins=np.array([x_min,y_min,z_min,roll_min,pitch_min,yaw_min])
maxs=np.array([x_max,y_max,z_max,roll_max,pitch_max,yaw_max])
normalized_interest_points = (interest_points - mins) / (maxs - mins)

threshold=0.3
distances = cdist(normalized_interest_points, normalized_interest_points)
# To avoid self-comparison, set the diagonal to a large value
np.fill_diagonal(distances, np.inf)

index_holder=[]
interest_points_holder=np.copy(interest_points_with_index)
keep_mask = np.ones(normalized_interest_points.shape[0], dtype=bool)
for i in range(normalized_interest_points.shape[0]):
    interest_points_holder[i][6]=i
    if keep_mask[i]:
        # Find indices of vectors within the threshold distance
        close_indices = np.where(distances[i] < threshold)[0]
        # Set keep_mask to False for these indices
        keep_mask[close_indices] = False
        # Ensure we don't set the current vector's mask to False
        keep_mask[i] = True


interest_points_filtered=interest_points_holder[keep_mask] # kept singularities
interest_points_deleted=interest_points_holder[~keep_mask] # removed singularities
```
Save singularities in task space and in joint space.
```
# Save the filtered singularities as task singularities
singularities_task_space=np.copy(interest_points_filtered[:,:6])
with open('/content/drive/My Drive/Github/filtered_singularities_task_space_2.txt', 'w') as f:
    np.savetxt(f, singularities_task_space)

# Save the filtered singularities as joint singularities
singularities_joint_space=np.copy(interest_points_filtered[:,:6])
for i in range(len(singularities_task_space)):
  singularities_joint_space[i,:]=np.linalg.norm(platform.getIK(singularities_task_space[i,:]),axis=1)
with open('/content/drive/My Drive/Github/filtered_singularities_joint_space_2.txt', 'w') as f:
    np.savetxt(f, singularities_joint_space)
```
### Closest Singularity Search and Visualization
In this section is shown how to search for the closest singularity given a configuration of the platform and how to visualize the singularity space in both position and orientation space. Note: stewart platform's singularities live in R6, to visualize them we need to fix either position or orientation of the platform.

Load singularities from the drive
```
with open('/content/drive/My Drive/Github/filtered_singularities_task_space_2.txt', 'r') as f:
    singularities_task_space = np.loadtxt(f)

with open('/content/drive/My Drive/Github/filtered_singularities_joint_space_2.txt', 'r') as f:
    singularities_joint_space = np.loadtxt(f)
```
Find the closest singularity to defined position.

Check local condition index and the actuator forces in pose and singularity.
```
# find the closest singularity.
pose=np.array([0,0.35,0.2,0,0,0])
mins=np.array([x_min,y_min,z_min,roll_min,pitch_min,yaw_min])
maxs=np.array([x_max,y_max,z_max,roll_max,pitch_max,yaw_max])
mins = np.array(mins)
maxs = np.array(maxs)

# Normalize each component
normalized_pose = (pose - mins) / (maxs - mins)
# print(normalized_pose)
normalized_singularities=(singularities_task_space-mins)/(maxs-mins)
# print(normalized_singularities)
distances = np.linalg.norm(normalized_singularities - normalized_pose, axis=1)
# print(distances)
# # Find the index of the minimum distance
min_index = np.argmin(distances)
# print(distances[min_index])

# # Return the closest vector
closest_vector = singularities_task_space[min_index]


k=platform.getLocalConditionIndexT()
lengths=platform.getIK(pose)
k=platform.getLocalConditionIndexT()
Fg=np.array([-10,0,0,0,0,0])

joint_forces=platform.getActuatorForces(Fg)
platform.plot()
print("pose :", pose)
print("local condition number T :", k)
print("joint_forces :", joint_forces)


lengths=platform.getIK(closest_vector)
platform.plot()
k=platform.getLocalConditionIndexT()
joint_forces=platform.getActuatorForces(Fg)
print("closest singularity :", closest_vector)
print("local condition number T :", k)
print("joint_forces :", joint_forces)
```
<img src="https://github.com/Flamisell/StewartPlatformSingularities_py/blob/main/img/stewart1.png" width="400"> <img src="https://github.com/Flamisell/StewartPlatformSingularities_py/blob/main/img/Stewart22.png" width="450">


It is possible to plot 3d the singularity planes as long as we fix either position or orientation.
We can use plotly libraries to clearly see the planes. I will leave the code directly in the colab file.
```
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots
```
**Fixing Position**
```
# Choose a position
position = np.array([-0.45,0.1,0.2])

N=10 # discretization of space
choice=6 # choose choice 6 for local condition index
orientation_limits = [-10, 10, -10, 10, -10, 10] # use same orientation limits as singularity search
roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max = orientation_limits

# use the getIndexWorkspaceOrientation function to see how the local condition index changes when orienting the platform
workspace_indices_orientation = platform.getIndexWorkspaceOrientation(position, orientation_limits, N, choice)
```
<img src="https://github.com/Flamisell/StewartPlatformSingularities_py/blob/main/img/FixingPos.png" width="400">

**Fixing Orientation**
```
# Choose an orientation
orientation = np.array([8,7,5]) # RPY
N=10 # discretization of space
choice=6 # choose choice 6 for local condition index
workspace_limits = [-0.5, 0.5, -0.5, 0.5, 0.1, 0.6] # use same workspace limits as singularity search
x_min, x_max, y_min, y_max, z_min, z_max = workspace_limits

# use the getIndexWorkspacePosition function to see how the local condition index changes when positioning the platform
workspace_indices_position = platform.getIndexWorkspacePosition(orientation, workspace_limits, N, choice)
```
<img src="https://github.com/Flamisell/StewartPlatformSingularities_py/blob/main/img/FixingOr.png" width="400">

## Class Methods Overview
- **getIK(pose):** Computes inverse kinematics.
- **getFK(starting_pose, lengths_desired):** Computes forward kinematics.
- **getLocalConditionIndexT():** Calculates the local condition index of Transposed Jacobian
- **getPlatformForces(F_actuators):** Computes platform forces from actuator forces.
- **getSingularityWorkspace(workspace_limits,orientation_limits,N_pos,N_orient):** Evaluate singularities over a range of positions in the workspace.
- **plot():** Plots the Stewart platform configuration.
