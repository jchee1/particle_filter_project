# particle_filter_project

## Implementation Plan

### Team Members (github/cnet):
RD Babiera (rdbabiera), Jason Chee (jchee1)

### Questions
How will you initialize your particle cloud (initialize_particle_cloud)?
- For the number of particles in the particle filter, we will sample an x and 
y coordinate based off of the map origin and boundaries. For each generated 
particle, we will randomly generate an orientation based off of the input 
range. Each particle will then be appended to self.particle_cloud.

How will you update the position of the particles will be updated based on the 
movements of the robot (update_particles_with_motion_model)?
- For each particle, we can use the self.odom_pose (current), as well as the 
self.odom_pose_last_motion_update (previous) states to calculate a pose offset and 
update particle states.

How will you compute the importance weights of each particle after receiving the 
robot's laser scan data?(update_particle_weights_with_measurement_model)?
- For each of the particles, we can use the range finders algorithm that will be 
discussed in class 6. We can get z_t from the data.ranges and z_{max} from data.ranges.max.
We'll get x_t from the particle's pose. We can also use the simpler line of q = q*prob(dist, sigma)

How will you normalize the particles' importance weights (normalize_particles) 
and resample the particles (resample_particles)?
- In normalize_particles, we can take the sum of all particle weights in 
update_particle_weights_with_measurement_model, and divide each weight by 
this sum. We can then create an array of indices and another array corresponding 
to normalized weights, and calls to np.random_sample or np.random_choice for the 
amount of new particles will give a new array of resampled particles. In addition, 
we can also apply a weight threshold to eliminate 

How will you update the estimated pose of the robot (update_estimated_robot_pose)

- We could implement a naive clustering algorithm in which we divide the map 
into a grid and then we take the most dense grid and find the average position/
orientation of all particles within that grid.

How will you incorporate noise into your particle filter localization?
- We can implement Gaussian noise in the update_particle_weights_with_measurement_model 
for both position and orientation. For both position and orientation, we use a 
mean of zero and standard deviation cof 0.1. Then, we will multiply the sampled 
value by a constant, most likely different for each category. Position would be 
a tiny fraction of a meter while orientation would be a few degrees.


### Timeline

- 4/13: Get the mapping done in lab C
- 4/16: initializing particle cloud
- 4/20: updating particles with motion and measurement models function implemented (maybe
    initially do without noise then add noise once later)
- 4/22: implement updating robot pose function and test on robot
- 4/24: More testing and debugging and recording videos / rosbags

## Objectives
The goal of this project is to gain experience with robot localization through the 
usage of probablistic approaches. Through the particle filter algorithm, we aim 
to allow the Turtlebot to determine its location with respect to its environment 
given its sensor readings and a map of the environment.

## High Level Description
First we initialize the particle cloud by randoming sampling particles across our map.
Then when we receive new scans, we essentially go through the process of going through our motion
model to update the particles based off of the odometry from our scan, our measurement model to update the weights, 
normalize the particles so that the weights sum up to 1, resample the particles with higher weight particles having priority, 
updating the estimated robot's pose based off of the resampled particles, and then publishing our updated particle cloud
and updated robot pose estimate. We repeat this process each time we get new scans.

## Demonstration
![](https://github.com/jchee1/particle_filter_project/blob/main/gifs/particle_filter_demo.gif)

## Particle Filter Pipeline
Our implementation utilizes two python files - likelihood_field.py and 
particle_filter.py, both of which are contained in the ./scripts folder.
### Step 1: Initialization of Particle Cloud
We initialize the particle cloud through the ParticleFilter class' 
initialize_particle_cloud method. Before a call to this function, the __init__ 
method of this class makes a call to self.likelihood_field.get_obstacle_bounding_box(). 
This method provided by the teaching staff finds a lower bound and upper bound 
for both x and y by determining which cells in the map are occupied by scanned 
walls. For this bounding box, we use random_sample() to randomize x and y values 
corresponding to particle position as well as the z value for orientation; these 
values are inputted to the Particle class' Pose object and weight is set to 1. 
Once the particle cloud list is initialized, we normalize weights such that all 
particle weights sum to 1 using the helper function normalize_particles(). This 
function divides each particle's weight by the total weight of all particles 
contained in the cloud. Lastly, the cloud is published using publich_particle_cloud().

### Step 2: Movement Model
The ParticleFilter class' update_particles_with_motion_model() function handles 
our implementation's movement model; within it, we follow the 
sample_motion_model_odometry pseudocode from the Probablistic Robotics textbook 
and from the course staff in Slack. The robot in the time interval (t-1, t] rotates 
in about delta_rot1, translated by delta_trans, and rotates again by delta_rot2. 
We first get the current and the old x, y, and theta values from the odometry. 
We then set the delta as the difference between the current and old odometries' 
x, y, and theta values. We then calculate the delta_rot1, delta_trans, and 
delta_rot2 using the given formulas in the algorithm. We then account for noise 
by subtracting each delta with a random sample from a Gaussian distribution with 
mean 0 and stdev 0.05 for each rotation, and 0.1 for translation. For each particle, 
we update theta by adding delta_rot1 and delta_rot2 to it and the x and y values 
by adding the delta_trans while multiplying by the cos and sin respecitvely of 
the particle's theta plus delta_rot1. 

### Step 3: Measurement Model
The ParticleFilter class' update_particle_weights_with_measurement_model() and 
the LikelihoodField class handle updating particle weights using the measurement 
model. We readapt the likelihood field methods provided by the course staff. 
For each particle in the particle cloud, for each 8th of the unit circle, x_ztk 
and y_ztk are calculated by adding the particle's position to the robot's range 
finder measurement for that given angle multipled by the cos or sin of the 
particle's orientation added to the angle being tested. Then, the closest distance 
to this calculated position is determined using the LikelihoodField class' 
get_closest_obstacle_distance() method, and the probability of this distance 
being correct is computed using a Gaussian with mean 0 and stdev 0.1. It is 
important to note that each particle starts with a weight of 1, and this weight 
is multiplied by the computed probability. Should the particle be outside of the 
map, the weight is automatically set to zero. After all angles are accounted for, 
the particle's weight is set to the running total.

### Step 4: Resampling
The ParticleFilter class' resample_particles() method handles resampling of 
particles. In this function, we first extract the weights of each particle in the 
cloud and put it in a list. We then use the function np.random.choice() to get our 
resampling as it allows us to add the parameter of our weights list into it so 
that the resampling would be weighted. A for loop then goes through the resampling 
list and each resample index is replaced with a new Particle object with the same 
parameters as the resampled particle in order to avoid concurrency issues. 
The particle cloud is then set to the resample list.

### Step 5: Incorporation of Noise
Incorporation of noise is handled both within the ParticleFilter class' 
update_particle_weights_with_measurement_model() and the 
update_particles_with_motion_model() methods. Within the measurement model, 
noise is accounted for by utilizing the probability that a calculated distance 
given a particle's position and the robot's scanning data. Within the motion 
model, noise is added subtracting randomly generated values from the changes in 
rotation and position of each particle. The first rotation noise aids in calculating 
the new position values of the robot by simulating random turning before moving, 
while a second rotation noise simulates the robot turning after stopping. Position 
noise is used in order to account for early and late stopping of the robot.

### Step 6: Updating Estimated Robot Pose
We use the ParticleFilter class' update_estimated_robot_pose() method in order 
to update the robot's pose. The method creates a blank estimate of x, y, and 
yaw as well as keeping a running total for sin and cos values. For each particle, 
the x and y positions are added to the running totals, and cos and sin values of 
the yaw are added to their respective fields. If yaw is less than zero, we add 
2pi such that the angle ranges are from [0, 360) as opposed to (-180, 180). Lastly, 
the average x and y value determine position, while the math.atan2 method takes 
the average sin over average cos to calculate theta. This was inspired by 
this post (https://stackoverflow.com/questions/491738/how-do-you-calculate-the-average-of-a-set-of-circular-data/491769#:~:text=return%20atan2%20(y_part%20/%20size%2C%20x_part%20/%20size). The robot pose is then set to these values.

### Step 7: Optimization of Parameters
The optimization of parameters across the project was done for the most part 
through experimentation. For example, the motion model's stdev values when 
generating a random value were decided to be 0.05 for each rotation and 0.1 for 
translation through visual fine tuning; larger values were seen to prevent late 
convergence while smaller values would create little spread in particles and 
and prevent convergence at all. The other important parameter chosen in this 
project was the angles iterated over through in the measurement model. Because 
iterating over all angles is costly, and angles outside of the LiDAR range are 
unaccounted for, 8 evenly spaced angles allowed us to have equal contribution 
from all important scans from the LiDAR and much faster runtime.

## Challenges
One challenge we had was when updating the robot estimated pose, we had trouble calculating the average theta. We intially tried to do the regular way of taking averages (i.e. taking sum of thetas and then dividing by the number of particles); however, there were edge cases that didn't work with this method. So what we instead did was sum up the sin's and cos's of each particle's yaw and took the arc tan of the average sin over the average cos. Another challenge was when incorporting our likelihood measurement model, we first tried to use all 360 degrees from the data.ranges list; however, this turned out to be very computationaly slow. What we instead did was picked which 8 angles to use corresponding to sections like front, front-left, left, right, etc.

## Future Work
Given more time, there are two major improvements that can be made to our particle 
filter implementation. First, when updating weights with the measurement model, we 
utilize a simplified version of the likelihood field which sets q = q * prob(dist, 
sigma_hit). A more complex alternative would be to use q = q * (z_hit * 
prob(dist, sigma_hit) + z_random/z_max), which accounts for all of measurement noise, 
unexpected objects, failure to detect obstacles, and random measurements. Within updating particles 
with the motion model, random samples are drawn from a Gaussian with mean 0 and 
varying standard deviation. The robotics textbooks presents the sampling function 
as a function of different alpha coefficients as well as different deltas on 
different measurements (ie delta_hat of rot1 = rot1 - sample(a_1 * rot1 + a_2 * trans)). 
Exploring less naive noise techniques should certainly improve the model's performance. 

## Takeaways
1. Pair programming through VSCode LiveShare is one of the most efficient ways 
to work together. Because the code is always up to date on one computer that is 
usually connected to the robot and merge conflicts through Git become nonexistent, 
it becomes easy to work on separate parts of the codebase at once and very 
little time gets wasted with this overhead.

2. Many debugging issues within this project may be the result of object 
referencing schemes or math background both partners may not have a perfect grasp on. 
One of our main issues pertained to unit testing in which all particles resample 
to the same particle, and reweighting the particle resulted in the weight of 
each particle decreasing as the loop continued as opposed to holding the 
weight the same; this issue was the result of referencing the same object every 
time throughout a method call. Our average angle dilemma needed to be solved 
using sin and cos averages instead of flat angle averages, which wasn't intuitive 
in the first place. The main idea is of this bullet point is that having a grasp 
on outside-of-class topics rewards development speed vastly.
