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

## Particle Filter Pipeline

## Takeaways and Conclusion


## Writeup

- Objectives: The goals of this project is to gain experience with robot localization, specifically the particle filter algorithm. What this means is that we want the turtlebot to determine where it is located with respect to its environemnt. 

- We call the initialize_particle_cloud function within the ParticleFilter class to initialize the particle cloud. In this function, we first want to get the width, height, and origin (x,y) values from our map. We then use these values and the random_sample() function to get a position with random x and y values and to get a random z_angular value for orientation. We initialize a Pose object with this position and orientation. We then create our particle object with this pose, set its weight to 1, and add it to our particle cloud list. After we get our particle cloud list (i.e. gone through the for loop), we normalize the particle weights so it sums to 1 with a helper function (normalize_particles()). In this function, we essentially divide each of the particle's weight by the total weight of all the particles in the particle cloud. Finally, after normalizing the particles, we publish it.

- We call the update_particles_with_motion_model() function to handle the movement model. In this function, we essentially followed the sample motion model odometry referred in the textbook and from the professor in Slack. The robot in the time interval (t-1, t] rotates in about delta_rot1, translated by delta_trans, and rotates again by delta_rot2. We first get the current and the old x, y, and theta values from the odometry. We then set the delta as the difference between the current and old odometries' x, y, and theta values. We then calculate the delta_rot1, delta_trans, and delta_rot2 using the given formulas in the algorithm. We then account for noise by subtracting each of the deltas with a random sample value which we get by calling the random.normal function with a mean of 0 and a standard deviation of 0.1 for simplicity. For each of the particle, we update the theta by adding delta_rot1 and delta_rot2 to it and the x and y values by adding the delta_trans while multiplying by the cos and sin respecitvely of the particle's theta plus delta_rot1. 

- We call the resample_particles() function to handle resampling the particles. In this function, we first extract the weights of each particle in the particle cloud and put it in a list. We then use the function np.random.choice() to get our resampling as it allows us to add the parameter of our weights list into it so that the resampling would be weighted. We added a for loop to go through the resample list and confirm to set its values from the resampling as we had some concurrency issues. We would then set the particle cloud to the resample list.

- We call the update_estimated_robot_pose() function to handle updating estimated robot pose. In this function, we took the average of the x, y, and theta of each particle in the particle cloud to get the new estimated robot pose. For the x and y, the function would go through the particle cloud and sum up the x's and y's of each particle and then we would divide them by the number of particles. For theta, we would get the yaw of each particle and then sum up the sin's and cos's of each particle's yaw. We then took the arc tan of the average sin over the average cos to get our average theta (got idea from https://stackoverflow.com/questions/491738/how-do-you-calculate-the-average-of-a-set-of-circular-data/491769#:~:text=return%20atan2%20(y_part%20/%20size%2C%20x_part%20/%20size). We then set the new pose for the robot based off our calculated average x, y, and theta.

