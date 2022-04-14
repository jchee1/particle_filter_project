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