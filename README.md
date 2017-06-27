# RobotLocalization
Determine the location of an underwater swimming robot in a mapped reef using a particle filter algorithm

This project was done for a class assignment, only the code that I have written has been uploaded.

This is a localizer node that uses ROS to communicate with our robot and it's sensors/motion controls. For the sake of this project it is assumed that the robot is only moving in the x y plane and is not moving up or down within the reef, as such the robot only makes yaw rotations. This code could be edited to allow the robot to roll and pitch, but would require more linear algebra for the image observation, as we would no longer only be looking directly down and the hegiht wouldn't be constant as it is in our situation. This change would take place when finding the points on the map that coorespond to the image that would be seen by each particle. There would also be a slight update to the motion call back however the algebra and implementation for this section would be easier.

This localizer uses a particle filter that can be initialized in either a kidnapped mode or a known start mode. In the kidnapped mode the starting location is not known so all particles are distributed randomly ithin the map, in the known start mode all particles are initialized to the known starting position.

The particle filter has 3 steps, Observation Update, Resample, and Motion Update.

The observation update will assign a weight to each particle based on how simmilar the image at that location is to the observed location. This weight is determined from a half-gaussian using the pixel simmilarity for the images. 

The Resample step uses the distribution from the observation step to randmoly sample N particles. Our estimated locations will then cluster around the points with highest confidence, or if all points are equal then randomly disperse the particles.

The Motion Update uses the robot's odometry to determine where each particle would now be after one time step of movement. As the odometry is not consitent random noise is added to both the rotation and speed for each particle, this also allows for our particles to avoid local minima.

This code works in conjunction with ROS and Gazebo using a robot designed by McGill University Robotics.
