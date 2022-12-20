# omni-mapper

Still in development, this is an autonomy stack for, right now an omni-wheeled robot, maybe, in the future it can be agnostic?

It current consists of a node that can perform a Vanilla ICP algorithm on 2d lidar scans. The next steps of this are to:
1) Write a node that can output either wheeled encoder or IMU odometry
2) Write a Kalman Filter node to output the fusion of the two sensors mentioned.
