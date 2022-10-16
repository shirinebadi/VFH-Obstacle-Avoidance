# VFH Obstacle Avoidance

Implementation of a new real-time obstacle avoidance method for mobile robots, named "vector field histogram (VFH)".
This method first introduced in [**THE VECTOR FIELD HISTOGRAM - FAST OBSTACLE AVOIDANCE FOR MOBILE ROBOTS**](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/borenstein_VFHisto.pdf)

## Method

The VFH method uses a two-dimensional Cartesian histogram grid as a world model. This world
model is updated continuously with range data sampled by on-board range sensors
