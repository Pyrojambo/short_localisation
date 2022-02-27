A one day exercise to perform localisation from encoder odometry and GNSS.

# Install

First clone the repo into your catkin workspace
`git clone https://github.com/Pyrojambo/short_localisation.git`
From root of catkin workspace run:
`rosdep install short_localisation`
then build the package
`catkin build short_localisation`
and source workspace
`source devel/setup.bash`

# Method 1:

Use the extended Kalman filter (ekf) and unscented Kalman filter (ukf) from [robot_localization](http://wiki.ros.org/robot_localization)

## Configuration

The parameters(short_localisation/config/ekf.yaml) for using robot_localisation were configured as follows:
- odom0 was set to use `sensors/odom`
    - Used the linear x and y velocities
    - Used the angular yaw position
        - Yaw position instead of velocity was used as we had no other reference for the yaw position and needed to have a starting position
    - pose outlier rejection was disabled so that our initial orientation would not be ignored

- odom1 was set to use `sensors/gnss/odom`
    - Used the linear x and y positions
    - pose outlier rejection was disabled so that our initial position would not be ignored

The same configuration was used for the ukf for parameters that appeared in both

## Run

Run
`roslaunch short_localisation robot_localisation.launch`
then press space to start `.bag` file

# Method 2:

Implement own version of localisation

## Run

Run
`roslaunch short_localisation custom_localisation.launch`
then press space to start `.bag` file

## Process

### Step 1

- Create a framework for subscribing to the ROS msgs and publishing the fused output and the `base_link` tf frame
- Use a naive approach of averaging the positions to test the framework
- Extend the naive approach to use encoder velocities instead of pose


