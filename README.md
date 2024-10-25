# Our naming: sb = smallbelaz, TODO: rename this package to `sb_control`

# Changes to the original example:
- Removed a dependency on `bicycle_steering_controller`, brought its code into this source tree under directory `controller` and edited CMakeLists.txt correspondingly (taking inspiration from example 10)
- Renamed the controller from `bicycle_steering_controller/BicycleSteeringController` to `ros2_control_demo_example_11/BicycleSteeringController` and changed its namespace from `bicycle_steering_controller` to `sb_steering_controller`
- Edited controller manager config file and launch file accoridng to other changes

# Build and launch instructions
Building:
- The repository itself is a ros2 workspace, so run `colcon build --symlink-install` from root

Launching:
- Run `ros2 launch ros2_control_demo_example_11 carlikebot.launch.py` with underlay being active

# TODO:
- Rename things all around
- Clean up `ros2_control_demo_description`, get rid of `ros2_control_demo_testing`

---

# ros2_control_demo_example_11

   *CarlikeBot*, or ''Carlike Mobile Robot'', is a simple mobile base with bicycle drive.
   The robot has two wheels in the front that steer the robot and two wheels in the back that power the robot forward and backwards. However, since each pair of wheels (steering and traction) are controlled by one interface, a bicycle steering model is used.

Find the documentation in [doc/userdoc.rst](doc/userdoc.rst) or on [control.ros.org](https://control.ros.org/master/doc/ros2_control_demos/example_11/doc/userdoc.html).
