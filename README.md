# Install
1. first create a ros2 workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```
2. clone the current repository into the ''src'' folder
```bash
cd src
git clone https://github.com/yguel/ihu_prismatic_joint.git
```
3. checkout the branch gazebo-classic
```bash
git checkout gazebo-classic
cd ../..
```
1. install the dependencies by running the following commands:
```bash
source /opt/ros/humble/setup.bash
vcs import src < src/ihu_prismatic_joint/ros2_prismatic_joint.repos
```
1. build the workspace by running the following commands from the root of the workspace:
```bash
source /opt/ros/humble/setup.bash
rosdep install --ignore-src --from-paths . -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source install/setup.bash
```

# Simulation
Start the simulation by running the following command:
```bash
source install/setup.bash
ros2 launch xy_unilivers_bringup gazebo_secured_ctrl_display.launch.py
```

# Just control the velocity of the motor
1. Start the ethercat communication (after [having installed the igh ethercat master](https://icube-robotics.github.io/ethercat_driver_ros2/quickstart/installation.html) ) by running the following command:
```bash
sudo /etc/init.d/ethercat start
```
2. 
Start the real robot control:
```bash
source install/setup.bash
ros2 launch xy_unilivers_bringup xy_unilivers.launch.py
```

# On the real robot with security enabled (secured mode) by limit switches
1. Start the ethercat communication (after [having installed the igh ethercat master](https://icube-robotics.github.io/ethercat_driver_ros2/quickstart/installation.html) ) by running the following command:
```bash
sudo /etc/init.d/ethercat start
```
2. 
Start the real robot control:
```bash
source install/setup.bash
ros2 launch xy_unilivers_bringup secured_ctrl_display_launch.py
```