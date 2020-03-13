# Controller_tb3
This repo is to build controllers for ground vehicle, such as PID controller, LQR controller, etc,.

The controller is based on the Turtlebot3 simulator, you should follow the instrutions on [TurtleBot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) to install related dependencies. You don't have to install the simulator when you install the dependencies since the code of the simulator are already contained in this repo.

Once you have installed the dependencies, cd to your home directory and follow the below instructions to start the controller.

# Build and Run
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/ArlenCHEN/Controller_for_tb3.git
cd ..
catkin build

```
Open one terminal and 
```
source devel/setup.bash
roslaunch turtlebot3_fake turtlebot3_fake.launch 
```
This will start the rviz with a turtlebot model. 
Now open another terminal and
```
source devel/setup.bash
roslaunch tb3_control control.launch
```
This will launch a planner node and a controller node and the turtlebot model will track the desired trajectory, which you can define yourself in tb3_planner package.
