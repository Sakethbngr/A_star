# A_star Phase 1

Open and run the .py file
Enter the required parameters
The parameters are: x & y co-ordinates of start and goal points, stating and goal orientation and step size.
The code implements A star algorithm and finds out the optimal path.

# A_star Phase 2

Open the .py file and run it
Enter the required inputs such as start points, orientation, goal and rpm values when prompted. 
The heuristic value is set to 1.5 (line 110 in code, feel free to change it to increase the optimality of the path) and the clearance as 0.2.

PS: We have scaled this map by 100 times
output video and image will be saved in the current directory

# A_star in Gazebo


In the following instructions, we assume your catkin workspace is named catkin_ws and is located in your home directory.

cd ~/catkin_ws/src

git clone https://github.com/Sakethbngr/A_star.git

rename the package as astar_gazebo661 from astar_gazeobo661
cd ~/catkin_ws

catkin build

source ~/catkin_ws/devel/setup.bash

Try roscd astar_gazebo661 to make sure the package can be found.

We need the Burger model for this assignment. Make sure you have the following line in .bashrc:

export TURTLEBOT3_MODEL=burger
source the file using $ source devel/setup.bash

Start the environment by running the following command on terminal:

roslaunch astar_gazebo661 astar_gazebo.launch
make sure to run the command " chmod +x task " for file named task in this location : catkin_ws/src/astar_gazebo661/nodes

Dependancies numpy opencv math
