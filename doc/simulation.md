# How to Run Simulation
Make sure to follow setup_instructions.md first!

## Start the Development Docker
Open a terminal and run the start_dev_docker.sh script located in the scripts folder

## Source and Launch
source install/setup.bash
ros2 launch sparke_bringup simulation_bringup.launch.py

## Enter the Container From a Second Terminal
Open a second terminal and run the enter_dev_docker.sh script located in the scripts folder

## Source and Start Teleop
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run spot_micro_keyboard_command spotMicroKeyboardMove

## Note
Robot will struggle to move but motor commands will work. I still need to tune the parameters of the simulation.