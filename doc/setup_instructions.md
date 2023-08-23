# Setup

## Install Nvidia Container Toolkit & Docker
https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

## Install Nvidia Container Runtime
sudo apt-get install nvidia-container-runtime

## Set ROS_DOMAIN_ID ENV VAR
export ROS_DOMAIN_ID=id
Replace the id with your chosen domain id. If you do not know what this means just use the number 1.
This must be done everytime a terminal is opened. Add it to your .bashrc file to automate it.

## Run initial_setup.sh
setup/initial_setup.sh

## Build The Docker
docker-compose build sparke

## Run the Docker
docker-compose up sparke

## Simulation
For simulation please look at simulation.md