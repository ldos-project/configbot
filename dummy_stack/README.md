# E2E Workflow Demonstration
```bash
# starting and attaching to the container
docker rm --force ros-container
docker run -d --network host -v $PWD:/root/configbot --cgroupns host --name ros-container rohitdwivedula/ros:noetic-adaptor sleep infinity

# build ConfigBot
docker exec -it ros-container /bin/bash # to open a terminal in the container
cd /root/configbot/optim && make clean && make

# run all the ROS nodes
cd $HOME/configbot/dummy_stack/ && bash run_all.sh

docker update --cpus="0.5" ros-container 
# run the optimization
source devel/setup.bash && python3 iterate.py
```