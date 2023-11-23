# soar_docker

## how to build
```bash
sudo docker build -t soar:2.0 .
```


## how to run
```bash
xhost +si:localuser:root
docker run -it -e DISPLAY --device=/dev/ttyUSB0 --privileged -v /home/sp/Desktop/docker/docker_folder:/docker_folder -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix --network=host soar:2.0
```
note: you need to custom
```bash
-v /home/sp/Desktop/docker/docker_folder:/docker_folder
```
to your own directory. It's just for me.


## how to add terminal
```bash
docker exec -it 프로세스ID bash
```
and you can search 프로세스ID by
```bash
docker ps
```


---
## in container,
### to run unilidar
```bash
cd unilidar_sdk/unitree_lidar_ros
source devel/setup.bash
roslaunch unitree_lidar_ros run_without_rviz.launch
```


### to run pointlio
```bash
cd catkin_point_lio_unilidar
source devel/setup.bash
roslaunch point_lio_unilidar mapping_unilidar.launch
```


### to offboard control
```bash
source ~/.bashrc
roslaunch offboard_py start_offb.launch
```
