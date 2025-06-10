This repo contains docker containers mostly related to robotics.


Images can be found on docker hub:

- CMU_container: https://hub.docker.com/r/camreed60/cmu_container 


Run images in command line using ```docker run -it --rm --net=host --ipc=host -e FASTDDS_BUILTIN_TRANSPORT=UDPv4 -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0 <IMAGE_NAME>```.
This ensures that the host machine and container can communicate. 
