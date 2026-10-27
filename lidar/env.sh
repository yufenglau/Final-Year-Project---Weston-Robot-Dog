# ROS env for our LiDAR work on PC2 (same as ~/.bashrc, plus our workspace)
source /opt/ros/humble/setup.bash
source /home/unitree/slam_config/cyclonedds_go2_B2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/home/unitree/slam_config/cyclonedds_go2_B2_ws/cyclonedds.xml
[ -f ~/unitree_ros2/install/setup.bash ] && source ~/unitree_ros2/install/setup.bash  # gone after Weston reset (20 Oct)
source ~/a2_lidar/ws/install/setup.bash
# one maths thread per program: numpy/OpenBLAS otherwise spins several cores each (7 Oct)
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
