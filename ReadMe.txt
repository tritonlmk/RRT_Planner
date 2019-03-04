rrt planner package depends on:
navigation libraries: map_server(nav_msgs)
Opencv2
Eigen3
tiguan_movebase package

for map_server msg and srv, pls refer to ROS wiki
rrt planner package provide service:
header: rrt_planner/GlobalPath.h
service name:/global_path_sender

rrt_planner srv:
geometry_msgs/Pose2D destination
geometry_msgs/Pose2D initial_config
---
geometry_msgs/Pose2D[] globalpath

coordiante is the map_server nav_msgs OccupancyGrid coordiante, in which point (0, 0) is equal to the left lower corner of the map_ps.pgm picture