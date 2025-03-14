acutm:~/ros2_ws] 
[macutm:~/ros2_ws] ros2 launch linorobot2_navigation slam.launch.py 
[INFO] [launch]: All log files can be found below /home/pitosalas/.ros/log/2025-03-12-10-55-03-649379-macutm-20814
[INFO] [launch]: Default logging verbosity is set to INFO

[INFO] [controller_server-1]: process started with pid [20830]
[INFO] [smoother_server-2]: process started with pid [20831]
[INFO] [planner_server-3]: process started with pid [20832]
[INFO] [behavior_server-4]: process started with pid [20833]
[INFO] [bt_navigator-5]: process started with pid [20834]
[INFO] [waypoint_follower-6]: process started with pid [20835]
[INFO] [velocity_smoother-7]: process started with pid [20836]
[INFO] [collision_monitor-8]: process started with pid [20837]
[INFO] [opennav_docking-9]: process started with pid [20838]
[INFO] [lifecycle_manager-10]: process started with pid [20839]
[INFO] [async_slam_toolbox_node-11]: process started with pid [20844]
[waypoint_follower-6] [INFO] [1741791304.068766346] [waypoint_follower]: 
[waypoint_follower-6] 	waypoint_follower lifecycle node launched. 
[waypoint_follower-6] 	Waiting on external lifecycle transitions to activate
[waypoint_follower-6] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[waypoint_follower-6] [INFO] [1741791304.069074555] [waypoint_follower]: Creating
[velocity_smoother-7] [INFO] [1741791304.100260148] [velocity_smoother]: 
[velocity_smoother-7] 	velocity_smoother lifecycle node launched. 
[velocity_smoother-7] 	Waiting on external lifecycle transitions to activate
[velocity_smoother-7] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[controller_server-1] [INFO] [1741791304.137793493] [controller_server]: 
[controller_server-1] 	controller_server lifecycle node launched. 
[controller_server-1] 	Waiting on external lifecycle transitions to activate
[controller_server-1] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[controller_server-1] [INFO] [1741791304.157928332] [controller_server]: Creating controller server
[behavior_server-4] [INFO] [1741791304.193092468] [behavior_server]: 
[behavior_server-4] 	behavior_server lifecycle node launched. 
[behavior_server-4] 	Waiting on external lifecycle transitions to activate
[behavior_server-4] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[smoother_server-2] [INFO] [1741791304.201936304] [smoother_server]: 
[smoother_server-2] 	smoother_server lifecycle node launched. 
[smoother_server-2] 	Waiting on external lifecycle transitions to activate
[smoother_server-2] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[smoother_server-2] [INFO] [1741791304.202859805] [smoother_server]: Creating smoother server
[lifecycle_manager-10] [INFO] [1741791304.211748057] [lifecycle_manager_navigation]: Creating
[collision_monitor-8] [INFO] [1741791304.218716310] [collision_monitor]: 
[collision_monitor-8] 	collision_monitor lifecycle node launched. 
[collision_monitor-8] 	Waiting on external lifecycle transitions to activate
[collision_monitor-8] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[planner_server-3] [INFO] [1741791304.228966563] [planner_server]: 
[planner_server-3] 	planner_server lifecycle node launched. 
[planner_server-3] 	Waiting on external lifecycle transitions to activate
[planner_server-3] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[planner_server-3] [INFO] [1741791304.239042941] [planner_server]: Creating
[async_slam_toolbox_node-11] [INFO] [1741791304.248560402] [slam_toolbox]: Node using stack size 40000000
[lifecycle_manager-10] [INFO] [1741791304.250091361] [lifecycle_manager_navigation]: Creating and initializing lifecycle service clients
[bt_navigator-5] [INFO] [1741791304.253130112] [bt_navigator]: 
[bt_navigator-5] 	bt_navigator lifecycle node launched. 
[bt_navigator-5] 	Waiting on external lifecycle transitions to activate
[bt_navigator-5] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[bt_navigator-5] [INFO] [1741791304.254062820] [bt_navigator]: Creating
[controller_server-1] [INFO] [1741791304.257325738] [local_costmap.local_costmap]: 
[controller_server-1] 	local_costmap lifecycle node launched. 
[controller_server-1] 	Waiting on external lifecycle transitions to activate
[controller_server-1] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[controller_server-1] [INFO] [1741791304.257535738] [local_costmap.local_costmap]: Creating Costmap
[opennav_docking-9] [INFO] [1741791304.278854536] [docking_server]: 
[opennav_docking-9] 	docking_server lifecycle node launched. 
[opennav_docking-9] 	Waiting on external lifecycle transitions to activate
[opennav_docking-9] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[opennav_docking-9] [INFO] [1741791304.279180828] [docking_server]: Creating docking_server
[planner_server-3] [INFO] [1741791304.317891299] [global_costmap.global_costmap]: 
[planner_server-3] 	global_costmap lifecycle node launched. 
[planner_server-3] 	Waiting on external lifecycle transitions to activate
[planner_server-3] 	See https://design.ros2.org/articles/node_lifecycle.html for more information.
[planner_server-3] [INFO] [1741791304.318794299] [global_costmap.global_costmap]: Creating Costmap
[async_slam_toolbox_node-11] [INFO] [1741791304.381760360] [slam_toolbox]: Configuring
[async_slam_toolbox_node-11] [INFO] [1741791304.390577988] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
[async_slam_toolbox_node-11] [INFO] [1741791304.399730532] [slam_toolbox]: CeresSolver: Using SCHUR_JACOBI preconditioner.
[INFO] [launch.user]: [LifecycleLaunch] Slamtoolbox node is activating.
[async_slam_toolbox_node-11] [INFO] [1741791304.442657879] [slam_toolbox]: Activating
[lifecycle_manager-10] [INFO] [1741791304.926858195] [lifecycle_manager_navigation]: Starting managed nodes bringup...
[lifecycle_manager-10] [INFO] [1741791304.926885070] [lifecycle_manager_navigation]: Configuring controller_server
[controller_server-1] [INFO] [1741791304.928299362] [controller_server]: Configuring controller interface
[controller_server-1] [INFO] [1741791304.928315237] [controller_server]: getting progress checker plugins..
[controller_server-1] [INFO] [1741791304.928674529] [controller_server]: getting goal checker plugins..
[controller_server-1] [INFO] [1741791304.929442029] [controller_server]: Controller frequency set to 20.0000Hz
[controller_server-1] [INFO] [1741791304.929488446] [local_costmap.local_costmap]: Configuring
[controller_server-1] [INFO] [1741791304.956307371] [local_costmap.local_costmap]: Using plugin "obstacle_layer"
[controller_server-1] [INFO] [1741791304.998321217] [local_costmap.local_costmap]: Subscribed to Topics: scan base_scan
[controller_server-1] [INFO] [1741791305.088074245] [local_costmap.local_costmap]: Initialized plugin "obstacle_layer"
[controller_server-1] [INFO] [1741791305.088097787] [local_costmap.local_costmap]: Using plugin "voxel_layer"
[controller_server-1] [INFO] [1741791305.090704871] [local_costmap.local_costmap]: Subscribed to Topics: pointcloud
[controller_server-1] [INFO] [1741791305.145587596] [local_costmap.local_costmap]: Initialized plugin "voxel_layer"
[controller_server-1] [INFO] [1741791305.145615763] [local_costmap.local_costmap]: Using plugin "sonar_layer"
[controller_server-1] [INFO] [1741791305.149716806] [local_costmap.local_costmap]: sonar_layer: ALL as input_sensor_type given
[controller_server-1] [INFO] [1741791305.150291556] [local_costmap.local_costmap]: RangeSensorLayer: subscribed to topic /sonar
[controller_server-1] [INFO] [1741791305.150304931] [local_costmap.local_costmap]: Initialized plugin "sonar_layer"
[controller_server-1] [INFO] [1741791305.150308431] [local_costmap.local_costmap]: Using plugin "inflation_layer"
[controller_server-1] [INFO] [1741791305.151825307] [local_costmap.local_costmap]: Initialized plugin "inflation_layer"
[controller_server-1] [INFO] [1741791305.215499868] [controller_server]: Created progress_checker : progress_checker of type nav2_controller::SimpleProgressChecker
[controller_server-1] [INFO] [1741791305.217207952] [controller_server]: Controller Server has progress_checker  progress checkers available.
[controller_server-1] [INFO] [1741791305.217498077] [controller_server]: Created goal checker : general_goal_checker of type nav2_controller::SimpleGoalChecker
[controller_server-1] [INFO] [1741791305.217674410] [controller_server]: Controller Server has general_goal_checker  goal checkers available.
[controller_server-1] [INFO] [1741791305.219914869] [controller_server]: Created controller : FollowPath of type dwb_core::DWBLocalPlanner
[controller_server-1] [INFO] [1741791305.222560954] [controller_server]: Setting transform_tolerance to 0.200000
[controller_server-1] [INFO] [1741791305.252426796] [controller_server]: Using critic "RotateToGoal" (dwb_critics::RotateToGoalCritic)
[controller_server-1] [INFO] [1741791305.254506505] [controller_server]: Critic plugin initialized
[controller_server-1] [INFO] [1741791305.254757588] [controller_server]: Using critic "Oscillation" (dwb_critics::OscillationCritic)
[controller_server-1] [INFO] [1741791305.256233339] [controller_server]: Critic plugin initialized
[controller_server-1] [INFO] [1741791305.256286589] [controller_server]: Using critic "BaseObstacle" (dwb_critics::BaseObstacleCritic)
[controller_server-1] [INFO] [1741791305.256354964] [controller_server]: Critic plugin initialized
[controller_server-1] [INFO] [1741791305.256416672] [controller_server]: Using critic "GoalAlign" (dwb_critics::GoalAlignCritic)
[controller_server-1] [INFO] [1741791305.256580047] [controller_server]: Critic plugin initialized
[controller_server-1] [INFO] [1741791305.256665839] [controller_server]: Using critic "PathAlign" (dwb_critics::PathAlignCritic)
[controller_server-1] [INFO] [1741791305.256780756] [controller_server]: Critic plugin initialized
[controller_server-1] [INFO] [1741791305.256822089] [controller_server]: Using critic "PathDist" (dwb_critics::PathDistCritic)
[controller_server-1] [INFO] [1741791305.256925964] [controller_server]: Critic plugin initialized
[controller_server-1] [INFO] [1741791305.256995256] [controller_server]: Using critic "GoalDist" (dwb_critics::GoalDistCritic)
[controller_server-1] [INFO] [1741791305.257114464] [controller_server]: Critic plugin initialized
[controller_server-1] [INFO] [1741791305.257124131] [controller_server]: Controller Server has FollowPath  controllers available.
[lifecycle_manager-10] [INFO] [1741791305.300813894] [lifecycle_manager_navigation]: Configuring smoother_server
[smoother_server-2] [INFO] [1741791305.317061608] [smoother_server]: Configuring smoother server
[smoother_server-2] [INFO] [1741791305.437590770] [smoother_server]: Created smoother : simple_smoother of type nav2_smoother::SimpleSmoother
[smoother_server-2] [INFO] [1741791305.440018729] [smoother_server]: Smoother Server has simple_smoother  smoothers available.
[lifecycle_manager-10] [INFO] [1741791305.462120069] [lifecycle_manager_navigation]: Configuring planner_server
[planner_server-3] [INFO] [1741791305.462960111] [planner_server]: Configuring
[planner_server-3] [INFO] [1741791305.463236861] [global_costmap.global_costmap]: Configuring
[planner_server-3] [INFO] [1741791305.490706453] [global_costmap.global_costmap]: Using plugin "static_layer"
[planner_server-3] [INFO] [1741791305.520943171] [global_costmap.global_costmap]: Subscribing to the map topic (/map) with transient local durability
[planner_server-3] [INFO] [1741791305.541131594] [global_costmap.global_costmap]: Subscribing to updates
[planner_server-3] [INFO] [1741791305.576961772] [global_costmap.global_costmap]: Initialized plugin "static_layer"
[planner_server-3] [INFO] [1741791305.576988313] [global_costmap.global_costmap]: Using plugin "obstacle_layer"
[planner_server-3] [INFO] [1741791305.577302813] [global_costmap.global_costmap]: Subscribed to Topics: scan base_scan
[planner_server-3] [INFO] [1741791305.607231614] [global_costmap.global_costmap]: Initialized plugin "obstacle_layer"
[planner_server-3] [INFO] [1741791305.607256823] [global_costmap.global_costmap]: Using plugin "voxel_layer"
[planner_server-3] [INFO] [1741791305.607501448] [global_costmap.global_costmap]: Subscribed to Topics: pointcloud
[planner_server-3] [INFO] [1741791305.623643578] [global_costmap.global_costmap]: Initialized plugin "voxel_layer"
[planner_server-3] [INFO] [1741791305.624246328] [global_costmap.global_costmap]: Using plugin "sonar_layer"
[planner_server-3] [INFO] [1741791305.626388662] [global_costmap.global_costmap]: sonar_layer: ALL as input_sensor_type given
[planner_server-3] [INFO] [1741791305.629851121] [global_costmap.global_costmap]: RangeSensorLayer: subscribed to topic /sonar
[planner_server-3] [INFO] [1741791305.631807538] [global_costmap.global_costmap]: Initialized plugin "sonar_layer"
[planner_server-3] [INFO] [1741791305.631813497] [global_costmap.global_costmap]: Using plugin "inflation_layer"
[planner_server-3] [INFO] [1741791305.632610664] [global_costmap.global_costmap]: Initialized plugin "inflation_layer"
[planner_server-3] [FATAL] [1741791305.710003354] [planner_server]: Failed to create global planner. Exception: According to the loaded plugin descriptions the class nav2_navfn_planner/NavfnPlanner with base class type nav2_core::GlobalPlanner does not exist. Declared types are  nav2_navfn_planner::NavfnPlanner nav2_smac_planner::SmacPlanner2D nav2_smac_planner::SmacPlannerHybrid nav2_smac_planner::SmacPlannerLattice nav2_theta_star_planner::ThetaStarPlanner
[planner_server-3] [INFO] [1741791305.710490105] [planner_server]: Cleaning up
[planner_server-3] [INFO] [1741791305.710505938] [global_costmap.global_costmap]: Cleaning up
[lifecycle_manager-10] [ERROR] [1741791305.809952260] [lifecycle_manager_navigation]: Failed to change state for node: planner_server
[lifecycle_manager-10] [ERROR] [1741791305.810098760] [lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.
[async_slam_toolbox_node-11] [INFO] [1741791308.925591808] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791308.644 for reason 'the timestamp on the message is earlier than all the data in the transform cache'
[async_slam_toolbox_node-11] [INFO] [1741791309.264695829] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791308.845 for reason 'the timestamp on the message is earlier than all the data in the transform cache'
[async_slam_toolbox_node-11] [INFO] [1741791309.432400465] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791309.045 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-11] [INFO] [1741791309.611890729] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791309.246 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-11] [INFO] [1741791309.729656015] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791309.447 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-11] Info: clipped range threshold to be within minimum and maximum range!
[async_slam_toolbox_node-11] Registering sensor: [Custom Described Lidar]
[async_slam_toolbox_node-11] [WARN] [1741791309.776038946] [slam_toolbox]: maximum laser range setting (10.0 m) exceeds the capabilities of the used Lidar (3.5 m)
[async_slam_toolbox_node-11] [INFO] [1741791404.166595782] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791403.963 for reason 'the timestamp on the message is earlier than all the data in the transform cache'
[async_slam_toolbox_node-11] [INFO] [1741791404.366830802] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791404.163 for reason 'the timestamp on the message is earlier than all the data in the transform cache'
[async_slam_toolbox_node-11] [INFO] [1741791404.572228449] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791404.364 for reason 'the timestamp on the message is earlier than all the data in the transform cache'
[async_slam_toolbox_node-11] [INFO] [1741791404.770362010] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791404.565 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-11] [INFO] [1741791404.970547822] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791404.765 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-11] [INFO] [1741791405.178047428] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791404.966 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-11] [INFO] [1741791405.402711081] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791405.167 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-11] [INFO] [1741791526.173251912] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791525.969 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-11] [INFO] [1741791526.373441070] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791526.169 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-11] [INFO] [1741791526.574928352] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791526.370 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-11] [INFO] [1741791526.780284214] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791526.571 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-11] [INFO] [1741791526.980212581] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791526.772 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-11] [INFO] [1741791527.179221490] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791526.973 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-11] [INFO] [1741791527.377994650] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791527.173 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-11] [INFO] [1741791527.578094267] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791527.374 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-11] [INFO] [1741791527.781224339] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791527.575 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-11] [INFO] [1741791527.979834291] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791527.776 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-11] [INFO] [1741791528.180291657] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791527.976 for reason 'discarding message because the queue is full'
[async_slam_toolbox_node-11] [INFO] [1741791528.382193147] [slam_toolbox]: Message Filter dropping message: frame 'laser' at time 1741791528.177 for reason 'discarding message because the queue is full'



[planner_server-3] [INFO] [1741801829.546356306] [global_costmap.global_costmap]: Using plugin "inflation_layer"
[planner_server-3] [INFO] [1741801829.549437723] [global_costmap.global_costmap]: Initialized plugin "inflation_layer"
[planner_server-3] [FATAL] [1741801829.961285918] [planner_server]: Failed to create global planner. Exception: According to the loaded plugin descriptions the class nav2_navfn_planner/NavfnPlanner with base class type nav2_core::GlobalPlanner does not exist. Declared types are  nav2_navfn_planner::NavfnPlanner nav2_smac_planner::SmacPlanner2D nav2_smac_planner::SmacPlannerHybrid nav2_smac_planner::SmacPlannerLattice nav2_theta_star_planner::ThetaStarPlanner
[planner_server-3] [INFO] [1741801829.964957085] [planner_server]: Cleaning up

