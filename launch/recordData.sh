#/bin/bash
rosbag record \
       /band/contact \
       /band/gsr \
       /band/heartrate \
       /band/imu \
       /band/light \
       /band/temp \
       /control_node/grasp_task \
       /control_node/nav_task \
       /control_node/sample_task \
       /control_node/path_task \
       /control_node/workload_task \
       /experiment_ui/attention \
       /experiment_ui/feedback \
       /experiment_ui/progress \
       /experiment_ui/telemetry \
       /experiment_ui/user_rating \
       /experiment_ui/tlx \
       /robot0/cmd_vel \
       /robot0/odom_throttle
