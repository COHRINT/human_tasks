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
       /control_node/workload_task \
       /experiment_ui/attention \
       /experiment_ui/feedback \
       /experiment_ui/progress \
       /experiment_ui/telemetry \
       /robot0/cmd_vel \
       /robot0/odom_throttle
