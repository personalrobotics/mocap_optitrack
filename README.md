1. Edit `config/mocap.yaml`
2. Broadcast necessary tf `rosrun tf static_transform_publisher 0 0 0 0 0 1.5708 map optitrack_natnet 1000`
3. Launch streaming: `roslaunch mocap_optitrack mocap.launch`