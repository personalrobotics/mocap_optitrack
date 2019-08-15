1. Edit `config/mocap.yaml`
2. For point tracking, the software transforms point poses from optitrack frame to map frame in `mocap_datapackets.cpp`, `get_3d_point()`.
3. Optionally, you can remove the transformation in `get_3d_point()` and broadcast tf for any transform you want: e.g. `rosrun tf static_transform_publisher 0 0 0 0 0 1.5708 map optitrack_natnet 1000`
3. Launch streaming: `roslaunch mocap_optitrack mocap.launch`