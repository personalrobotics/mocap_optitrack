
# mocap_optitrack fork

This repo builds on top of `mocap_optitrack` library. It adds support for
- Tracking unlabeled markers based on a naive assignment + filtering method.
- Publishing model markers (that belong to rigid body) along side unlabeled markers

# Usage

0. In Optitrack software:
	1. Turn on Optitrack Streaming via Natnet. For the local interface field, write the machine's IP address. For further instructions, check out [Optitrack Motive wiki](https://v21.wiki.optitrack.com/index.php?title=Data_Streaming)
	![](https://v21.wiki.optitrack.com/images/thumb/1/11/DataStreaming_Pane_21.png/400px-DataStreaming_Pane_21.png)
	2. Check the option to stream Rigid Body + Model Markers + Unlabeled Markers.
1. Edit `config/mocap.yaml` for rigid body and markers.
2. The software transforms point poses from optitrack frame to map frame in `mocap_datapackets.cpp`, `get_3d_point()`.
3. Optionally, you can remove the transformation in `get_3d_point()` and broadcast tf for any transform you want: e.g. `rosrun tf static_transform_publisher 0 0 0 0 0 1.5708 map optitrack_natnet 1000`
4. Launch streaming: `roslaunch mocap_optitrack mocap.launch`

# Tips

1. Tweak `trackMarkers` function defined in `mocap_node`, especially the constant number. This might improve the tracking of unlabeled markers when facing marker occlusion / marker went too fast.
