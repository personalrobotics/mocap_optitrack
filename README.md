# mocap_optitrack fork

This repo builds on top of `mocap_optitrack` library. It adds support for
- Tracking unlabeled markers based on a naive assignment + filtering method.
- Publishing model markers (that belong to rigid body) along side unlabeled markers

# Calibrating the Mocap System (Wanding)
1. Remove or cover anything in the workspace which appears on the cameras.
2. Click “Start Wanding” in the Camera Calibration pane under Calibration

	![](figures/start-wanding.png)

3. Move the wand around the workspace, focusing on specific cameras as needed so each camera has at least a thousand samples.

	![](figures/camera-samples.png)

4. After there is at least a thousand samples for each camera, click “Calculate”
5. The Calibration Result should be excellent or exceptional (exceptional is better). Click “Apply” in the Calibration Result pop-up window.
6. Make sure the three ground plane points are visible to the cameras. In the Camera Calibration window, under Ground Plane, click “Set Ground Plane”. It will prompt you to save, and you can use the default file name.
	
    ![](figures/ground-plane.png)

# Stream Mocap Data

1. In Optitrack software: Click View>DataStreaming
2. In the new Streaming window, make sure the settings match those in the pictures below:

<p align="center">
    <img width="300" src="figures/motive-streaming-1.png">
    <img width="300"src="figures/motive-streaming-2.png">
</p>

3. For the "Network Interface / Local Interface" field write the machine’s IP address. For the “Multicast Interface”, match its value with `multicast_address` in [`config/mocap.yaml`](config/mocap.yaml)

4. Edit [`config/mocap.yaml`](config/mocap.yaml) so that points `T3` and `T4` reflect the starting points of the chopstick with two trackers, and `T5` (or `Ball`) reflects the position of the tracker to pick up.

	![](figures/labeled-trackers.png) 

5. In the Optitrack software, select the three markers on the end of the arm (in the same order as they appear in [`config/mocap.yaml`](config/mocap.yaml)). Right-click, and select “Rigid Body > Create from Selected Markers”
6. Repeat the previous step for the three-marker chopstick (it’s important to mark the arm tracker rigid body first, before the chopstick)
7. The software transforms point poses from optitrack frame to map frame in `mocap_datapackets.cpp`, `get_3d_point()`. Currently we do not modify the transformation. Instead. we rely on an explicit calibration (hebi_calibration) to find the correct transformation. The client side will apply the transformation as needed.
8. Optionally, you can remove the transformation in `get_3d_point()` and broadcast tf for any transform you want: e.g. `rosrun tf static_transform_publisher 0 0 0 0 0 1.5708 map optitrack_natnet 1000`
9. Launch streaming: `roslaunch mocap_optitrack mocap.launch`

## Tips

1. Tweak `trackMarkers` function defined in `mocap_node`, especially the constant number. This might improve the tracking of unlabeled markers when facing marker occlusion / marker went too fast.
