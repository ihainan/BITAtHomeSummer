# bitathome_vision_control

---

## Package summery
This package is used to control vision devices of bit@home robot.

- Status : maintaining
- Maintainer : Alan Snape (for now)

## Nodes
- KinectVision : uses Kinect device to get RGBD data, uses openni_tracker to get skeleton data
    - Dependence : cv_bridge, image_geometry, image_transport, image_view, tf, roscpp, std_msgs 

## Launch files
- vision.launch : 
    - launch : 
        openni_tracker/openni_tracker, 
        bitathome_vision_control/KinectVision    
- debug.launch : 
    - launch :
        openni_launch/openni.launch
        bitathome_vision
- face_recognition.launch :
    - launch :
        openni_launch/openni.launch
        face_recognition/Fserver
        face_recognition/Fclient
