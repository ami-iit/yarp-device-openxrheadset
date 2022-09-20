To robsub:  
mamba activate robsub  
call robotology-superbuild\build\install\share\robotology-superbuild\setup.bat

To edit code:  
C:\Users\icub\robotology-superbuild\build\src\yarp-device-openxrheadset>start ALL_BUILD.vcxproj

After editing:  
build --> ALL_BUILD  
build --> INSTALL

To push commits:  
(robsub) C:\Users\icub\robotology-superbuild\src\yarp-device-openxrheadset>git branch  
* main  
* opengl_eye_layers <--

To check the effect of SW mods on the headset:  
Run VIVE Wireless  
Run STEAMVR  
Enable Display VR View (right click on STEAMVR window)

Run Terminal T1 (robsub):  
check YARP namespace != /root  
yarp namespace  
/glocale  
If necessary, change yarp namespace (/root is used by everyone. For local tests use something different)

T1 (robsub):  
yarpserver

T2 (robsub): transform server  
yarpdev --device transformServer --ROS::enable_ros_publisher 0 --ROS::enable_ros_subscriber 0

T3 (robsub): what to stop and restart every time you do a modification (relative paths are resolved according to where this is launched from)  
yarpdev --from openXRHeadsetParameters.ini

T4 (robsub): Fake camera  
yarpdev --device fakeFrameGrabber --name /grabber

T5 (robsub):  
yarp connect /grabber /joypadDevice/Oculus/display/left:i mjpeg  
yarp connect /grabber /joypadDevice/Oculus/display/right:i mjpeg
