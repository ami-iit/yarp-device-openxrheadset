# yarp-device-openxrheadset

``yarp-device-openxrheadset`` is a [``YARP``](https://github.com/robotology/yarp) device allowing to use VR devices through [``OpenXR``](https://www.khronos.org/openxr/).

It can be used, for example, to display the images coming from a robot camera in VR, while retrieving the position and orientation of the headset and the joysticks used by the operator.

# Installation
## Windows

## Linux

# Running

# Reference frames

    //The grip position:
    //  For tracked hands: The user’s palm centroid when closing the fist, at the surface of the palm.
    //  For handheld motion controllers: A fixed position within the controller that generally lines up
    //  with the palm centroid when held by a hand in a neutral position.
    //  This position should be adjusted left or right to center the position within the controller’s grip.

    //The grip orientation:
    //  +X axis: When you completely open your hand to form a flat 5-finger pose, the ray that is normal to the user’s
    //           palm (away from the palm in the left hand, into the palm in the right hand).
    //  -Z axis: When you close your hand partially (as if holding the controller), the ray that goes through the center
    //           of the tube formed by your non-thumb fingers, in the direction of little finger to thumb.
    //  +Y axis: orthogonal to +Z and +X using the right-hand rule.

# Visualizer

# Trackers

# Inputs order

## Left hand
### ``/interaction_profiles/khr/simple_controller``
Buttons:
- "/input/select/click"
- "/input/menu/click"

### ``/interaction_profiles/oculus/touch_controller``
Buttons:
- "/input/menu/click"
- "/input/x/click"
- "/input/y/click"
- "/input/thumbstick/click"

Axes:
- /input/trigger/value
- /input/squeeze/value

Thumbsticks:
- /input/thumbstick

### ``/interaction_profiles/htc/vive_controller``
Buttons:
- /input/menu/click
- /input/trigger/click
- /input/squeeze/click
- /input/trackpad/click

Axes:
- /input/trigger/value

Thumbsticks:
- /input/trackpad

## Right hand
### ``/interaction_profiles/khr/simple_controller``
Buttons:
- "/input/select/click"
- "/input/menu/click"

### ``/interaction_profiles/oculus/touch_controller``
Buttons:
- "/input/a/click"
- "/input/b/click"
- "/input/thumbstick/click"

Axes:
- /input/trigger/value
- /input/squeeze/value

Thumbsticks:
- /input/thumbstick

### ``/interaction_profiles/htc/vive_controller``
Buttons:
- /input/menu/click
- /input/trigger/click
- /input/squeeze/click
- /input/trackpad/click

Axes:
- /input/trigger/value

Thumbsticks:
- /input/trackpad

## Trackers
Roles
- /user/vive_tracker_htcx/role/handheld_object",
- /user/vive_tracker_htcx/role/left_foot
- /user/vive_tracker_htcx/role/right_foot
- /user/vive_tracker_htcx/role/left_shoulder
- /user/vive_tracker_htcx/role/right_shoulder
- /user/vive_tracker_htcx/role/left_elbow
- /user/vive_tracker_htcx/role/right_elbow
- /user/vive_tracker_htcx/role/left_knee
- /user/vive_tracker_htcx/role/right_knee
- /user/vive_tracker_htcx/role/waist
- /user/vive_tracker_htcx/role/chest
- /user/vive_tracker_htcx/role/camera
- /user/vive_tracker_htcx/role/keyboard

Buttons:
- /input/menu/click
- /input/trigger/click
- /input/squeeze/click
- /input/trackpad/click

Axes:
- /input/trigger/value

Thumbsticks:
- /input/trackpad

# Configuration Parameters

- name
- gui_elements
    - width
    - height
    - x
    - y
    - z
    - opens port with name ``/name/gui_*``

- labels
    - width
    - height
    - x
    - y
    - z
    - opens port with name ``/name/label_*``
    - optional
    - prefix (def "")
    - suffix (def "")
    - font (either absolute, or relative to https://github.com/ami-iit/GLFont/tree/64a02d6dc382e2bc052e90b15d80cc9792d689c5/fonts)
    - pixel_size (def 64)
    - automatically_enabled (def true)
    - ``disable_timeout_in_S`` (def -1)
    - horizontal_alignement (allowed entries ``left``, ``right``, ``center``, def ``center``)
    - vertical_alignement (allowed entries ``top``, ``bottom``, ``center``, def ``center``)
    - color (def ``(1.0, 1.0, 1.0, 1.0)``)
    - background_color (def ``(0.0, 0.0, 0.0, 0.0)``)

- ``stick_as_axis`` (def false)
- ``tf_left_hand_frame`` (def ``openxr_left_hand``)
- ``tf_right_hand_frame`` (def ``openxr_right_hand``)
- ``tf_head_frame`` (def ``openxr_head``)
- ``tf_left_eye_frame`` (def ``openxr_left_eye``)
- ``tf_right_eye_frame`` (def ``openxr_right_eye``)
- ``tf_root_frame`` (def ``openxr_origin``)
- ``left_azimuth_offset`` (def 0.0)
- ``left_elevation_offset`` (def 0.0)
- ``eye_z_position`` (def -1.0)
- ``inter_camera_distance`` (def 0.07)
- ``right_azimuth_offset``(def 0.0)
- ``right_elevation_offset`` (def 0.0)
- ``split_eye_ports`` (def true)
- ``tfDevice`` (def ``transformClient``)
- ``tfLocal`` (def ``/name/tf``)
- ``tfRemote`` (def ``/transformServer``)
- ``[POSES_LABELS]``

# Ports opened
- ``/name/gui_*`` for each GUI
- ``/name/label_*`` for each label
- ``/name/eyeAngles/left:i``
- ``/name/eyeAngles/right:i``
- ``/name/display/left:i``
- ``/name/display/right:i``
- ``/name/display:i``
- ``/name/rpc``

# RPC interface


    * Get the current interaction profile for the left hand
    * It returns a string that can be one between none, khr_simple_controller, oculus_touch_controller or htc_vive_controller
    * @return a string indicating the interaction profile in use.
    */
    string getLeftHandInteractionProfile();

    /**
    * Get the current interaction profile for the right hand
    * It returns a string that can be one between none, khr_simple_controller, oculus_touch_controller or htc_vive_controller
    * @return a string indicating the interaction profile in use.
    */
    string getRightHandInteractionProfile();

    /**
     * Get the left image width and height.
     * @return A vector of two elements with the left image width and height in meters, in this order
     */
    list<double> getLeftImageDimensions();

    /**
     * Get the right image width and height.
     * @return A vector of two elements with the right image width and height in meters, in this order
     */
    list<double> getRightImageDimensions();

    /**
     * Get the left image azimuth (positive anticlockwise) and elevation (positive upwards) offsets in radians
     * @return A vector of two elements with the left image azimuth and elevation offsets in radians, in this order
     */
    list<double> getLeftImageAnglesOffsets();

    /**
     * Get the right image azimuth (positive anticlockwise) and elevation (positive upwards) offsets in radians
     * @return A vector of two elements with the left image azimuth and elevation offsets in radians, in this order
     */
    list<double> getRightImageAnglesOffsets();

    /**
     * Set the left image azimuth (positive anticlockwise) and elevation (positive upwards) offsets in radians
     * @param azimuth The azimuth angle offset in radians (positive anticlockwise)
     * @param elevation The elevation angle offset in radians (positive upwards)
     * @return True if successfull
     */
    bool setLeftImageAnglesOffsets(1:double azimuth, 2:double elevation);

    /**
     * Set the right image azimuth (positive anticlockwise) and elevation (positive upwards) offsets in radians
     * @param azimuth The azimuth angle offset in radians (positive anticlockwise)
     * @param elevation The elevation angle offset in radians (positive upwards)
     * @return True if successfull
     */
    bool setRightImageAnglesOffsets(1:double azimuth, 2:double elevation);

    /**
     * Check if the left eye is visualizing images.
     * @return True if the left eye is active. False otherwise
     */
    bool isLeftEyeActive();

    /**
     * Check if the right eye is visualizing images.
     * @return True if the right eye is active. False otherwise
     */
    bool isRightEyeActive();

   /**
    * Get the current Z position (i.e. the location on the axis perpendicular to the screens) of the eyes visualization
    * @return The (signed) value of the eyes Z position. The Z axis is pointing backward, hence this value will be negative.
    */
    double getEyesZPosition();

    /**
     * Set the Z position (i.e. the location on the axis perpendicular to the screens) of the eyes visualization
     * @return True if successfull, false otherwise
     */
    bool setEyesZPosition(1: double eyesZPosition);

   /**
    * Get the current lateral distance between the visualization of the robot cameras.
    * @return The IPD in meters.
    */
    double getInterCameraDistance();

   /**
    * Set the lateral distance between the visualization of the robot cameras, in meters.
    * @param distance the lateral distance in meters.
    * @return True if successfull.
    */
    bool setInterCameraDistance(1:double distance);

   /**
    * Get the name of the port trough which it is possible to control the left image.
    * @return the name of the port to control the left image.
    */
    string getLeftImageControlPortName();

   /**
    * Get the name of the port trough which it is possible to control the right image.
    * @return the name of the port to control the right image.
    */
    string getRightImageControlPortName();

   /**
    * Set a label visible or not
    * @param labelIndex The label index to change state
    * @param enabled The state to set
    * @return True if successfull, false if the index is out of bounds
    */
    bool setLabelEnabled(1:i32 labelIndex, 2:bool enabled);

    /**
     * Align the root frame to the current headset position and angle around gravity
     * @return True if successfull. False otherwise, e.g. if the current headset pose is not valid
     */
     bool alignRootFrameToHeadset();

