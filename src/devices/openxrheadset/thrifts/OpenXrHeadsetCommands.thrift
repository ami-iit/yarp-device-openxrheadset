/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

service OpenXrHeadsetCommands
{
    /**
    * Get the current interaction profile for the left hand
    * It returns a string that can be one between none, khr_simple_controller, oculus_touch_controller, htc_vive_controller, or htc_vive_focus3_controller
    * @return a string indicating the interaction profile in use.
    */
    string getLeftHandInteractionProfile();

    /**
    * Get the current interaction profile for the right hand
    * It returns a string that can be one between none, khr_simple_controller, oculus_touch_controller, htc_vive_controller, or htc_vive_focus3_controller
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
     * Get the left image rotation around its center
     * @return The left image rotation in radians, positive anticlockwise
     */
    double getLeftImageRotation();

    /**
     * Get the right image rotation around its center
     * @return The right image rotation in radians, positive anticlockwise
     */
    double getRightImageRotation();

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
     * Set the left image rotation around its center.
     * @param angle The rotation angle in radians (positive anticlockwise)
     * @return True if successfull
     */
    bool setLeftImageRotation(1:double angle);

    /**
     * Set the right image rotation around its center.
     * @param angle The rotation angle in radians (positive anticlockwise)
     * @return True if successfull
     */
    bool setRightImageRotation(1:double angle);

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
    * @return The distance in meters.
    */
    double getInterCameraDistance();

   /**
    * Set the lateral distance between the visualization of the robot cameras, in meters.
    * @param distance the lateral distance in meters.
    * @return True if successfull.
    */
    bool setInterCameraDistance(1:double distance);

    /**
    * Get the current IPD (Inter Pupillary Distance) of the VR eyes.
    * @return The IPD in meters
    */
    double getIPD();

    /**
    * Get the current drawable area percentage.
    * @return The drawable area percentage.
    */
    double getDrawableArea();

    /**
    * Set the drawable area percentage.
    * @param area The drawable area percentage to set (between 0.0 and 1.0).
    * @return True if successfull.
    */
    bool setDrawableArea(1:double area);

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
    * Set a GUI visible or not
    * @param GUIIndex The GUI index to change state
    * @param enabled The state to set
    * @return True if successfull, false if the index is out of bounds
    */
    bool setGUIEnabled(1:i32 GUIIndex, 2:bool enabled);

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

    /**
     * Set the position offset of a custom pose
     * @param customFrameName The name of the pose to edit
     * @param x The x coordinate of the relative position (with respect the parent frame)
     * @param y The y coordinate of the relative position (with respect the parent frame)
     * @param z The z coordinate of the relative position (with respect the parent frame)
     * @return True if successfull, false if the pose is not found
     */
     bool setCustomPoseRelativePosition(1:string customFrameName, 2:double x, 3:double y, 4:double z);

    /**
     * Set the rotation offset of a custom pose
     * The order depends on the chosen euler angles in the configuration file.
     * @param customFrameName The name of the pose to edit
     * @param angle1 The first angle offset (with respect the parent frame).
     * @param angle2 The second angle offset (with respect the parent frame).
     * @param angle3 The third angle offset (with respect the parent frame).
     * @return True if successfull, false if the pose is not found
     */
     bool setCustomPoseRelativeOrientation(1:string customFrameName, 2:double angle1, 3:double angle2, 4:double angle3);

    /**
     * Reset the transforms all the published tranforms.
     * This will also delete all the transforms currently stored in the transform server,
     * so also the static poses will be published again. This must be used with caution,
     * as it will reset all the transforms, including the ones that are not published by this module.
     */
     bool resetTransforms();

    /**
     * Start the joypad control server. The server will restart if already started.
     * @return True if the server is started successfully, false otherwise.
     */

     bool restartJoypadControlServer();

    /**
     * Check if the eye expressions are enabled
     * @return True if the eye expressions are enabled, false otherwise
     */
     bool eyeExpressionsEnabled();

    /**
     * Get the name of the port trough which it is possible to get the eye expressions.
     * @return the name of the port to get the eye expressions.
     */
     string getEyeExpressionsPortName();

    /**
     * Check if the lip expressions are enabled
     * @return True if the lip expressions are enabled, false otherwise
     */
     bool lipExpressionsEnabled();

    /**
     * Get the name of the port trough which it is possible to get the lip expressions.
     * @return the name of the port to get the lip expressions.
     */
     string getLipExpressionsPortName();

    /**
     * Check if the gaze acquisition is enabled
     * @return True if the gaze acquisition is enabled, false otherwise
     */
     bool gazeEnabled();

    /**
     * Get the name of the port trough which it is possible to get the gaze position.
     * @return the name of the port to get the gaze position.
     */
     string getGazePortName();
}
