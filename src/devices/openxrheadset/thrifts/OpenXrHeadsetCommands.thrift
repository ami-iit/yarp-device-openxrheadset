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
}
