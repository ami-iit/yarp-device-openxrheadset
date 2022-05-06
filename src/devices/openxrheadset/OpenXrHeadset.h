/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_OPENXRHEADSET_H
#define YARP_DEV_OPENXRHEADSET_H

#include <vector>
#include <atomic>
#include <mutex>
#include <unordered_map>

#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/IJoypadController.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ServiceInterfaces.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/BufferedPort.h>
#include <OpenXrInterface.h>
#include <ImagePortToQuadLayer.h>
#include <LabelPortToQuadLayer.h>
#include <EyesManager.h>
#include <FramePorts.h>
#include <AdditionalPosesPublisher.h>
#include <thrifts/OpenXrHeadsetCommands.h>

#include <Eigen/Core>

namespace yarp {
namespace dev {

class OpenXrHeadset;

}
}

class yarp::dev::OpenXrHeadset : public yarp::dev::DeviceDriver,
                                 public yarp::os::PeriodicThread,
                                 public yarp::dev::IService,
                                 public yarp::dev::IJoypadController,
                                 public OpenXrHeadsetCommands
{
public:
    OpenXrHeadset();

    virtual ~OpenXrHeadset();

    // yarp::dev::DeviceDriver methods
    virtual bool open(yarp::os::Searchable& cfg) override;
    virtual bool close() override;

    // yarp::os::RateThread methods
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;

    //  yarp::dev::IService methods
    virtual bool startService() override;
    virtual bool updateService() override;
    virtual bool stopService() override;

    // yarp::dev::IJoypadController methods
    virtual bool getAxisCount(unsigned int& axis_count) override;
    virtual bool getButtonCount(unsigned int& button_count) override;
    virtual bool getTrackballCount(unsigned int& trackball_count) override;
    virtual bool getHatCount(unsigned int& hat_count) override;
    virtual bool getTouchSurfaceCount(unsigned int& touch_count) override;
    virtual bool getStickCount(unsigned int& stick_count) override;
    virtual bool getStickDoF(unsigned int stick_id, unsigned int& dof) override;
    virtual bool getButton(unsigned int button_id, float& value) override;
    virtual bool getTrackball(unsigned int trackball_id, yarp::sig::Vector& value) override;
    virtual bool getHat(unsigned int hat_id, unsigned char& value) override;
    virtual bool getAxis(unsigned int axis_id, double& value) override;
    virtual bool getStick(unsigned int stick_id, yarp::sig::Vector& value, JoypadCtrl_coordinateMode coordinate_mode) override;
    virtual bool getTouch(unsigned int touch_id, yarp::sig::Vector& value) override;

    //OpenXrHeadsetCommands
    /**
     * Get the current interaction profile for the left hand
     * It returns a string that can be one between none, khr_simple_controller, oculus_touch_controller or htc_vive_controller
     * @return a string indicating the interaction profile in use.
     */
    virtual std::string getLeftHandInteractionProfile() override;

    /**
     * Get the current interaction profile for the right hand
     * It returns a string that can be one between none, khr_simple_controller, oculus_touch_controller or htc_vive_controller
     * @return a string indicating the interaction profile in use.
     */
    virtual std::string getRightHandInteractionProfile() override;

    /**
     * Get the left image width and height.
     * @return A vector of two elements with the left image width and height, in this order
     */
    virtual std::vector<double> getLeftImageDimensions() override;

    /**
     * Get the right image width and height.
     * @return A vector of two elements with the right image width and height, in this order
     */
    virtual std::vector<double> getRightImageDimensions() override;

    /**
     * Get the left image azimuth (positive anticlockwise) and elevation (positive upwards) offsets in radians
     * @return A vector of two elements with the left image azimuth and elevation offsets in radians, in this order
     */
    virtual std::vector<double> getLeftImageAnglesOffsets() override;

    /**
     * Get the right image azimuth (positive anticlockwise) and elevation (positive upwards) offsets in radians
     * @return A vector of two elements with the left image azimuth and elevation offsets in radians, in this order
     */
    virtual std::vector<double> getRightImageAnglesOffsets() override;

    /**
     * Set the left image azimuth (positive anticlockwise) and elevation (positive upwards) offsets in radians
     * @param azimuth The azimuth angle offset in radians (positive anticlockwise)
     * @param elevation The elevation angle offset in radians (positive upwards)
     * @return True if successfull
     */
    virtual bool setLeftImageAnglesOffsets(const double azimuth, const double elevation) override;

    /**
     * Set the right image azimuth (positive anticlockwise) and elevation (positive upwards) offsets in radians
     * @param azimuth The azimuth angle offset in radians (positive anticlockwise)
     * @param elevation The elevation angle offset in radians (positive upwards)
     * @return True if successfull
     */
    virtual bool setRightImageAnglesOffsets(const double azimuth, const double elevation) override;

    /**
     * Check if the left eye is visualizing images.
     * @return True if the left eye is active. False otherwise
     */
    virtual bool isLeftEyeActive() override;

    /**
     * Check if the right eye is visualizing images.
     * @return True if the right eye is active. False otherwise
     */
    virtual bool isRightEyeActive() override;

    /**
     * Get the current Z position (i.e. the location on the axis perpendicular to the screens) of the eyes visualization
     * @return The (signed) value of the eyes Z position. The Z axis is pointing backward, hence this value will be negative.
     */
    virtual double getEyesZPosition() override;

    /**
     * Set the Z position (i.e. the location on the axis perpendicular to the screens) of the eyes visualization
     * @return True if successfull, false otherwise
     */
    virtual bool setEyesZPosition(const double eyesZPosition) override;

    /**
     * Get the current lateral distance between the visualization of the robot cameras.
     * @return The IPD in meters.
     */
    virtual double getInterCameraDistance() override;

    /**
     * Set the lateral distance between the visualization of the robot cameras, in meters.
     * @param distance the lateral distance in meters.
     * @return True if successfull.
     */
    virtual bool setInterCameraDistance(const double distance) override;

    /**
     * Get the name of the port trough which it is possible to control the left image.
     * @return the name of the port to control the left image.
     */
    virtual std::string getLeftImageControlPortName() override;

    /**
     * Get the name of the port trough which it is possible to control the right image.
     * @return the name of the port to control the right image.
     */
    virtual std::string getRightImageControlPortName() override;

    /**
     * Set a label visible or not
     * @param labelIndex The label index to change state
     * @param elevation The state to set
     * @return True if successfull, false if the index is out of bounds
     */
    virtual bool setLabelEnabled(const std::int32_t labelIndex, const bool enabled) override;

private:

    struct GuiParam
    {
        float         width;
        float         height;
        float         x;
        float         y;
        float         z;
        std::string    portName;
        ImagePortToQuadLayer<yarp::sig::ImageOf<yarp::sig::PixelRgba>> layer;
    };

    struct LabelLayer
    {
        float         width;
        float         height;
        float         x;
        float         y;
        float         z;
        LabelPortToQuadLayer::Options options;
        LabelPortToQuadLayer layer;
    };

    FramePorts m_headFramePorts;
    FramePorts m_leftHandFramePorts;
    FramePorts m_rightHandFramePorts;

    AdditionalPosesPublisher m_additionalPosesPublisher;

    yarp::os::Stamp m_stamp;

    std::string m_prefix;

    EyesManager m_eyesManager;

    std::vector<GuiParam> m_huds;
    std::vector<LabelLayer> m_labels;

    bool m_getStickAsAxis;

    IFrameTransform* m_tfPublisher;
    std::string      m_leftFrame;
    std::string      m_rightFrame;
    std::string      m_headFrame;
    std::string      m_rootFrame;
    PolyDriver       m_driver;

    yarp::os::Port m_rpcPort;

    std::atomic_bool m_closed{ false };

    OpenXrInterface m_openXrInterface;

    std::vector<bool> m_buttons;
    std::vector<float> m_axes;
    std::vector<Eigen::Vector2f> m_thumbsticks;

    std::mutex m_mutex;

};

#endif // YARP_DEV_OPENXRHEADSET_H
