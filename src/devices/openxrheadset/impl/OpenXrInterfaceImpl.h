/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_OPENXRINTERFACE_IMPLEMENTATION_H
#define YARP_DEV_OPENXRINTERFACE_IMPLEMENTATION_H

#include <OpenXrConfig.h>
#include <OpenXrHeadsetLogComponent.h>
#include <OpenXrInterface.h>
#include <impl/OpenXrQuadLayer.h>
#include <impl/OpenGLQuadLayer.h>
#include <OpenXrEigenConversions.h>


#include <cstdio>
#include <vector>
#include <array>
#include <cstring>
#include <cstdarg>
#include <atomic>
#include <unordered_map>
#include <cmath>

#include <yarp/os/LogStream.h>

#define NO_INTERACTION_PROFILE_TAG "no_interaction_profile"
#define TOP_LEVEL_NOT_SUPPORTED_TAG "top_level_path_not_supported"
#define KHR_SIMPLE_CONTROLLER_INTERACTION_PROFILE "/interaction_profiles/khr/simple_controller"
#define OCULUS_TOUCH_INTERACTION_PROFILE_TAG "/interaction_profiles/oculus/touch_controller"
#define HTC_VIVE_INTERACTION_PROFILE_TAG "/interaction_profiles/htc/vive_controller"
#define HTC_VIVE_TRACKER_INTERACTION_PROFILE_TAG "/interaction_profiles/htc/vive_tracker_htcx"

// STRUCTS
template <typename ActionType>
struct Action {
    std::string name;

    XrAction xrAction;

    ActionType value;

    XrActionType type() const;

    XrResult create(XrActionSet actionSet, const std::string& inputName)
    {
        name = inputName;
        XrActionCreateInfo action_info = {.type = XR_TYPE_ACTION_CREATE_INFO,
                                          .next = NULL,
                                          .actionType = type(),
                                          .countSubactionPaths = 0,
                                          .subactionPaths = NULL};
        strcpy(action_info.actionName, name.c_str());
        strcpy(action_info.localizedActionName, name.c_str());

        return xrCreateAction(actionSet, &action_info, &xrAction);
    }

    XrResult update(XrSession session);
};

template<>
XrActionType Action<bool>::type() const;

template<>
XrActionType Action<float>::type() const;

template<>
XrActionType Action<Eigen::Vector2f>::type() const;

template<>
XrResult Action<bool>::update(XrSession session);

template<>
XrResult Action<float>::update(XrSession session);

template<>
XrResult Action<Eigen::Vector2f>::update(XrSession session);

struct PoseAction : public OpenXrInterface::NamedPoseVelocity
{
    XrAction xrAction;

    XrSpace xrSpace;

    XrResult create(XrSession session, XrActionSet actionSet, const std::string &inputName);

    XrResult update(XrSession session, XrSpace referenceSpace, XrTime time);
};


struct InputActions
{
    std::vector<PoseAction> poses;

    std::vector<Action<bool>> buttons;

    std::vector<Action<float>> axes;

    std::vector<Action<Eigen::Vector2f>> thumbsticks;

    void clear();
};

struct ActionDeclaration
{
    std::string path;
    std::string nameSuffix;
};

struct InputActionsDeclaration
{
    std::vector<ActionDeclaration> poses;

    std::vector<ActionDeclaration>  buttons;

    std::vector<ActionDeclaration>  axes;

    std::vector<ActionDeclaration>  thumbsticks;
};

struct InteractionProfileDeclaration
{
    std::string path;
    std::string actionNamePrefix;
};

struct TopLevelPath
{
    std::string stringPath;

    XrPath xrPath;

    std::string currentInteractionProfile{NO_INTERACTION_PROFILE_TAG};

    std::unordered_map<std::string, InputActions> interactionProfileActions;

    InputActions& currentActions();
};

struct TopLevelPathDeclaration
{
    std::string stringPath;

    std::string actionNamePrefix;

    std::unordered_map<std::string, InputActionsDeclaration> inputsDeclarations;
};

struct TrackerStatus
{
    bool connected{false};
    size_t top_level_index;
};

struct SwapChainData
{
    // the swapchain is a series of buffers that allows the application to render an image on a buffer differnt from the one in use
    XrSwapchain swapchain;

    // array of view_count array of swapchain_length containers holding an OpenGL texture
    // that is allocated by the runtime
    std::vector<XrSwapchainImageOpenGLKHR> swapchain_images;

    //Acquired index for the swapchain
    uint32_t acquired_index;
};

OpenXrInterface::Pose XrSpaceLocationToPose(const XrSpaceLocation& spaceLocation);

OpenXrInterface::Velocity XrSpaceVelocityToVelocity(const XrSpaceVelocity& spaceVelocity);


class OpenXrInterface::Implementation
{
public:
    //Helper methods

    bool checkXrOutput(XrResult result, const char* format, ...)
    {
        if (XR_SUCCEEDED(result))
            return true;

        if (instance)
        {
            char resultString[XR_MAX_RESULT_STRING_SIZE];
            xrResultToString(instance, result, resultString);

            char formatRes[XR_MAX_RESULT_STRING_SIZE + 1024];
            snprintf(formatRes, XR_MAX_RESULT_STRING_SIZE + 1023, "%s [%s]", format, resultString);

            char output[XR_MAX_RESULT_STRING_SIZE + 1024];

            va_list args;
            va_start(args, format);
            vsprintf(output, formatRes, args);
            va_end(args);

            yCError(OPENXRHEADSET) << output;
        }
        else
        {
            yCError(OPENXRHEADSET) << format;
        }

        return false;
    }

    static XrBool32 OpenXrDebugCallback(XrDebugUtilsMessageSeverityFlagsEXT severity,
                                        XrDebugUtilsMessageTypeFlagsEXT /*type*/,
                                        const XrDebugUtilsMessengerCallbackDataEXT* data,
                                        void* /*userData*/);

    static void glfwErrorCallback(int error, const char* description);

    static void GLMessageCallback(GLenum source,
                                  GLenum type,
                                  GLuint id,
                                  GLenum severity,
                                  GLsizei /*length*/,
                                  const GLchar* message,
                                  const void* /*userParam*/);

    void submitLayer(const XrCompositionLayerBaseHeader* layer);

    bool fillActionBindings(const std::vector<InteractionProfileDeclaration>& interactionProfilesPrefixes, const std::vector<TopLevelPathDeclaration>& topLevelPaths);

    std::string getInteractionProfileShortTag(const std::string& interactionProfile);

    std::string sessionStateToString(XrSessionState state);

    // DATA

    // the instance handle can be thought of as the basic connection to the OpenXR runtime
    XrInstance instance = XR_NULL_HANDLE;

    // the system represents an (opaque) set of XR devices in use, managed by the runtime
    XrSystemId system_id = XR_NULL_SYSTEM_ID;

    // the session deals with the renderloop submitting frames to the runtime
    XrSession session = XR_NULL_HANDLE;

    // the playspace is needed to define a reference frame for the application
    XrSpace play_space = XR_NULL_HANDLE;

    // the space fixed to the head of the operator
    XrSpace view_space = XR_NULL_HANDLE;

    // info about the views (the "eyes")
    std::vector<XrViewConfigurationView> viewconfig_views;

    // info to create the swapchain
    std::vector<XrSwapchainCreateInfo> projection_view_swapchain_create_info;

    // Swapchain for what the eyes see in the 3D world
    std::vector<SwapChainData> projection_view_swapchains;

    // Swapchain for the depth of what the eyes see in the 3D world
    std::vector<SwapChainData> projection_view_depth_swapchains;

    // containers for submitting swapchains with rendered VR frames
    std::vector<XrCompositionLayerProjectionView> projection_views;

    // containers for submitting swapchains with rendered VR depth frames
    std::vector<XrCompositionLayerDepthInfoKHR> depth_projection_views;

    // array of views, filled by the runtime with current HMD display pose (basically the position of each eye)
    std::vector<XrView> views;

    // position of a frame in the middle of the eyes, oriented as the first eye
    XrPosef mid_views_pose;

    // List of top level paths to retrieve the state of each action
    std::vector<TopLevelPath> top_level_paths;

    // flag to check if the HTC Vive trackers are supported by the runtime.
    bool htc_trackers_supported = false;

    // Pointer to function to get the list of trackers
    PFN_xrEnumerateViveTrackerPathsHTCX pfn_xrEnumerateViveTrackerPathsHTCX = NULL;

    // Vector of trackers connected
    std::vector<XrViveTrackerPathsHTCX> htc_trackers_connected;

    // Map defining which tracker is connected
    std::unordered_map<std::string, TrackerStatus> htc_trackers_status;

    // state of the application
    XrSessionState state = XR_SESSION_STATE_UNKNOWN;

    // state of the frame during the rendering loop
    XrFrameState frame_state;

    // state of the eyes
    XrViewState view_state;

    // time in which the reference space will change (for example when doing "Reset Seated Position")
    XrTime local_reference_space_change_time;

    // flag to determine if the reference space is about to change
    bool local_reference_space_changing{true};

    // time in which the poses are requested
    XrTime locate_space_time;

    // amount of prediction in nanoseconds to retrieve a pose (if negative it will add delay)
    long locate_space_prediction_in_ns{0};

    // layer with rendered eyes projection
    XrCompositionLayerProjection projection_layer;

    // Set of head-locked quad layers added
    std::vector<std::shared_ptr<OpenXrQuadLayer>> headLockedQuadLayers;

    std::vector <std::shared_ptr<OpenGLQuadLayer>> openGLQuadLayers;

    // Location of the head with respect to the play_space
    XrSpaceLocation view_space_location;

    //Head velocity
    XrSpaceVelocity view_space_velocity;

    // Set of actions used in the application
    XrActionSet actionset;

    // Buffer containing the submitted layers
    std::vector<const XrCompositionLayerBaseHeader*> submitted_layers;

    // The number of layers that have been added. This is lower or equal than the number of submitted layers
    size_t layer_count = 0;

    // Functions pointer to get OpenGL requirements
    PFN_xrGetOpenGLGraphicsRequirementsKHR pfnGetOpenGLGraphicsRequirementsKHR = NULL;

    // Struct storing OpenGL requirements
    XrGraphicsRequirementsOpenGLKHR opengl_reqs;

    // Each graphics API requires the use of a specialized struct
#ifdef XR_USE_PLATFORM_WIN32
    XrGraphicsBindingOpenGLWin32KHR graphics_binding_gl;
#else
    XrGraphicsBindingOpenGLXlibKHR graphics_binding_gl;
#endif

    // GLFW window to mirror what the user sees
    GLFWwindow* window = nullptr;

    // The window size
    std::vector<unsigned int> windowSize;

    // Internal OpenGL framebuffer ID
    GLuint glFrameBufferId = 0;

    std::atomic<bool> initialized{false};

    std::atomic<bool> closing{false};

    std::atomic<bool> closed{true};

    // Flag to enable the window visualization
    bool hideWindow{false};

    // Flag to enable the visualization of the layers in the play space instead of the head space
    bool renderInPlaySpace{false};
};


#endif // YARP_DEV_OPENXRINTERFACE_IMPLEMENTATION_H
