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
#include <OpenXrEigenConversions.h>


#include <cstdio>
#include <vector>
#include <array>
#include <cstring>
#include <cstdarg>
#include <atomic>
#include <unordered_map>

#include <yarp/os/LogStream.h>

#define NO_INTERACTION_PROFILE_TAG "no_interaction_profile"

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

struct InputActions
{
    std::vector<Action<bool>> buttons;

    std::vector<Action<float>> axes;

    std::vector<Action<Eigen::Vector2f>> thumbsticks;

    void clear();
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

    static void GLMessageCallback(GLenum /*source*/,
                                  GLenum type,
                                  GLuint /*id*/,
                                  GLenum severity,
                                  GLsizei /*length*/,
                                  const GLchar* message,
                                  const void* /*userParam*/);

    void submitLayer(const XrCompositionLayerBaseHeader* layer);

    OpenXrInterface::Pose getPose(const XrSpaceLocation& spaceLocation);

    OpenXrInterface::Velocity getVelocity(const XrSpaceVelocity& spaceVelocity);

    bool suggestInteractionProfileBindings(const std::string &interactionProfileName,
                                           const std::vector<XrActionSuggestedBinding> &poseBindings,
                                           std::initializer_list<std::pair<const char*, const char*>> buttonsList = {},
                                           std::initializer_list<std::pair<const char*, const char*>> axisList = {},
                                           std::initializer_list<std::pair<const char*, const char*>> thumbStickList = {});

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

    // state of the application
    XrSessionState state = XR_SESSION_STATE_UNKNOWN;

    // state of the frame during the rendering loop
    XrFrameState frame_state;

    // state of the eyes
    XrViewState view_state;

    // layer with rendered eyes projection
    XrCompositionLayerProjection projection_layer;

    // Set of head-locked quad layers added
    std::vector<std::shared_ptr<OpenXrQuadLayer>> headLockedQuadLayers;

    // Location of the head with respect to the play_space
    XrSpaceLocation view_space_location;

    //Head velocity
    XrSpaceVelocity view_space_velocity;

    // Top level path for the two hands. The first is the left
    XrPath hand_paths[2];

    // Set of actions used in the application
    XrActionSet actionset;

    // Action related to the hand poses
    XrAction hand_pose_action;

    // Poses can't be queried directly, we need to create a space for each
    XrSpace hand_pose_spaces[2];

    // Velocity of the hands
    XrSpaceVelocity hand_velocities[2];

    // Location of the hands
    XrSpaceLocation hand_locations[2];

    // Current interaction profile
    std::string currentHandInteractionProfile{NO_INTERACTION_PROFILE_TAG};

    // Map from the interaction profile to the actions
    std::unordered_map<std::string, InputActions> inputActions;

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
};


#endif // YARP_DEV_OPENXRINTERFACE_IMPLEMENTATION_H
