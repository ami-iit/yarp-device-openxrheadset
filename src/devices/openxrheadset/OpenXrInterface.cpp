/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include "OpenXrConfig.h"

#if defined(_WIN32)
 #define GLFW_EXPOSE_NATIVE_WIN32
 #define GLFW_EXPOSE_NATIVE_WGL
#elif defined(__APPLE__)
 #define GLFW_EXPOSE_NATIVE_COCOA
 #define GLFW_EXPOSE_NATIVE_NSGL
#elif defined(__linux__)
 #define GLFW_EXPOSE_NATIVE_X11
 #define GLFW_EXPOSE_NATIVE_GLX
#endif

#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#include <cstdio>
#include <vector>
#include <array>
#include <cstring>
#include <cstdarg>
#include <atomic>

#include <yarp/os/LogStream.h>

#include "OpenXrHeadsetLogComponent.h"

#include "OpenXrInterface.h"
#include "OpenXrQuadLayer.h"

class OpenXrInterface::Implementation
{
public:

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
                                        void* /*userData*/) {
        yCTrace(OPENXRHEADSET);

        if (severity & XR_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT)
        {
            yCError(OPENXRHEADSET) << data->functionName + std::string(": ") + data->message;
        }
        else if (severity & XR_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT)
        {
            yCWarning(OPENXRHEADSET) << data->functionName + std::string(": ") + data->message;
        }
        else if (severity & XR_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT)
        {
            yCInfo(OPENXRHEADSET) << data->functionName + std::string(": ") + data->message;
        }
        else if (severity & XR_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT)
        {
            yCDebug(OPENXRHEADSET) << data->functionName + std::string(": ") + data->message;
        }

        return XR_TRUE;
    }

    static void glfwErrorCallback(int error, const char* description)
    {
        yCError(OPENXRHEADSET) << "GLFW Error:" << error << description;
    }

    static void GLMessageCallback(GLenum /*source*/,
                                  GLenum type,
                                  GLuint /*id*/,
                                  GLenum severity,
                                  GLsizei /*length*/,
                                  const GLchar* message,
                                  const void* /*userParam*/) {
        yCError(OPENXRHEADSET, "GL CALLBACK: %s type = 0x%x, severity = 0x%x, message = %s",
                ( type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : "" ),
                type, severity, message );
    }

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
    XrSwapchainCreateInfo swapchain_create_info;
    // the swapchain is a series of buffers that allows the application to render an image on a buffer differnt from the one in use
    XrSwapchain swapchain;
    // array of view_count array of swapchain_length containers holding an OpenGL texture
    // that is allocated by the runtime
    std::vector<XrSwapchainImageOpenGLKHR> swapchain_images;
    // a separate swapchain only for the depth buffer
    XrSwapchain depth_swapchain;
    // imges for the depth swapchain
    std::vector<XrSwapchainImageOpenGLKHR> depth_swapchain_images;
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
    //Set of head-locked quad layers added
    std::vector<std::shared_ptr<OpenXrQuadLayer>> headLockedQuadLayers;
    //Buffer containing the submitted layers
    std::vector<const XrCompositionLayerBaseHeader*> submitted_layers;
    //The number of layers that have been added. This is lower or equal than the number of submitted layers
    size_t layer_count = 0;

    // functions pointer to get OpenGL requirements
    PFN_xrGetOpenGLGraphicsRequirementsKHR pfnGetOpenGLGraphicsRequirementsKHR = NULL;
    // Struct storing OpenGL requirements
    XrGraphicsRequirementsOpenGLKHR opengl_reqs;

    // each graphics API requires the use of a specialized struct
#ifdef XR_USE_PLATFORM_WIN32
    XrGraphicsBindingOpenGLWin32KHR graphics_binding_gl;
#else
    XrGraphicsBindingOpenGLXlibKHR graphics_binding_gl;
#endif

    // GLFW window to mirror what the user sees
    GLFWwindow* window = nullptr;
    // The window size
    std::vector<unsigned int> windowSize;

    // internal OpenGL framebuffer ID
    GLuint glFrameBufferId = 0;

    std::atomic<bool> initialized{false};

    std::atomic<bool> closing{false};

    std::function<void (int, int, int, int)> keyCallback = [this](int key, int /*scancode*/, int action, int /*mods*/)
    {
        if (GLFW_PRESS != action) {
            return;
        }

        if (key == GLFW_KEY_ESCAPE)
        {
            closing = true;
        }
    };

    static void glfwKeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
    {
        Implementation* instance = (Implementation*)glfwGetWindowUserPointer(window);
        instance->keyCallback(key, scancode, action, mods);
    }

    void submitLayer(const XrCompositionLayerBaseHeader* layer)
    {
        layer_count++;
        if (layer_count > submitted_layers.size())
        {
            submitted_layers.resize(layer_count);
        }
        submitted_layers[layer_count-1] = layer;
    }
};

bool OpenXrInterface::checkExtensions()
{
    yCTrace(OPENXRHEADSET);
    // reuse this variable for all our OpenXR return codes
    XrResult result = XR_SUCCESS;

    // xrEnumerate*() functions are usually called once with CapacityInput = 0.
    // The function will write the required amount into CountOutput. We then have
    // to allocate an array to hold CountOutput elements and call the function
    // with CountOutput as CapacityInput.
    uint32_t ext_count = 0;
    result = xrEnumerateInstanceExtensionProperties(NULL, 0, &ext_count, NULL);

    /* TODO: instance null will not be able to convert XrResult to string */
    if (!m_pimpl->checkXrOutput(result, "Failed to enumerate number of extension properties"))
        return false;

    XrExtensionProperties defaultProperty;
    defaultProperty.type = XR_TYPE_EXTENSION_PROPERTIES;
    defaultProperty.next = nullptr;
    std::vector<XrExtensionProperties> ext_props(ext_count, defaultProperty);

    result = xrEnumerateInstanceExtensionProperties(NULL, ext_count, &ext_count, ext_props.data());
    if (!m_pimpl->checkXrOutput(result, "Failed to enumerate extension properties"))
        return false;

    bool opengl_supported = false;
    bool depth_supported = false;
    bool debug_supported = false;

    for (uint32_t i = 0; i < ext_count; i++) {
        if (strcmp(XR_KHR_OPENGL_ENABLE_EXTENSION_NAME, ext_props[i].extensionName) == 0) {
            opengl_supported = true;
        }

        if (strcmp(XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME, ext_props[i].extensionName) == 0) {
            depth_supported = true;
        }

        if (strcmp(XR_EXT_DEBUG_UTILS_EXTENSION_NAME, ext_props[i].extensionName) == 0) {
            debug_supported = true;
        }
    }

    // A graphics extension like OpenGL is required to draw anything in VR
    if (!opengl_supported) {
        yCError(OPENXRHEADSET) << "Runtime does not support OpenGL extension!";
        return false;
    }

    if (!depth_supported) {
        yCError(OPENXRHEADSET) << "Runtime does not support the depth buffer!";
        return false;
    }

    if (!debug_supported) {
        yCError(OPENXRHEADSET) << "Runtime does not support debug calls!";
        return false;
    }

    return true;
}

void OpenXrInterface::printInstanceProperties()
{
    yCTrace(OPENXRHEADSET);

    XrResult result;
    XrInstanceProperties instance_props;
    instance_props.type = XR_TYPE_INSTANCE_PROPERTIES;
    instance_props.next = NULL;

    result = xrGetInstanceProperties(m_pimpl->instance, &instance_props);
    if (!XR_SUCCEEDED(result))
    {
        yCDebug(OPENXRHEADSET) << "Failed to get instance properties";
        return;
    }

    yCInfo(OPENXRHEADSET) << "Runtime Name:" << instance_props.runtimeName;
    yCInfo(OPENXRHEADSET) << "Runtime Version:" << std::to_string(XR_VERSION_MAJOR(instance_props.runtimeVersion)) + "." +
                                                   std::to_string(XR_VERSION_MINOR(instance_props.runtimeVersion)) + "." +
                                                   std::to_string(XR_VERSION_PATCH(instance_props.runtimeVersion));
}

bool OpenXrInterface::prepareXrInstance()
{
    yCTrace(OPENXRHEADSET);

    if (!checkExtensions())
    {
        return false;
    }

    // List the requested extensions to the runtime
    std::vector<const char*> requestedExtensions;
    requestedExtensions.push_back(XR_KHR_OPENGL_ENABLE_EXTENSION_NAME);
    requestedExtensions.push_back(XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME);
    requestedExtensions.push_back(XR_EXT_DEBUG_UTILS_EXTENSION_NAME);

    // Populate the info to create the instance
    XrInstanceCreateInfo instanceCreateInfo
            = {
        .type = XR_TYPE_INSTANCE_CREATE_INFO,
        .next = NULL,
        .createFlags = 0,
        .applicationInfo =
        {
            // some compilers (like gcc) have trouble with char* initialization, so we use the strncpy afterwards
            .applicationName = {},
            .applicationVersion = 1,
            .engineName = {},
            .engineVersion = 0,
            .apiVersion = XR_CURRENT_API_VERSION,
        },
        .enabledApiLayerCount = 0,
        .enabledApiLayerNames = NULL,
        .enabledExtensionCount = static_cast<uint32_t>(requestedExtensions.size()),
        .enabledExtensionNames = requestedExtensions.data()
    };
    strncpy(instanceCreateInfo.applicationInfo.applicationName, "YARP OpenXr Device",
            XR_MAX_APPLICATION_NAME_SIZE);
    strncpy(instanceCreateInfo.applicationInfo.engineName, "OpenGL", XR_MAX_ENGINE_NAME_SIZE);


    //Add the structure to call the debug callback
    XrDebugUtilsMessengerCreateInfoEXT debugCallbackSettings =
    {
        .type = XR_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT,

        .next = NULL,

        //All the severities
        .messageSeverities = XR_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT |
        XR_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT |
        XR_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
        XR_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT,

        //All the messages
        .messageTypes = XR_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT |
        XR_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT |
        XR_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
        XR_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT,

        .userCallback = &OpenXrInterface::Implementation::OpenXrDebugCallback,

        .userData = m_pimpl.get()
    };
    instanceCreateInfo.next = &debugCallbackSettings;

    XrResult result = xrCreateInstance(&instanceCreateInfo, &(m_pimpl->instance));

    if (!m_pimpl->checkXrOutput(result, "Failed to create XR instance."))
        return false;

    printInstanceProperties();

    result = xrGetInstanceProcAddr(m_pimpl->instance, "xrGetOpenGLGraphicsRequirementsKHR",
                              (PFN_xrVoidFunction*)&(m_pimpl->pfnGetOpenGLGraphicsRequirementsKHR));
    if (!m_pimpl->checkXrOutput(result, "Failed to get OpenGL graphics requirements function!"))
        return false;

    return true;

}

bool OpenXrInterface::prepareXrSystem()
{
    yCTrace(OPENXRHEADSET);

    XrSystemGetInfo system_get_info = {
        .type = XR_TYPE_SYSTEM_GET_INFO,
        .next = NULL,
        .formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY};

    XrResult result = xrGetSystem(m_pimpl->instance, &system_get_info, &(m_pimpl->system_id));
    if (!m_pimpl->checkXrOutput(result, "Failed to get system for HMD form factor."))
        return false;

    yCInfo(OPENXRHEADSET) << "Successfully got XrSystem with id" << m_pimpl->system_id << "for HMD form factor";

    printSystemProperties();

    uint32_t view_count = 0;
    // We first get the number of view configurations for the STEREO type
    result = xrEnumerateViewConfigurationViews(m_pimpl->instance, m_pimpl->system_id,
                                               XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
                                               0, &view_count, NULL);
    if (!m_pimpl->checkXrOutput(result, "Failed to get view configuration view count!"))
        return false;

    m_pimpl->viewconfig_views.resize(view_count);
    for (uint32_t i = 0; i < view_count; i++) {
        m_pimpl->viewconfig_views[i].type = XR_TYPE_VIEW_CONFIGURATION_VIEW;
        m_pimpl->viewconfig_views[i].next = NULL;
    }
    result = xrEnumerateViewConfigurationViews(m_pimpl->instance, m_pimpl->system_id,
                                               XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, view_count,
                                               &view_count, m_pimpl->viewconfig_views.data());
    if (!m_pimpl->checkXrOutput(result, "Failed to enumerate view configuration views!"))
        return false;

    if (m_pimpl->viewconfig_views.size() != 2) {
        yCError(OPENXRHEADSET) << "Unexpected number of view configurations";
        return false;
    }

    for (uint32_t i = 0; i < view_count; i++) {
        yCInfo(OPENXRHEADSET, "View Configuration View %d:", i);
        yCInfo(OPENXRHEADSET, "\tResolution       : Recommended %dx%d, Max: %dx%d",
               m_pimpl->viewconfig_views[i].recommendedImageRectWidth,
               m_pimpl->viewconfig_views[i].recommendedImageRectHeight, m_pimpl->viewconfig_views[i].maxImageRectWidth,
               m_pimpl->viewconfig_views[i].maxImageRectHeight);
        yCInfo(OPENXRHEADSET, "\tSwapchain Samples: Recommended: %d, Max: %d)",
               m_pimpl->viewconfig_views[i].recommendedSwapchainSampleCount,
               m_pimpl->viewconfig_views[i].maxSwapchainSampleCount);
    }

    XrView dummy;
    dummy.type = XR_TYPE_VIEW;
    dummy.next = NULL;
    m_pimpl->views.resize(view_count, dummy);

    // Initialization for frame state
    m_pimpl->frame_state.type = XR_TYPE_FRAME_STATE;
    m_pimpl->frame_state.next = NULL;

    // Initialization for view state
    m_pimpl->view_state.type = XR_TYPE_VIEW_STATE;
    m_pimpl->view_state.next = NULL;

    // OpenXR requires checking graphics requirements before creating a session.
    XrGraphicsRequirementsOpenGLKHR opengl_reqs;
    opengl_reqs.type = XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR;
    opengl_reqs.next = NULL;

    // this function pointer was loaded with xrGetInstanceProcAddr
    result = m_pimpl->pfnGetOpenGLGraphicsRequirementsKHR(m_pimpl->instance, m_pimpl->system_id, &opengl_reqs);
    if (!m_pimpl->checkXrOutput(result, "Failed to get OpenGL graphics requirements!"))
        return false;

    return true;
}

void OpenXrInterface::printSystemProperties()
{
    XrSystemProperties system_props;
    system_props.type = XR_TYPE_SYSTEM_PROPERTIES;
    system_props.next = NULL;

    XrResult result = xrGetSystemProperties(m_pimpl->instance, m_pimpl->system_id, &system_props);
    if (!XR_SUCCEEDED(result))
    {
        yCDebug(OPENXRHEADSET) << "Failed to get system properties";
        return;
    }

    yCInfo(OPENXRHEADSET, "System properties for system %lu: \"%s\", vendor ID %d", system_props.systemId,
           system_props.systemName, system_props.vendorId);
    yCInfo(OPENXRHEADSET, "\tMax layers          : %d", system_props.graphicsProperties.maxLayerCount);
    yCInfo(OPENXRHEADSET, "\tMax swapchain height: %d",
           system_props.graphicsProperties.maxSwapchainImageHeight);
    yCInfo(OPENXRHEADSET, "\tMax swapchain width : %d",
           system_props.graphicsProperties.maxSwapchainImageWidth);
    yCInfo(OPENXRHEADSET, "\tOrientation Tracking: %d", system_props.trackingProperties.orientationTracking);
    yCInfo(OPENXRHEADSET, "\tPosition Tracking   : %d", system_props.trackingProperties.positionTracking);
}

bool OpenXrInterface::prepareWindow()
{
    yCTrace(OPENXRHEADSET);

    m_pimpl->windowSize.resize(2);
    m_pimpl->windowSize[0] = (m_pimpl->viewconfig_views[0].recommendedImageRectWidth +
                              m_pimpl->viewconfig_views[1].recommendedImageRectWidth)/2;
     m_pimpl->windowSize[1] = std::max(m_pimpl->viewconfig_views[0].recommendedImageRectHeight,
                                       m_pimpl->viewconfig_views[1].recommendedImageRectHeight) / 2;

    if (!glfwInit()) {
        yCError(OPENXRHEADSET, "Unable to initialize GLFW");
        return false;
    }
    glfwSetErrorCallback(&OpenXrInterface::Implementation::glfwErrorCallback);

    glfwWindowHint(GLFW_DEPTH_BITS, 16);
    m_pimpl->window = glfwCreateWindow(m_pimpl->windowSize[0], m_pimpl->windowSize[1], "YARP OpenXr Device Window", nullptr, nullptr);
    if (!m_pimpl->window) {
        yCError(OPENXRHEADSET, "Could not create window");
        return false;
    }

    glfwSetWindowUserPointer(m_pimpl->window, m_pimpl.get());
    glfwSetKeyCallback(m_pimpl->window, &OpenXrInterface::Implementation::glfwKeyCallback);
    glfwMakeContextCurrent(m_pimpl->window);
    glfwSwapInterval(1);

    glDebugMessageCallback(&OpenXrInterface::Implementation::GLMessageCallback, NULL);
    glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);

    return true;
}

bool OpenXrInterface::prepareXrSession()
{
    yCTrace(OPENXRHEADSET);

#ifdef XR_USE_PLATFORM_WIN32
    m_pimpl->graphics_binding_gl.type = XR_TYPE_GRAPHICS_BINDING_OPENGL_WIN32_KHR;
    m_pimpl->graphics_binding_gl.next = nullptr;
    m_pimpl->graphics_binding_gl.hDC = wglGetCurrentDC();
    m_pimpl->graphics_binding_gl.hGLRC = wglGetCurrentContext();
#else
    m_pimpl->graphics_binding_gl.type = XR_TYPE_GRAPHICS_BINDING_OPENGL_XLIB_KHR;
    m_pimpl->graphics_binding_gl.next = nullptr;
    m_pimpl->graphics_binding_gl.xDisplay = XOpenDisplay(NULL);
    m_pimpl->graphics_binding_gl.glxContext = glXGetCurrentContext();
    m_pimpl->graphics_binding_gl.glxDrawable = glXGetCurrentDrawable();
#endif

    XrSessionCreateInfo session_create_info = {
        .type = XR_TYPE_SESSION_CREATE_INFO,
        .next = &(m_pimpl->graphics_binding_gl),
        .createFlags = 0,
        .systemId = m_pimpl->system_id};

    XrResult result = xrCreateSession(m_pimpl->instance, &session_create_info, &(m_pimpl->session));
    if (!m_pimpl->checkXrOutput(result, "Failed to create session"))
        return false;

    yCInfo(OPENXRHEADSET, "Successfully created a session with OpenGL!");

    XrPosef identity_pose = {.orientation = {.x = 0, .y = 0, .z = 0, .w = 1.0},
                                    .position = {.x = 0, .y = 0, .z = 0}};

    //The LOCAL reference frame has the origin fixed in the user initial position with +Y up, +X to the right, and -Z forward.
    XrReferenceSpaceCreateInfo space_create_info = {.type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO,
                                                    .next = NULL,
                                                    .referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL,
                                                    .poseInReferenceSpace = identity_pose};

    result = xrCreateReferenceSpace(m_pimpl->session, &space_create_info, &(m_pimpl->play_space));
    if (! m_pimpl->checkXrOutput(result, "Failed to create play space!"))
        return false;

    space_create_info.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW; //This space if fixed on the head of the operator
    result = xrCreateReferenceSpace(m_pimpl->session, &space_create_info, &(m_pimpl->view_space));
    if (! m_pimpl->checkXrOutput(result, "Failed to create view space!"))
        return false;

    return true;

}

bool OpenXrInterface::prepareXrSwapchain()
{
    yCTrace(OPENXRHEADSET);

    //We create a single swapchain for both the views
    m_pimpl->swapchain_create_info = {
        .type = XR_TYPE_SWAPCHAIN_CREATE_INFO,
        .next = NULL,
        .createFlags = 0,
        .usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT,
        .format = GL_SRGB8_ALPHA8,
        .sampleCount = std::max(m_pimpl->viewconfig_views[0].recommendedSwapchainSampleCount,
                                m_pimpl->viewconfig_views[1].recommendedSwapchainSampleCount),
        .width = m_pimpl->viewconfig_views[0].recommendedImageRectWidth + m_pimpl->viewconfig_views[1].recommendedImageRectWidth,
        .height = std::max(m_pimpl->viewconfig_views[0].recommendedImageRectHeight,
                           m_pimpl->viewconfig_views[1].recommendedImageRectHeight),
        .faceCount = 1,
        .arraySize = 1,
        .mipCount = 1,
    };

    XrResult result = xrCreateSwapchain(m_pimpl->session, &(m_pimpl->swapchain_create_info), &(m_pimpl->swapchain));
    if (!m_pimpl->checkXrOutput(result, "Failed to create swapchain"))
        return false;

    // The runtime controls how many textures we have to be able to render to
    // (e.g. "triple buffering")
    uint32_t swapchain_images_count = 0;
    result = xrEnumerateSwapchainImages(m_pimpl->swapchain, 0, &swapchain_images_count, NULL);
    if (!m_pimpl->checkXrOutput(result, "Failed to enumerate swapchain images"))
        return false;

    XrSwapchainImageOpenGLKHR dummy;
    dummy.type = XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR;
    dummy.next = nullptr;
    m_pimpl->swapchain_images.resize(swapchain_images_count, dummy);

    result = xrEnumerateSwapchainImages(m_pimpl->swapchain, swapchain_images_count, &swapchain_images_count,
                                        (XrSwapchainImageBaseHeader*)(m_pimpl->swapchain_images.data()));
    if (!m_pimpl->checkXrOutput(result, "Failed to enumerate swapchain images"))
        return false;

    //We create an additional swapchain for the depth

    XrSwapchainCreateInfo depth_swapchain_create_info = {
        .type = XR_TYPE_SWAPCHAIN_CREATE_INFO,
        .next = NULL,
        .createFlags = 0,
        .usageFlags = XR_SWAPCHAIN_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
        .format = GL_DEPTH_COMPONENT16,
        .sampleCount = m_pimpl->swapchain_create_info.sampleCount,
        .width = m_pimpl->swapchain_create_info.width,
        .height = m_pimpl->swapchain_create_info.height,
        .faceCount = 1,
        .arraySize = 1,
        .mipCount = 1,
    };

    result = xrCreateSwapchain(m_pimpl->session, &depth_swapchain_create_info, &(m_pimpl->depth_swapchain));
    if (!m_pimpl->checkXrOutput(result, "Failed to create depth swapchain"))
        return false;

    uint32_t depth_swapchain_images_count = 0;
    result = xrEnumerateSwapchainImages(m_pimpl->depth_swapchain, 0, &depth_swapchain_images_count, NULL);
    if (!m_pimpl->checkXrOutput(result, "Failed to enumerate depth swapchain images"))
        return false;

    m_pimpl->depth_swapchain_images.resize(depth_swapchain_images_count, dummy);

    result = xrEnumerateSwapchainImages(m_pimpl->depth_swapchain, depth_swapchain_images_count, &depth_swapchain_images_count,
                                        (XrSwapchainImageBaseHeader*)(m_pimpl->depth_swapchain_images.data()));
    if (!m_pimpl->checkXrOutput(result, "Failed to enumerate depth swapchain images"))
        return false;

    return true;
}

bool OpenXrInterface::prepareXrCompositionLayers()
{
    yCTrace(OPENXRHEADSET);

    // Prepare projection views structures for the rendering
    m_pimpl->projection_views.resize(m_pimpl->viewconfig_views.size());
    int xOffset = 0;
    for (uint32_t i = 0; i < m_pimpl->projection_views.size(); i++) {
        m_pimpl->projection_views[i].type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW;
        m_pimpl->projection_views[i].next = NULL;
        m_pimpl->projection_views[i].subImage.swapchain = m_pimpl->swapchain;
        m_pimpl->projection_views[i].subImage.imageArrayIndex = 0;
        m_pimpl->projection_views[i].subImage.imageRect.offset.x = xOffset;
        m_pimpl->projection_views[i].subImage.imageRect.offset.y = 0;
        m_pimpl->projection_views[i].subImage.imageRect.extent.width =
            m_pimpl->viewconfig_views[i].recommendedImageRectWidth;
        m_pimpl->projection_views[i].subImage.imageRect.extent.height =
            m_pimpl->viewconfig_views[i].recommendedImageRectHeight;

        xOffset += m_pimpl->viewconfig_views[i].recommendedImageRectWidth; //Both the images are on the same swapchain, one after the other along the width dimension

        // projection_views[i].{pose, fov} have to be filled every frame in frame loop
    };

    m_pimpl->depth_projection_views.resize(m_pimpl->viewconfig_views.size());
    for (uint32_t i = 0; i < m_pimpl->depth_projection_views.size(); i++) {
        m_pimpl->depth_projection_views[i].type = XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR;
        m_pimpl->depth_projection_views[i].next = NULL;
        m_pimpl->depth_projection_views[i].minDepth = 0.f;
        m_pimpl->depth_projection_views[i].maxDepth = 1.f;
        m_pimpl->depth_projection_views[i].nearZ = 0.01f;
        m_pimpl->depth_projection_views[i].farZ = 100.0f;

        m_pimpl->depth_projection_views[i].subImage.swapchain = m_pimpl->depth_swapchain;
        m_pimpl->depth_projection_views[i].subImage.imageArrayIndex = 0;
        m_pimpl->depth_projection_views[i].subImage.imageRect.offset.x =
            m_pimpl->projection_views[i].subImage.imageRect.offset.x;
        m_pimpl->depth_projection_views[i].subImage.imageRect.offset.y = 0;
        m_pimpl->depth_projection_views[i].subImage.imageRect.extent.width =
            m_pimpl->projection_views[i].subImage.imageRect.extent.width;
        m_pimpl->depth_projection_views[i].subImage.imageRect.extent.height =
            m_pimpl->projection_views[i].subImage.imageRect.extent.height;

        // depth is chained to projection, not submitted as separate layer
        m_pimpl->projection_views[i].next = &(m_pimpl->depth_projection_views[i]);
    };

    m_pimpl->projection_layer = {
        .type = XR_TYPE_COMPOSITION_LAYER_PROJECTION,
        .next = NULL,
        .layerFlags = 0,
        .space = m_pimpl->play_space,
        .viewCount = static_cast<uint32_t>(m_pimpl->projection_views.size()),
        .views = m_pimpl->projection_views.data(),
    };

    return true;
}

bool OpenXrInterface::prepareXrActions()
{
    yCTrace(OPENXRHEADSET);
//TODO
    return true;
}

bool OpenXrInterface::prepareGlFramebuffer()
{
    yCTrace(OPENXRHEADSET);

    // Create a framebuffer for printing in our window (not required by OpenXr)
    glGenFramebuffers(1, &(m_pimpl->glFrameBufferId));

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

    glEnable(GL_DEPTH_TEST);

    return true;

}

void OpenXrInterface::pollXrEvents()
{
    yCTrace(OPENXRHEADSET);

    XrEventDataBuffer runtime_event;
    runtime_event.type = XR_TYPE_EVENT_DATA_BUFFER;
    runtime_event.next = NULL;
    XrResult poll_result = xrPollEvent(m_pimpl->instance, &runtime_event);

    while (poll_result == XR_SUCCESS) {
        switch (runtime_event.type) {
        case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING: {
            XrEventDataInstanceLossPending* event = (XrEventDataInstanceLossPending*)&runtime_event;
            yCError(OPENXRHEADSET, "EVENT: instance loss pending at %lu! Destroying instance.", event->lossTime);
            // still handle rest of the events instead of immediately quitting
            m_pimpl->closing = true;
            break;
        }
        case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
            XrEventDataSessionStateChanged* event = (XrEventDataSessionStateChanged*)&runtime_event;
            yCInfo(OPENXRHEADSET, "EVENT: session state changed from %d to %d", m_pimpl->state, event->state);

            m_pimpl->state = event->state;

            if (m_pimpl->state == XR_SESSION_STATE_READY && !(m_pimpl->closing))
            {
                XrSessionBeginInfo session_begin_info = {
                    .type = XR_TYPE_SESSION_BEGIN_INFO,
                    .next = NULL,
                    .primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO};
                XrResult result = xrBeginSession(m_pimpl->session, &session_begin_info);
                if (!m_pimpl->checkXrOutput(result, "Failed to begin session!"))
                {
                    m_pimpl->closing = true;
                    return;
                }

                yCInfo(OPENXRHEADSET, "Session started!");

            }
            else if (m_pimpl->state >= XR_SESSION_STATE_STOPPING) {
                yCInfo(OPENXRHEADSET, "Session is stopping...");
                // still handle rest of the events instead of immediately quitting
                m_pimpl->closing = true;
            }
            break;
        }
        case XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED: {
            yCInfo(OPENXRHEADSET, "EVENT: interaction profile changed!");

            break;
        }
        default: yCWarning(OPENXRHEADSET, "Unhandled event (type %d)", runtime_event.type);
        }

        runtime_event.type = XR_TYPE_EVENT_DATA_BUFFER;
        poll_result = xrPollEvent(m_pimpl->instance, &runtime_event);
    }
    if (poll_result == XR_EVENT_UNAVAILABLE) {
        // processed all events in the queue
    } else {
        yCWarning(OPENXRHEADSET, "Failed to poll events!");
        return;
    }

}

bool OpenXrInterface::startXrFrame()
{
    yCTrace(OPENXRHEADSET);

    XrFrameWaitInfo frame_wait_info = {.type = XR_TYPE_FRAME_WAIT_INFO, .next = NULL};
    XrResult result;

    XrFrameBeginInfo frame_begin_info = {.type = XR_TYPE_FRAME_BEGIN_INFO, .next = NULL};

    switch (m_pimpl->state) {
        case XR_SESSION_STATE_READY:
        case XR_SESSION_STATE_FOCUSED:
        case XR_SESSION_STATE_SYNCHRONIZED:
        case XR_SESSION_STATE_VISIBLE:

        result = xrWaitFrame(m_pimpl->session, &frame_wait_info, &(m_pimpl->frame_state));
        if (!m_pimpl->checkXrOutput(result, "xrWaitFrame() was not successful, exiting..."))
        {
            m_pimpl->closing = true;
            return false;
        }

        result = xrBeginFrame(m_pimpl->session, &frame_begin_info);
        if (!m_pimpl->checkXrOutput(result, "Failed to begin frame!"))
        {
            m_pimpl->closing = true;
            return false;
        }
        default:
            break;
    }

    return true;
}

void OpenXrInterface::updateXrViews()
{
    yCTrace(OPENXRHEADSET);
    XrViewLocateInfo view_locate_info = {.type = XR_TYPE_VIEW_LOCATE_INFO,
                                         .next = NULL,
                                         .viewConfigurationType =
                                             XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
                                         .displayTime = m_pimpl->frame_state.predictedDisplayTime,
                                         .space = m_pimpl->play_space};


    uint32_t output_viewCount = m_pimpl->views.size();
    XrResult result = xrLocateViews(m_pimpl->session, &view_locate_info, &(m_pimpl->view_state),
                                    m_pimpl->views.size(), &output_viewCount, m_pimpl->views.data());
    if (!m_pimpl->checkXrOutput(result, "Failed to begin frame!"))
    {
        m_pimpl->closing = true;
        return;
    }

    for (size_t i = 0; i < m_pimpl->views.size(); ++i)
    {
        m_pimpl->projection_views[i].pose = m_pimpl->views[i].pose;
        m_pimpl->projection_views[i].fov = m_pimpl->views[i].fov;
    }

}

void OpenXrInterface::updateXrActions()
{
    yCTrace(OPENXRHEADSET);
    //TODO

}

void OpenXrInterface::render()
{
    yCTrace(OPENXRHEADSET);

    // Acquire swapchain images
    XrSwapchainImageAcquireInfo acquire_info = {.type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO,
                                                .next = NULL};
    uint32_t acquired_index;
    XrResult result = xrAcquireSwapchainImage(m_pimpl->swapchain, &acquire_info, &acquired_index);
    if (!m_pimpl->checkXrOutput(result, "Failed to acquire swapchain image!"))
    {
        m_pimpl->closing = false;
        return;
    }

    XrSwapchainImageWaitInfo wait_info = {
        .type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO, .next = NULL, .timeout = 1000};
    result = xrWaitSwapchainImage(m_pimpl->swapchain, &wait_info);
    if (!m_pimpl->checkXrOutput(result, "Failed to wait for swapchain image!"))
    {
        m_pimpl->closing = false;
        return;
    }

    uint32_t depth_acquired_index;
    XrSwapchainImageAcquireInfo depth_acquire_info = {
        .type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO, .next = NULL};
    result = xrAcquireSwapchainImage(m_pimpl->depth_swapchain, &depth_acquire_info,
                                     &depth_acquired_index);
    if (!m_pimpl->checkXrOutput(result, "Failed to acquire depth swapchain image!"))
    {
        m_pimpl->closing = false;
        return;
    }

    XrSwapchainImageWaitInfo depth_wait_info = {
        .type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO, .next = NULL, .timeout = 1000};
    result = xrWaitSwapchainImage(m_pimpl->depth_swapchain, &depth_wait_info);
    if (!m_pimpl->checkXrOutput(result, "Failed to wait for depth swapchain image!"))
    {
        m_pimpl->closing = false;
        return;
    }

    //-----------------------------------
    // Dummy rendering
    glBindFramebuffer(GL_FRAMEBUFFER, m_pimpl->glFrameBufferId);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_pimpl->swapchain_images[acquired_index].image, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_pimpl->depth_swapchain_images[depth_acquired_index].image, 0);

    // "render" to the swapchain image
    glEnable(GL_SCISSOR_TEST);

    //select left eye
    glScissor(m_pimpl->projection_views[0].subImage.imageRect.offset.x,
              m_pimpl->projection_views[0].subImage.imageRect.offset.y,
              m_pimpl->projection_views[0].subImage.imageRect.extent.width,
              m_pimpl->projection_views[0].subImage.imageRect.extent.height);

    //Set green color
    glClearColor(0, 1, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //select right eye
    glScissor(m_pimpl->projection_views[1].subImage.imageRect.offset.x,
              m_pimpl->projection_views[1].subImage.imageRect.offset.y,
              m_pimpl->projection_views[1].subImage.imageRect.extent.width,
              m_pimpl->projection_views[1].subImage.imageRect.extent.height);

    //Set blue color
    glClearColor(0, 0, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //Restore default clear
    glClearColor(0, 0, 0, 0);

    glDisable(GL_SCISSOR_TEST);

    //------------------------------

    // Replicate swapchain on screen
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    GLint ww, wh;
    glfwGetWindowSize(m_pimpl->window, &ww, &wh);

    glBlitFramebuffer(0, 0, m_pimpl->swapchain_create_info.width, m_pimpl->swapchain_create_info.height,
                      0, 0, ww, wh, GL_COLOR_BUFFER_BIT, GL_NEAREST);

    glFramebufferTexture(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 0, 0);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);


    // Release swapchains
    XrSwapchainImageReleaseInfo release_info = {.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO,
                                                .next = NULL};
    result = xrReleaseSwapchainImage(m_pimpl->swapchain, &release_info);
    if (!m_pimpl->checkXrOutput(result, "Failed to release swapchain image!"))
    {
        m_pimpl->closing = false;
        return;
    }

    XrSwapchainImageReleaseInfo depth_release_info = {
        .type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO, .next = NULL};
    result = xrReleaseSwapchainImage(m_pimpl->depth_swapchain, &depth_release_info);
    if (!m_pimpl->checkXrOutput(result, "Failed to release depth swapchain image!"))
    {
        m_pimpl->closing = false;
        return;
    }

    glfwSwapBuffers(m_pimpl->window);

}

void OpenXrInterface::endXrFrame()
{
    yCTrace(OPENXRHEADSET);

    m_pimpl->layer_count = 0; //Reset the layer count

    if (m_pimpl->frame_state.shouldRender) {

        if (m_pimpl->view_state.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) {
            m_pimpl->submitLayer((XrCompositionLayerBaseHeader*) & (m_pimpl->projection_layer)); //Submit the projection layer
        }

        for (const auto& layer : m_pimpl->headLockedQuadLayers)
        {
            if (layer->shouldSubmit())
            {
                m_pimpl->submitLayer((XrCompositionLayerBaseHeader*) &layer->layer);
            }
        }
    }

    XrFrameEndInfo frameEndInfo = {.type = XR_TYPE_FRAME_END_INFO,
                                   .next = NULL,
                                   .displayTime = m_pimpl->frame_state.predictedDisplayTime,
                                   .environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE,
                                   .layerCount = static_cast<uint32_t>(m_pimpl->layer_count),
                                   .layers = m_pimpl->submitted_layers.data(),
                                   };

    XrResult result = xrEndFrame(m_pimpl->session, &frameEndInfo);
    if (!m_pimpl->checkXrOutput(result, "Failed to end frame!"))
    {
        m_pimpl->closing = true;
        return;
    }

}

OpenXrInterface::OpenXrInterface()
{
    yCTrace(OPENXRHEADSET);
    m_pimpl = std::make_unique<Implementation>();
}

OpenXrInterface::~OpenXrInterface()
{
    yCTrace(OPENXRHEADSET);
    close();
}

bool OpenXrInterface::initialize()
{
    yCTrace(OPENXRHEADSET);

    if (m_pimpl->initialized)
    {
        yCError(OPENXRHEADSET) << "The OpenXr interface has been already initialized.";
        return false;
    }

    m_pimpl->closing = false;

    bool ok = prepareXrInstance();
    ok = ok && prepareXrSystem();
    ok = ok && prepareWindow();
    ok = ok && prepareXrSession();
    ok = ok && prepareXrSwapchain();
    ok = ok && prepareXrCompositionLayers();
    ok = ok && prepareXrActions();
    ok = ok && prepareGlFramebuffer();

    m_pimpl->initialized = ok;

    return ok;
}

bool OpenXrInterface::isInitialized() const
{
    yCTrace(OPENXRHEADSET);

    return m_pimpl->initialized;
}

void OpenXrInterface::draw()
{
    yCTrace(OPENXRHEADSET);

    if (m_pimpl->closing)
    {
        return;
    }

    glfwPollEvents();

    if (m_pimpl->closing)
    {
        return;
    }

    m_pimpl->closing = glfwWindowShouldClose(m_pimpl->window);

    if (m_pimpl->closing)
    {
        return;
    }

    pollXrEvents();
    if (m_pimpl->closing)
    {
        return;
    }

    if (startXrFrame()) {
        updateXrViews();
        updateXrActions();
        if (m_pimpl->frame_state.shouldRender) {
            render();
        }
        endXrFrame();
    }
}

std::shared_ptr<IOpenXrQuadLayer> OpenXrInterface::addHeadFixedQuadLayer()
{
    yCTrace(OPENXRHEADSET);

    if (!m_pimpl->initialized)
    {
        yCError(OPENXRHEADSET) << "The OpenXr interface has not been initialized.";
        return nullptr;
    }

    std::shared_ptr<OpenXrQuadLayer> newLayer = std::make_shared<OpenXrQuadLayer>();

    XrSwapchainCreateInfo swapchain_create_info = {
        .type = XR_TYPE_SWAPCHAIN_CREATE_INFO,
        .next = NULL,
        .createFlags = 0,
        .usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT |
        XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT |
        XR_SWAPCHAIN_USAGE_MUTABLE_FORMAT_BIT,
        .format = GL_SRGB8_ALPHA8,
        .sampleCount = m_pimpl->swapchain_create_info.sampleCount,
        .width = std::min(m_pimpl->viewconfig_views[0].recommendedImageRectWidth,
                          m_pimpl->viewconfig_views[1].recommendedImageRectWidth),
        .height = std::min(m_pimpl->viewconfig_views[0].recommendedImageRectHeight,
                           m_pimpl->viewconfig_views[1].recommendedImageRectHeight),
        .faceCount = 1,
        .arraySize = 1,
        .mipCount = 1,
    };

    newLayer->swapchainHeight = swapchain_create_info.height;
    newLayer->swapchainWidth = swapchain_create_info.width;

    XrRect2Di initialRect =
    {
        .offset = {.x = 0, .y = 0},
        .extent =
        {
            .width = static_cast<int32_t>(swapchain_create_info.width),
            .height = static_cast<int32_t>(swapchain_create_info.height),
        }
    };

    XrPosef initialPose =
    {
        .orientation = {.x = 0.0, .y = 0.0, .z = 0.0, .w = 1.0},
        .position = {.x = 0.0, .y = 0.0, .z = -1.0} //The origin is in the centroid of the views
                                                    //with +Y up, +X to the right, and -Z forward.
                                                    //See https://www.khronos.org/registry/OpenXR/specs/1.0/html/xrspec.html#reference-spaces
    };

    newLayer->layer = {
        .type = XR_TYPE_COMPOSITION_LAYER_QUAD,
        .next = NULL,
        .layerFlags = 0,
        .space = m_pimpl->view_space,        //Head fixed
        .eyeVisibility = XR_EYE_VISIBILITY_BOTH,
        .subImage = {
            .swapchain = XR_NULL_HANDLE,
            .imageRect = initialRect,
            .imageArrayIndex = 0
        },
        .pose = initialPose,
        .size = {.width = 1.75, .height = 1.75} //These numbers have been heuristically found
                                                //to fit almost the entirety of the screen when
                                                //at 1 meter distance. This, of course, will
                                                //depend on the FOV. They can be edited at any time
    };

    XrResult result = xrCreateSwapchain(m_pimpl->session, &(swapchain_create_info), &(newLayer->layer.subImage.swapchain));
    if (!m_pimpl->checkXrOutput(result, "Failed to create swapchain for the new head fixed quad layer."))
        return nullptr;

    uint32_t swapchain_images_count = 0;
    result = xrEnumerateSwapchainImages(newLayer->layer.subImage.swapchain, 0, &swapchain_images_count, NULL);
    if (!m_pimpl->checkXrOutput(result, "Failed to enumerate swapchain images for the new head fixed quad layer."))
        return nullptr;

    XrSwapchainImageOpenGLKHR dummy;
    dummy.type = XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR;
    dummy.next = nullptr;
    newLayer->swapchain_images.resize(swapchain_images_count, dummy);

    result = xrEnumerateSwapchainImages(newLayer->layer.subImage.swapchain, swapchain_images_count, &swapchain_images_count,
                                        (XrSwapchainImageBaseHeader*)(newLayer->swapchain_images.data()));
    if (!m_pimpl->checkXrOutput(result, "Failed to enumerate swapchain images"))
        return nullptr;

    m_pimpl->headLockedQuadLayers.push_back(newLayer);

    return newLayer;
}

bool OpenXrInterface::isRunning() const
{
    yCTrace(OPENXRHEADSET);

    return m_pimpl->initialized && !m_pimpl->closing;
}

void OpenXrInterface::setKeyCallback(std::function<void (int, int, int, int)> keyCallback)
{
    yCTrace(OPENXRHEADSET);

    m_pimpl->keyCallback = keyCallback;
}

void OpenXrInterface::close()
{
    yCTrace(OPENXRHEADSET);

    m_pimpl->closing = true;

    if (m_pimpl->glFrameBufferId != 0) {
        glDeleteFramebuffers(1, &(m_pimpl->glFrameBufferId));
        m_pimpl->glFrameBufferId = 0;
    }

    if (m_pimpl->state != XR_SESSION_STATE_UNKNOWN)
    {
        xrRequestExitSession(m_pimpl->session);
        xrEndSession(m_pimpl->session);
        m_pimpl->state = XR_SESSION_STATE_UNKNOWN;
    }

    if (m_pimpl->session != XR_NULL_HANDLE)
    {
        xrDestroySession(m_pimpl->session);
        m_pimpl->session = XR_NULL_HANDLE;
    }

    m_pimpl->swapchain_images.clear();
    m_pimpl->depth_swapchain_images.clear();

    if (m_pimpl->instance != XR_NULL_HANDLE)
    {
        xrDestroyInstance(m_pimpl->instance);
        m_pimpl->instance = XR_NULL_HANDLE;
        m_pimpl->system_id = XR_NULL_SYSTEM_ID;
        m_pimpl->session = XR_NULL_HANDLE;
        m_pimpl->play_space = XR_NULL_HANDLE;
        m_pimpl->view_space = XR_NULL_HANDLE;
    }

    m_pimpl->viewconfig_views.clear();
    m_pimpl->projection_views.clear();
    m_pimpl->depth_projection_views.clear();
    m_pimpl->views.clear();
    m_pimpl->headLockedQuadLayers.clear();
    m_pimpl->submitted_layers.clear();
    m_pimpl->layer_count = 0;

    if (m_pimpl->window)
    {
        glfwDestroyWindow(m_pimpl->window);
        glfwTerminate();
        m_pimpl->window = nullptr;
    }

    m_pimpl->initialized = false;

    yCInfo(OPENXRHEADSET) << "Closed";
}
