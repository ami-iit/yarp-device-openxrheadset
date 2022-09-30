/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */

#include <impl/OpenXrInterfaceImpl.h>

//#define DEBUG_RENDERING


bool OpenXrInterface::checkExtensions()
{
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

        if (strcmp(XR_HTCX_VIVE_TRACKER_INTERACTION_EXTENSION_NAME, ext_props[i].extensionName) == 0) {
            m_pimpl->htc_trackers_supported = true;
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

    if (!m_pimpl->htc_trackers_supported) {
        yCWarning(OPENXRHEADSET) << "Runtime does not support the HTC Vive Trackers!";
    }

    return true;
}

void OpenXrInterface::printInstanceProperties()
{
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
    if (!checkExtensions())
    {
        return false;
    }

    // List the requested extensions to the runtime
    std::vector<const char*> requestedExtensions;
    requestedExtensions.push_back(XR_KHR_OPENGL_ENABLE_EXTENSION_NAME);
    requestedExtensions.push_back(XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME);
    requestedExtensions.push_back(XR_EXT_DEBUG_UTILS_EXTENSION_NAME);
    if (m_pimpl->htc_trackers_supported)
    {
        requestedExtensions.push_back(XR_HTCX_VIVE_TRACKER_INTERACTION_EXTENSION_NAME);
    }

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

    if (m_pimpl->htc_trackers_supported)
    {
        result = xrGetInstanceProcAddr(m_pimpl->instance, "xrEnumerateViveTrackerPathsHTCX",
            (PFN_xrVoidFunction*)&(m_pimpl->pfn_xrEnumerateViveTrackerPathsHTCX));
        if (!m_pimpl->checkXrOutput(result, "Failed to get the function to enumerate the HTC trackers!"))
            return false;
    }

    return true;

}

bool OpenXrInterface::prepareXrSystem()
{
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

    // Initialization for the head location
    m_pimpl->view_space_velocity.type = XR_TYPE_SPACE_VELOCITY;
    m_pimpl->view_space_velocity.next = NULL;
    m_pimpl->view_space_velocity.velocityFlags = 0;
    m_pimpl->view_space_location.type = XR_TYPE_SPACE_LOCATION;
    m_pimpl->view_space_location.next = &(m_pimpl->view_space_velocity);
    m_pimpl->view_space_location.locationFlags = 0;

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

bool OpenXrInterface::prepareGL()
{
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

#ifndef DEBUG_RENDERING
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
#endif
    glfwWindowHint(GLFW_DEPTH_BITS, 16);
    m_pimpl->window = glfwCreateWindow(m_pimpl->windowSize[0], m_pimpl->windowSize[1], "YARP OpenXr Device Window", nullptr, nullptr);
    if (!m_pimpl->window) {
        yCError(OPENXRHEADSET, "Could not create window");
        return false;
    }

    glfwMakeContextCurrent(m_pimpl->window);
    glfwSwapInterval(1);

    // Initialize the GLEW OpenGL 3.x bindings
    // GLEW must be initialized after creating the window
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (err != GLEW_OK) {
        yCError(OPENXRHEADSET) << "glewInit failed, aborting.";
        return false;
    }
    yCInfo(OPENXRHEADSET) << "Using GLEW" << (const char*)glewGetString(GLEW_VERSION);

    glDebugMessageControl(GL_DEBUG_SOURCE_API, GL_DEBUG_TYPE_OTHER, GL_DEBUG_SEVERITY_NOTIFICATION, 0, NULL, GL_FALSE); //This is to ignore message 0x20071 about the use of the VIDEO memory

    glDebugMessageCallback(&OpenXrInterface::Implementation::GLMessageCallback, NULL);
    glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    glEnable(GL_DEBUG_OUTPUT);
    glEnable(GL_TEXTURE_2D);

    return true;
}

bool OpenXrInterface::prepareXrSession()
{
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
    m_pimpl->projection_view_swapchain_create_info.resize(m_pimpl->viewconfig_views.size());
    m_pimpl->projection_view_swapchains.resize(m_pimpl->viewconfig_views.size());
    m_pimpl->projection_view_depth_swapchains.resize(m_pimpl->viewconfig_views.size());

    for (size_t i = 0; i < m_pimpl->projection_view_swapchain_create_info.size(); ++i)
    {
        //We create a swapchain for both the views
        m_pimpl->projection_view_swapchain_create_info[i] = {
            .type = XR_TYPE_SWAPCHAIN_CREATE_INFO,
            .next = NULL,
            .createFlags = 0,
            .usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT,
            .format = GL_SRGB8_ALPHA8,
            .sampleCount = m_pimpl->viewconfig_views[i].recommendedSwapchainSampleCount,
            .width = m_pimpl->viewconfig_views[i].recommendedImageRectWidth,
            .height = m_pimpl->viewconfig_views[i].recommendedImageRectHeight,
            .faceCount = 1,
            .arraySize = 1,
            .mipCount = 1,
        };

        XrResult result = xrCreateSwapchain(m_pimpl->session, &(m_pimpl->projection_view_swapchain_create_info[i]), &(m_pimpl->projection_view_swapchains[i].swapchain));
        if (!m_pimpl->checkXrOutput(result, "Failed to create swapchain"))
            return false;

        // The runtime controls how many textures we have to be able to render to
        // (e.g. "triple buffering")
        uint32_t swapchain_images_count = 0;
        result = xrEnumerateSwapchainImages(m_pimpl->projection_view_swapchains[i].swapchain, 0, &swapchain_images_count, NULL);
        if (!m_pimpl->checkXrOutput(result, "Failed to enumerate swapchain images"))
            return false;

        XrSwapchainImageOpenGLKHR dummy;
        dummy.type = XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR;
        dummy.next = nullptr;
        m_pimpl->projection_view_swapchains[i].swapchain_images.resize(swapchain_images_count, dummy);

        result = xrEnumerateSwapchainImages(m_pimpl->projection_view_swapchains[i].swapchain, swapchain_images_count, &swapchain_images_count,
                                            (XrSwapchainImageBaseHeader*)(m_pimpl->projection_view_swapchains[i].swapchain_images.data()));
        if (!m_pimpl->checkXrOutput(result, "Failed to enumerate swapchain images"))
            return false;

        //We create an additional swapchain for the depth

        XrSwapchainCreateInfo depth_swapchain_create_info = {
            .type = XR_TYPE_SWAPCHAIN_CREATE_INFO,
            .next = NULL,
            .createFlags = 0,
            .usageFlags = XR_SWAPCHAIN_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
            .format = GL_DEPTH_COMPONENT16,
            .sampleCount = m_pimpl->projection_view_swapchain_create_info[i].sampleCount,
            .width = m_pimpl->projection_view_swapchain_create_info[i].width,
            .height = m_pimpl->projection_view_swapchain_create_info[i].height,
            .faceCount = 1,
            .arraySize = 1,
            .mipCount = 1,
        };

        result = xrCreateSwapchain(m_pimpl->session, &depth_swapchain_create_info, &(m_pimpl->projection_view_depth_swapchains[i].swapchain));
        if (!m_pimpl->checkXrOutput(result, "Failed to create depth swapchain"))
            return false;

        uint32_t depth_swapchain_images_count = 0;
        result = xrEnumerateSwapchainImages(m_pimpl->projection_view_depth_swapchains[i].swapchain, 0, &depth_swapchain_images_count, NULL);
        if (!m_pimpl->checkXrOutput(result, "Failed to enumerate depth swapchain images"))
            return false;

        m_pimpl->projection_view_depth_swapchains[i].swapchain_images.resize(depth_swapchain_images_count, dummy);

        result = xrEnumerateSwapchainImages(m_pimpl->projection_view_depth_swapchains[i].swapchain, depth_swapchain_images_count, &depth_swapchain_images_count,
                                            (XrSwapchainImageBaseHeader*)(m_pimpl->projection_view_depth_swapchains[i].swapchain_images.data()));
        if (!m_pimpl->checkXrOutput(result, "Failed to enumerate depth swapchain images"))
            return false;
    }


    return true;
}

bool OpenXrInterface::prepareXrCompositionLayers()
{
    // Prepare projection views structures for the rendering
    m_pimpl->projection_views.resize(m_pimpl->viewconfig_views.size());
    for (size_t i = 0; i < m_pimpl->projection_views.size(); i++) {
        m_pimpl->projection_views[i].type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW;
        m_pimpl->projection_views[i].next = NULL;
        m_pimpl->projection_views[i].subImage.swapchain =m_pimpl->projection_view_swapchains[i].swapchain;
        m_pimpl->projection_views[i].subImage.imageArrayIndex = 0;
        m_pimpl->projection_views[i].subImage.imageRect.offset.x = 0;
        m_pimpl->projection_views[i].subImage.imageRect.offset.y = 0;
        m_pimpl->projection_views[i].subImage.imageRect.extent.width =
            m_pimpl->viewconfig_views[i].recommendedImageRectWidth;
        m_pimpl->projection_views[i].subImage.imageRect.extent.height =
            m_pimpl->viewconfig_views[i].recommendedImageRectHeight;

        // projection_views[i].{pose, fov} have to be filled every frame in frame loop
    };

    m_pimpl->depth_projection_views.resize(m_pimpl->viewconfig_views.size());
    for (size_t i = 0; i < m_pimpl->depth_projection_views.size(); i++) {
        m_pimpl->depth_projection_views[i].type = XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR;
        m_pimpl->depth_projection_views[i].next = NULL;
        m_pimpl->depth_projection_views[i].minDepth = 0.f;
        m_pimpl->depth_projection_views[i].maxDepth = 1.f;
        m_pimpl->depth_projection_views[i].nearZ = 0.01f;
        m_pimpl->depth_projection_views[i].farZ = 100.0f;

        m_pimpl->depth_projection_views[i].subImage.swapchain = m_pimpl->projection_view_depth_swapchains[i].swapchain;
        m_pimpl->depth_projection_views[i].subImage.imageArrayIndex = 0;
        m_pimpl->depth_projection_views[i].subImage.imageRect.offset.x = 0;
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
        .space = m_pimpl->view_space,
        .viewCount = static_cast<uint32_t>(m_pimpl->projection_views.size()),
        .views = m_pimpl->projection_views.data(),
    };

    return true;
}

bool OpenXrInterface::prepareXrActions()
{
    std::vector<InteractionProfileDeclaration> interactionProfilesPrefixes =
    {{KHR_SIMPLE_CONTROLLER_INTERACTION_PROFILE, "khr_"},
     {OCULUS_TOUCH_INTERACTION_PROFILE_TAG, "oculus_"},
     {HTC_VIVE_INTERACTION_PROFILE_TAG, "vive_"}};

    TopLevelPathDeclaration left_hand;
    left_hand.stringPath = "/user/hand/left";
    left_hand.actionNamePrefix = "left_";
    TopLevelPathDeclaration right_hand;
    right_hand.stringPath = "/user/hand/right";
    right_hand.actionNamePrefix = "right_";


    InputActionsDeclaration& khr_left_inputs = left_hand.inputsDeclarations[KHR_SIMPLE_CONTROLLER_INTERACTION_PROFILE];

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


    khr_left_inputs.poses =
    {
        {"/input/grip/pose", "grip"}
    };

    khr_left_inputs.buttons =
    {
        {"/input/select/click",  "select"},
        {"/input/menu/click",    "menu"},
    };

    right_hand.inputsDeclarations[KHR_SIMPLE_CONTROLLER_INTERACTION_PROFILE] = khr_left_inputs; //Left and Right inputs for this interaction profile are the same


    InputActionsDeclaration& oculus_left_inputs = left_hand.inputsDeclarations[OCULUS_TOUCH_INTERACTION_PROFILE_TAG];
    InputActionsDeclaration& oculus_right_inputs = right_hand.inputsDeclarations[OCULUS_TOUCH_INTERACTION_PROFILE_TAG];

    oculus_left_inputs.poses =
    {
        {"/input/grip/pose", "grip"}
    };
    oculus_right_inputs.poses = oculus_left_inputs.poses;


    oculus_left_inputs.buttons =
    {
        {"/input/menu/click",        "menu"},
        {"/input/x/click",           "x"},
        {"/input/y/click",           "y"},
        {"/input/thumbstick/click",  "thumbstick_click"}
    };
    oculus_right_inputs.buttons =
    {
        {"/input/a/click",          "a"},
        {"/input/b/click",          "b"},
        {"/input/thumbstick/click", "thumbstick_click"}
    };

    oculus_left_inputs.axes =
    {
        {"/input/trigger/value",  "trigger"},
        {"/input/squeeze/value",  "squeeze"}
    };
    oculus_right_inputs.axes = oculus_left_inputs.axes;

    oculus_left_inputs.thumbsticks =
    {
        {"/input/thumbstick",  "thumbstick"}
    };

    oculus_right_inputs.thumbsticks = oculus_left_inputs.thumbsticks;


    InputActionsDeclaration& vive_left_inputs = left_hand.inputsDeclarations[HTC_VIVE_INTERACTION_PROFILE_TAG];

    vive_left_inputs.poses =
    {
        {"/input/grip/pose", "grip"}
    };

    vive_left_inputs.buttons =
    {
        {"/input/menu/click",      "menu"},
        {"/input/trigger/click",   "trigger_click"},
        {"/input/squeeze/click",   "squeeze_click"},
        {"/input/trackpad/click",  "trackpad_click"}
    };

    vive_left_inputs.axes =
    {
        {"/input/trigger/value",  "trigger"},
    };

    vive_left_inputs.thumbsticks =
    {
        {"/input/trackpad",  "trackpad"},
    };

    right_hand.inputsDeclarations[HTC_VIVE_INTERACTION_PROFILE_TAG] = vive_left_inputs; //the inputs from the left and right hand are the same

    std::vector<TopLevelPathDeclaration> topLevelPathsDeclaration = {left_hand,    //The left hand should always come first in this list
                                                                     right_hand};  //The right hand should always come second in this list

    //HTC Trackers
    if (m_pimpl->htc_trackers_supported)
    {
        InputActionsDeclaration viveTrackerInputs;

        viveTrackerInputs.poses =
        {
            {"/input/grip/pose", "pose"}
        };

        viveTrackerInputs.buttons =
        {
            {"/input/menu/click",      "menu"},
            {"/input/trigger/click",   "trigger_click"},
            {"/input/squeeze/click",   "squeeze_click"},
            {"/input/trackpad/click",  "trackpad_click"}
        };

        viveTrackerInputs.axes =
        {
            {"/input/trigger/value",  "trigger"},
        };

        viveTrackerInputs.thumbsticks =
        {
            {"/input/trackpad",  "trackpad"},
        };

        std::string trackersPrefix = "/user/vive_tracker_htcx/role/";

        std::vector<std::string> trackersNameList = {"handheld_object",
                                                     "left_foot",
                                                     "right_foot",
                                                     "left_shoulder",
                                                     "right_shoulder",
                                                     "left_elbow",
                                                     "right_elbow",
                                                     "left_knee",
                                                     "right_knee",
                                                     "waist",
                                                     "chest",
                                                     "camera",
                                                     "keyboard"};

        m_pimpl->htc_trackers_status.clear();

        for (size_t name = 0; name < trackersNameList.size(); ++name)
        {
            TopLevelPathDeclaration tracker;
            tracker.stringPath = trackersPrefix + trackersNameList[name];
            tracker.actionNamePrefix = trackersNameList[name] + "_";
            tracker.inputsDeclarations[HTC_VIVE_TRACKER_INTERACTION_PROFILE_TAG] = viveTrackerInputs;

            topLevelPathsDeclaration.push_back(tracker);
            m_pimpl->htc_trackers_status[tracker.stringPath].top_level_index = topLevelPathsDeclaration.size() - 1;
        }

        interactionProfilesPrefixes.push_back({HTC_VIVE_TRACKER_INTERACTION_PROFILE_TAG, "vive_tracker_"});
    }

    if (!m_pimpl->fillActionBindings(interactionProfilesPrefixes, topLevelPathsDeclaration))
        return false;

    return true;
}

bool OpenXrInterface::prepareGlFramebuffer()
{
    // Create a framebuffer for printing in our window (not required by OpenXr)
    glGenFramebuffers(1, &(m_pimpl->glFrameBufferId));

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

    glEnable(GL_DEPTH_TEST);

    return true;

}

void OpenXrInterface::pollXrEvents()
{
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
            yCInfo(OPENXRHEADSET, "EVENT: session state changed from %s to %s",
                   m_pimpl->sessionStateToString(m_pimpl->state).c_str(),
                   m_pimpl->sessionStateToString(event->state).c_str());

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

                XrSessionActionSetsAttachInfo actionset_attach_info = {
                    .type = XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO,
                    .next = NULL,
                    .countActionSets = 1,
                    .actionSets = &m_pimpl->actionset};
                result = xrAttachSessionActionSets(m_pimpl->session, &actionset_attach_info);
                if (!m_pimpl->checkXrOutput(result, "Failed to attach action set"))
                {
                    m_pimpl->closing = true;
                }

                if (!updateInteractionProfiles())
                {
                    m_pimpl->closing = true;
                }
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

            if (!updateInteractionProfiles())
            {
                m_pimpl->closing = true;
            }

            break;
        }
        case XR_TYPE_EVENT_DATA_VIVE_TRACKER_CONNECTED_HTCX: {
            yCInfo(OPENXRHEADSET, "EVENT: Vive tracker connected!");

            if (!updateConnectedTrackers())
            {
                m_pimpl->closing = true;
            }

            yCInfo(OPENXRHEADSET, "Updating interaction profiles manually.");

            //This is because we force the interaction profile to NO_INTERACTION_PROFILE_TAG of the not connected trackers.
            if (!updateInteractionProfiles())
            {
                m_pimpl->closing = true;
            }

            break;
        }
        case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING: {
            yCInfo(OPENXRHEADSET, "EVENT: Reference space change pending!");
            const XrEventDataReferenceSpaceChangePending& referenceSpaceChanging =
                    *reinterpret_cast<XrEventDataReferenceSpaceChangePending*>(&runtime_event);

            if (referenceSpaceChanging.referenceSpaceType == XR_REFERENCE_SPACE_TYPE_LOCAL)
            {
                m_pimpl->local_reference_space_changing = true;
                m_pimpl->local_reference_space_change_time = referenceSpaceChanging.changeTime;
            }

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

void OpenXrInterface::updateXrSpaces()
{
    //Update location of the views
    XrViewLocateInfo view_locate_info = {.type = XR_TYPE_VIEW_LOCATE_INFO,
                                         .next = NULL,
                                         .viewConfigurationType =
                                             XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
                                         .displayTime = m_pimpl->frame_state.predictedDisplayTime,
                                         .space = m_pimpl->play_space};


    uint32_t output_viewCount = m_pimpl->views.size();
    XrResult result = xrLocateViews(m_pimpl->session, &view_locate_info, &(m_pimpl->view_state),
                                    m_pimpl->views.size(), &output_viewCount, m_pimpl->views.data());
    if (!m_pimpl->checkXrOutput(result, "Failed to locate the views!"))
    {
        m_pimpl->closing = true;
        return;
    }

    XrPosef identity_pose = { .orientation = {.x = 0, .y = 0, .z = 0, .w = 1.0},
                            .position = {.x = 0, .y = 0, .z = 0} };

    for (size_t i = 0; i < m_pimpl->views.size(); ++i)
    {
        m_pimpl->projection_views[i].pose = identity_pose;
        m_pimpl->projection_views[i].fov = m_pimpl->views[i].fov;
    }

    result = xrLocateSpace(m_pimpl->view_space, m_pimpl->play_space,
                           m_pimpl->locate_space_time, &m_pimpl->view_space_location);

    if (!m_pimpl->checkXrOutput(result, "Failed to locate the head space!"))
    {
        m_pimpl->closing = true;
        return;
    }

}

void OpenXrInterface::updateXrActions()
{
    XrActiveActionSet active_actionsets =
        {.actionSet = m_pimpl->actionset,
         .subactionPath = XR_NULL_PATH};

    XrActionsSyncInfo actions_sync_info = {
        .type = XR_TYPE_ACTIONS_SYNC_INFO,
        .countActiveActionSets = 1,
        .activeActionSets = &active_actionsets,
    };
    XrResult result = xrSyncActions(m_pimpl->session, &actions_sync_info);
    if (!m_pimpl->checkXrOutput(result, "Failed to sync actions!"))
        return;

    for (auto& topLevel : m_pimpl->top_level_paths)
    {
        InputActions& inputs = topLevel.currentActions();

        for (PoseAction& pose : inputs.poses)
        {
            result = pose.update(m_pimpl->session, m_pimpl->play_space, m_pimpl->locate_space_time);
            m_pimpl->checkXrOutput(result, "Failed to get the pose of %s!", pose.name.c_str()); //Continue anyway
        }

        for (Action<bool>& button : inputs.buttons)
        {
            result = button.update(m_pimpl->session);
            m_pimpl->checkXrOutput(result, "Failed to update the status of %s!", button.name.c_str()); //Continue anyway
        }

        for (Action<float>& axis : inputs.axes)
        {
            result = axis.update(m_pimpl->session);
            m_pimpl->checkXrOutput(result, "Failed to update the status of %s!", axis.name.c_str()); //Continue anyway
        }

        for (Action<Eigen::Vector2f>& thumbstick : inputs.thumbsticks)
        {
            result = thumbstick.update(m_pimpl->session);
            m_pimpl->checkXrOutput(result, "Failed to update the status of %s!", thumbstick.name.c_str()); //Continue anyway
        }
    }
}

bool OpenXrInterface::updateInteractionProfiles()
{
    XrInteractionProfileState state = {.type = XR_TYPE_INTERACTION_PROFILE_STATE};

    for (auto& topLevel : m_pimpl->top_level_paths)
    {
        topLevel.currentInteractionProfile = NO_INTERACTION_PROFILE_TAG;

        XrResult result = xrGetCurrentInteractionProfile(m_pimpl->session, topLevel.xrPath, &state);
        if (!XR_SUCCEEDED(result))
        {
            topLevel.currentInteractionProfile = TOP_LEVEL_NOT_SUPPORTED_TAG;
        }
        else
        {
            XrPath xrProfile = state.interactionProfile;

            if (xrProfile != XR_NULL_PATH)
            {
                char interactionProfile[XR_MAX_PATH_LENGTH];
                uint32_t strl;
                result = xrPathToString(m_pimpl->instance, xrProfile, XR_MAX_PATH_LENGTH, &strl, interactionProfile);
                if (!m_pimpl->checkXrOutput(result, "Failed to convert interaction profile path for ", topLevel.stringPath.c_str()))
                {
                    return false;
                }

                topLevel.currentInteractionProfile = interactionProfile;
            }
            else
            {
                topLevel.currentInteractionProfile = NO_INTERACTION_PROFILE_TAG;
            }
        }
    }

    forceTrackersInteractionProfile();

    printInteractionProfiles();

    return true;
}

void OpenXrInterface::printInteractionProfiles()
{
    for (auto& topLevel : m_pimpl->top_level_paths)
    {
        if (topLevel.currentInteractionProfile == TOP_LEVEL_NOT_SUPPORTED_TAG)
        {
            yCError(OPENXRHEADSET, "Failed to get the interaction profile of %s", topLevel.stringPath.c_str());
        }
        else
        {
            yCInfo(OPENXRHEADSET) << "Interaction profile of" << topLevel.stringPath << ":" << topLevel.currentInteractionProfile;
        }
    }

    if (m_pimpl->top_level_paths[0].currentInteractionProfile != m_pimpl->top_level_paths[1].currentInteractionProfile)
    {
        yCWarning(OPENXRHEADSET) << "The left and right hand have different interaction profiles. Maybe are you using a single controller?";
    }
}

bool OpenXrInterface::updateConnectedTrackers()
{
    for (auto& tracker : m_pimpl->htc_trackers_status)
    {
        tracker.second.connected = false; //Reset the connected flag
    }

    uint32_t numberOfTrackers;
    XrResult result = m_pimpl->pfn_xrEnumerateViveTrackerPathsHTCX(m_pimpl->instance, 0, &numberOfTrackers, nullptr);
    if (!m_pimpl->checkXrOutput(result, "Failed to retrieve the number of HTC trackers connected."))
        return false;

    m_pimpl->htc_trackers_connected.resize(numberOfTrackers);

    result = m_pimpl->pfn_xrEnumerateViveTrackerPathsHTCX(m_pimpl->instance, (uint32_t)(m_pimpl->htc_trackers_connected.size()),
        &numberOfTrackers, m_pimpl->htc_trackers_connected.data());
    if (!m_pimpl->checkXrOutput(result, "Failed to get the list of HTC trackers connected."))
        return false;

    m_pimpl->htc_trackers_connected.resize(numberOfTrackers);

    for (size_t i = 0; i < m_pimpl->htc_trackers_connected.size(); ++i)
    {
        auto& tracker = m_pimpl->htc_trackers_connected[i];
        uint32_t nCount;
        char sPersistentPath[XR_MAX_PATH_LENGTH];
        XrResult result = xrPathToString(m_pimpl->instance,
            tracker.persistentPath,
            sizeof(sPersistentPath), &nCount, sPersistentPath);
        if (m_pimpl->checkXrOutput(result, "Failed to convert the tracker %d persistent path to string.", i))
        {
            yCInfo(OPENXRHEADSET, "Vive Tracker %zu persistent path: %s \n", i, sPersistentPath);
        }

        if (tracker.rolePath != XR_NULL_PATH)
        {
            char sRolePath[XR_MAX_PATH_LENGTH];
            result = xrPathToString(m_pimpl->instance,
                tracker.rolePath, sizeof(sRolePath),
                &nCount, sRolePath);

            if (m_pimpl->checkXrOutput(result, "Failed to convert the tracker %d role path to string.", i))
            {
                yCInfo(OPENXRHEADSET, "Vive Tracker %zu role path: %s \n", i, sRolePath);

                m_pimpl->htc_trackers_status[sRolePath].connected = true;
            }
        }
        else
        {
            yCWarning(OPENXRHEADSET, "No role specified for the tracker. It will not be considered."
                " Refer to https://www.khronos.org/registry/OpenXR/specs/1.0/html/xrspec.html#XR_HTCX_vive_tracker_interaction for more informations.");
        }
    }

    return true;
}

void OpenXrInterface::forceTrackersInteractionProfile()
{
    for (auto& tracker : m_pimpl->htc_trackers_status)
    {
        if (!tracker.second.connected)
        {
            //Forcing the interaction profile of not connected trackers to NO_INTERACTION_PROFILE_TAG
            m_pimpl->top_level_paths[tracker.second.top_level_index].currentInteractionProfile = NO_INTERACTION_PROFILE_TAG;
        }
    }
}

void OpenXrInterface::render()
{
    for (size_t i = 0; i < m_pimpl->projection_view_swapchains.size(); ++i)
    {
        // Acquire swapchain images
        XrSwapchainImageAcquireInfo acquire_info = {.type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO,
                                                    .next = NULL};
        XrResult result = xrAcquireSwapchainImage(m_pimpl->projection_view_swapchains[i].swapchain, &acquire_info, &m_pimpl->projection_view_swapchains[i].acquired_index);
        if (!m_pimpl->checkXrOutput(result, "Failed to acquire swapchain image!"))
        {
            m_pimpl->closing = true;
            return;
        }

        XrSwapchainImageWaitInfo wait_info = {
            .type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO, .next = NULL, .timeout = 1000};
        result = xrWaitSwapchainImage(m_pimpl->projection_view_swapchains[i].swapchain, &wait_info);
        if (!m_pimpl->checkXrOutput(result, "Failed to wait for swapchain image!"))
        {
            m_pimpl->closing = true;
            return;
        }
    }

    for (size_t i = 0; i < m_pimpl->projection_view_depth_swapchains.size(); ++i)
    {
        XrSwapchainImageAcquireInfo depth_acquire_info = {
            .type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO, .next = NULL};
        XrResult result = xrAcquireSwapchainImage(m_pimpl->projection_view_depth_swapchains[i].swapchain, &depth_acquire_info,
                                         &m_pimpl->projection_view_depth_swapchains[i].acquired_index);
        if (!m_pimpl->checkXrOutput(result, "Failed to acquire depth swapchain image!"))
        {
            m_pimpl->closing = true;
            return;
        }

        XrSwapchainImageWaitInfo depth_wait_info = {
            .type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO, .next = NULL, .timeout = 1000};
        result = xrWaitSwapchainImage(m_pimpl->projection_view_depth_swapchains[i].swapchain, &depth_wait_info);
        if (!m_pimpl->checkXrOutput(result, "Failed to wait for depth swapchain image!"))
        {
            m_pimpl->closing = true;
            return;
        }
    }


    //-----------------------------------
    // Dummy rendering

    GLint ww, wh;
    glfwGetWindowSize(m_pimpl->window, &ww, &wh);

    //Left Eye

#ifdef DEBUG_RENDERING
    //Set green color
    glClearColor(0, 1, 0, 1);
#else
    glClearColor(0, 0, 0, 0);
#endif

    glBindFramebuffer(GL_FRAMEBUFFER, m_pimpl->glFrameBufferId);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_pimpl->projection_view_swapchains[0].
        swapchain_images[m_pimpl->projection_view_swapchains[0].acquired_index].image, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_pimpl->projection_view_depth_swapchains[0].
        swapchain_images[m_pimpl->projection_view_depth_swapchains[0].acquired_index].image, 0);

    //Clear the backgorund color
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, m_pimpl->projection_view_swapchain_create_info[0].width, m_pimpl->projection_view_swapchain_create_info[0].height);

    Eigen::Matrix4f leftEyePose = toEigen(m_pimpl->view_space_location.pose).inverse() * toEigen(m_pimpl->views[0].pose);


    for (auto& openGLLayer : m_pimpl->openGLQuadLayers)
    {
        if (openGLLayer->shouldRender() && (openGLLayer->visibility() == IOpenXrQuadLayer::Visibility::LEFT_EYE || openGLLayer->visibility() == IOpenXrQuadLayer::Visibility::BOTH_EYES))
        {
            openGLLayer->setFOVs(std::abs(m_pimpl->views[0].fov.angleLeft) + std::abs(m_pimpl->views[0].fov.angleDown), std::abs(m_pimpl->views[0].fov.angleUp) + std::abs(m_pimpl->views[0].fov.angleDown));
            if (!openGLLayer->offsetIsSet())
            {
                openGLLayer->setOffsetPosition(leftEyePose.block<3,1>(0, 3));
            }
            openGLLayer->render();
        }
    }

    glBlitNamedFramebuffer(m_pimpl->glFrameBufferId, 0,
                           0, 0, m_pimpl->projection_view_swapchain_create_info[0].width, m_pimpl->projection_view_swapchain_create_info[0].height,
                           0, 0, ww / 2, wh,
                           GL_COLOR_BUFFER_BIT, GL_NEAREST);

    //Right Eye
    glBindFramebuffer(GL_FRAMEBUFFER, m_pimpl->glFrameBufferId);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_pimpl->projection_view_swapchains[1].
        swapchain_images[m_pimpl->projection_view_swapchains[1].acquired_index].image, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_pimpl->projection_view_depth_swapchains[1].
        swapchain_images[m_pimpl->projection_view_depth_swapchains[1].acquired_index].image, 0);

#ifdef DEBUG_RENDERING
    //Set blue color
    glClearColor(0, 0, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //Restore default clear
    glClearColor(0, 0, 0, 0);
#else
    //Clear the backgorund color
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
#endif

    glViewport(0, 0, m_pimpl->projection_view_swapchain_create_info[1].width, m_pimpl->projection_view_swapchain_create_info[1].height);

    Eigen::Matrix4f rightEyePose = toEigen(m_pimpl->view_space_location.pose).inverse() * toEigen(m_pimpl->views[1].pose);

    for (auto& openGLLayer : m_pimpl->openGLQuadLayers)
    {
        if (openGLLayer->shouldRender() && (openGLLayer->visibility() == IOpenXrQuadLayer::Visibility::RIGHT_EYE || openGLLayer->visibility() == IOpenXrQuadLayer::Visibility::BOTH_EYES))
        {
            openGLLayer->setFOVs(std::abs(m_pimpl->views[1].fov.angleLeft) + std::abs(m_pimpl->views[1].fov.angleDown), std::abs(m_pimpl->views[1].fov.angleUp) + std::abs(m_pimpl->views[1].fov.angleDown));
            if (!openGLLayer->offsetIsSet())
            {
                openGLLayer->setOffsetPosition(rightEyePose.block<3,1>(0, 3));
            }
            openGLLayer->render();
        }
    }

    // Replicate swapchain on screen
    glBlitNamedFramebuffer(m_pimpl->glFrameBufferId, 0,
                           0, 0, m_pimpl->projection_view_swapchain_create_info[1].width, m_pimpl->projection_view_swapchain_create_info[1].height,
                           ww / 2 + 1, 0, ww, wh,
                           GL_COLOR_BUFFER_BIT, GL_NEAREST);

    //------------------------------
    glFramebufferTexture(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 0, 0);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);


    for (size_t i = 0; i < m_pimpl->projection_view_swapchains.size(); ++i)
    {
        // Release swapchains
        XrSwapchainImageReleaseInfo release_info = {.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO,
                                                    .next = NULL};
        XrResult result = xrReleaseSwapchainImage(m_pimpl->projection_view_swapchains[i].swapchain, &release_info);
        if (!m_pimpl->checkXrOutput(result, "Failed to release swapchain image!"))
        {
            m_pimpl->closing = true;
            return;
        }
    }

    for (size_t i = 0; i < m_pimpl->projection_view_depth_swapchains.size(); ++i)
    {
        XrSwapchainImageReleaseInfo depth_release_info = {
            .type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO, .next = NULL};
        XrResult result = xrReleaseSwapchainImage(m_pimpl->projection_view_depth_swapchains[i].swapchain, &depth_release_info);
        if (!m_pimpl->checkXrOutput(result, "Failed to release depth swapchain image!"))
        {
            m_pimpl->closing = true;
            return;
        }
    }

    glfwSwapBuffers(m_pimpl->window);

}

void OpenXrInterface::endXrFrame()
{
    m_pimpl->layer_count = 0; //Reset the layer count

    if (m_pimpl->frame_state.shouldRender) {

        // Here we can check m_pimpl->view_state.viewStateFlags to see if the orientation has been updated
        // in case this is used for the rendering
        m_pimpl->submitLayer((XrCompositionLayerBaseHeader*) & (m_pimpl->projection_layer)); //Submit the projection layer

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
    m_pimpl = std::make_unique<Implementation>();
}

OpenXrInterface::~OpenXrInterface()
{
    if (!m_pimpl->closed)
    {
        close();
    }
}

bool OpenXrInterface::initialize(const OpenXrInterfaceSettings &settings)
{
    if (m_pimpl->initialized)
    {
        yCError(OPENXRHEADSET) << "The OpenXr interface has been already initialized.";
        return false;
    }

    m_pimpl->locate_space_prediction_in_ns = static_cast<long>(std::round(settings.posesPredictionInMs * 1e6));

    m_pimpl->closing = false;
    m_pimpl->closed = false;

    bool ok = prepareXrInstance();
    ok = ok && prepareXrSystem();
    ok = ok && prepareGL();
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
    return m_pimpl->initialized;
}

void OpenXrInterface::draw()
{
    if (m_pimpl->closing)
    {
        return;
    }

    glfwPollEvents();

    pollXrEvents();
    if (m_pimpl->closing)
    {
        return;
    }

    if (startXrFrame()) {
        m_pimpl->locate_space_time = currentNanosecondsSinceEpoch() + m_pimpl->locate_space_prediction_in_ns;
        updateXrSpaces();
        updateXrActions();
        if (m_pimpl->frame_state.shouldRender) {
            render();
        }
        endXrFrame();
    }
}

std::shared_ptr<IOpenXrQuadLayer> OpenXrInterface::addHeadFixedQuadLayer()
{
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
        .sampleCount = std::max(m_pimpl->projection_view_swapchain_create_info[0].sampleCount,
        m_pimpl->projection_view_swapchain_create_info[1].sampleCount),
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
        .size = {.width = 1.5, .height = 1.5} //These numbers have been heuristically found
                                                //considering more or less the focal length of iCub cameras
                                                //fixing the layer at 1 meter distance.
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

std::shared_ptr<IOpenXrQuadLayer> OpenXrInterface::addHeadFixedOpenGLQuadLayer()
{
    if (!m_pimpl->initialized)
    {
        yCError(OPENXRHEADSET) << "The OpenXr interface has not been initialized.";
        return nullptr;
    }

    std::shared_ptr<OpenGLQuadLayer> newLayer = std::make_shared<OpenGLQuadLayer>();

    newLayer->initialize(std::min(m_pimpl->viewconfig_views[0].recommendedImageRectWidth, m_pimpl->viewconfig_views[1].recommendedImageRectWidth),
                         std::min(m_pimpl->viewconfig_views[0].recommendedImageRectHeight, m_pimpl->viewconfig_views[1].recommendedImageRectHeight));

    m_pimpl->openGLQuadLayers.push_back(newLayer);

    return newLayer;
}

bool OpenXrInterface::isRunning() const
{
    return m_pimpl->initialized && !m_pimpl->closing;
}

OpenXrInterface::Pose OpenXrInterface::headPose() const
{
    return XrSpaceLocationToPose(m_pimpl->view_space_location);
}

OpenXrInterface::Velocity OpenXrInterface::headVelocity() const
{
    return XrSpaceVelocityToVelocity(m_pimpl->view_space_velocity);
}

OpenXrInterface::Pose OpenXrInterface::leftHandPose() const
{
    const std::vector<PoseAction>& currentPoses = m_pimpl->top_level_paths[0].currentActions().poses;
    if (currentPoses.size() == 0) //no pose in the current interaction profile
    {
        return OpenXrInterface::Pose();
    }

    return currentPoses.front().pose;
}

OpenXrInterface::Velocity OpenXrInterface::leftHandVelocity() const
{
    const std::vector<PoseAction>& currentPoses = m_pimpl->top_level_paths[0].currentActions().poses;
    if (currentPoses.size() == 0) //no pose in the current interaction profile
    {
        return OpenXrInterface::Velocity();
    }

    return currentPoses.front().velocity;
}

OpenXrInterface::Pose OpenXrInterface::rightHandPose() const
{
    const std::vector<PoseAction>& currentPoses = m_pimpl->top_level_paths[1].currentActions().poses;
    if (currentPoses.size() == 0) //no pose in the current interaction profile
    {
        return OpenXrInterface::Pose();
    }

    return currentPoses.front().pose;
}

OpenXrInterface::Velocity OpenXrInterface::rightHandVelocity() const
{
    const std::vector<PoseAction>& currentPoses = m_pimpl->top_level_paths[1].currentActions().poses;
    if (currentPoses.size() == 0) //no pose in the current interaction profile
    {
        return OpenXrInterface::Velocity();
    }

    return currentPoses.front().velocity;
}

std::string OpenXrInterface::currentLeftHandInteractionProfile() const
{
    return m_pimpl->getInteractionProfileShortTag(m_pimpl->top_level_paths[0].currentInteractionProfile);
}

std::string OpenXrInterface::currentRightHandInteractionProfile() const
{
    return m_pimpl->getInteractionProfileShortTag(m_pimpl->top_level_paths[1].currentInteractionProfile);
}

void OpenXrInterface::getButtons(std::vector<bool> &buttons) const
{
    size_t numberOfButtons = 0;
    for (auto& topLevelpath : m_pimpl->top_level_paths)
    {
        numberOfButtons += topLevelpath.currentActions().buttons.size();
    }

    buttons.resize(numberOfButtons);

    size_t buttonsIndex = 0;

    for (auto& topLevelpath : m_pimpl->top_level_paths)
    {
        for (auto& button : topLevelpath.currentActions().buttons)
        {
            buttons[buttonsIndex] = button.value;
            buttonsIndex++;
        }
    }
}

void OpenXrInterface::getAxes(std::vector<float> &axes) const
{
    size_t numberOfAxes = 0;
    for (auto& topLevelpath : m_pimpl->top_level_paths)
    {
        numberOfAxes += topLevelpath.currentActions().axes.size();
    }

    axes.resize(numberOfAxes);

    size_t axisIndex = 0;

    for (auto& topLevelpath : m_pimpl->top_level_paths)
    {
        for (auto& axis : topLevelpath.currentActions().axes)
        {
            axes[axisIndex] = axis.value;
            axisIndex++;
        }
    }
}

void OpenXrInterface::getThumbsticks(std::vector<Eigen::Vector2f> &thumbsticks) const
{
    size_t numberOfThumbsticks = 0;
    for (auto& topLevelpath : m_pimpl->top_level_paths)
    {
        numberOfThumbsticks += topLevelpath.currentActions().thumbsticks.size();
    }

    thumbsticks.resize(numberOfThumbsticks);

    size_t thumbstickIndex = 0;

    for (auto& topLevelpath : m_pimpl->top_level_paths)
    {
        for (auto& thumbstick : topLevelpath.currentActions().thumbsticks)
        {
            thumbsticks[thumbstickIndex] = thumbstick.value;
            thumbstickIndex++;
        }
    }
}

void OpenXrInterface::getAllPoses(std::vector<NamedPoseVelocity> &additionalPoses) const
{
    size_t numberOfPoses = 0;
    numberOfPoses += 3; //This is because the head and the hands are added manually
    for (size_t topLevelIndex = 0; topLevelIndex <  m_pimpl->top_level_paths.size(); ++topLevelIndex)
    {
        if (topLevelIndex < 2) //Left and right hand are already considered with leftHandPose() and rightHandPose()
        {
            if (m_pimpl->top_level_paths[topLevelIndex].currentActions().poses.size() > 1) //The first element is the one considerd by leftHandPose() and rightHandPose()
            {
                numberOfPoses += m_pimpl->top_level_paths[topLevelIndex].currentActions().poses.size() - 1;
            }
        }
        else
        {
            numberOfPoses += m_pimpl->top_level_paths[topLevelIndex].currentActions().poses.size();
        }
    }

    additionalPoses.resize(numberOfPoses);

    size_t poseIndex = 0;

    auto& head = additionalPoses[poseIndex];
    head.name = "openxr_head";
    head.pose = headPose();
    head.velocity = headVelocity();
    poseIndex++;

    auto& left_arm = additionalPoses[poseIndex];
    left_arm.name = "openxr_left_hand";
    left_arm.pose = leftHandPose();
    left_arm.velocity = leftHandVelocity();
    poseIndex++;

    auto& right_arm = additionalPoses[poseIndex];
    right_arm.name = "openxr_right_hand";
    right_arm.pose = rightHandPose();
    right_arm.velocity = rightHandVelocity();
    poseIndex++;

    for (size_t topLevelIndex = 0; topLevelIndex <  m_pimpl->top_level_paths.size(); ++topLevelIndex)
    {
        std::vector<PoseAction>& posesList = m_pimpl->top_level_paths[topLevelIndex].currentActions().poses;
        for (size_t i = (topLevelIndex < 2) ? 1 : 0; i < posesList.size(); ++i) // if we are in the first or the second top level path (hence left or right hand), we start from the index 1 (the first one is the default)
        {
            additionalPoses[poseIndex] = posesList[i];
            poseIndex++;
        }
    }
}

int64_t OpenXrInterface::currentNanosecondsSinceEpoch() const
{
    //Updated when calling startXrFrame
    return m_pimpl->frame_state.predictedDisplayTime;
}

bool OpenXrInterface::shouldResetLocalReferenceSpace()
{
    bool shouldReset = m_pimpl->local_reference_space_changing &&
            m_pimpl->locate_space_time > m_pimpl->local_reference_space_change_time;

    if (shouldReset)
    {
        // This methods returns true only one
        m_pimpl->local_reference_space_changing = false;
    }

    return shouldReset;
}

void OpenXrInterface::close()
{
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

# ifndef WIN32
    if (m_pimpl->graphics_binding_gl.xDisplay)
    {
        XCloseDisplay(m_pimpl->graphics_binding_gl.xDisplay);
        m_pimpl->graphics_binding_gl.xDisplay = nullptr;
    }
#endif

    m_pimpl->initialized = false;

    yCInfo(OPENXRHEADSET) << "Closed";

    m_pimpl->closed = true;
}

OpenXrInterface::NamedPoseVelocity OpenXrInterface::NamedPoseVelocity::Identity(const std::string &name)
{
    OpenXrInterface::NamedPoseVelocity output;
    output.name = name;
    output.pose.positionValid = true;
    output.pose.position.setZero();
    output.pose.rotationValid = true;
    output.pose.rotation.setIdentity();
    output.velocity.linearValid = true;
    output.velocity.linear.setZero();
    output.velocity.angularValid = true;
    output.velocity.angular.setZero();

    return output;
}
