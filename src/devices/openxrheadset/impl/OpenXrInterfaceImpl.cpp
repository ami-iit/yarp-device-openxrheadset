/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-2-Clause license. See the accompanying LICENSE file for details.
 */


#include <impl/OpenXrInterfaceImpl.h>

template<>
XrActionType Action<bool>::type() const
{
    return XR_ACTION_TYPE_BOOLEAN_INPUT;
}

template<>
XrActionType Action<float>::type() const
{
    return XR_ACTION_TYPE_FLOAT_INPUT;
}

template<>
XrActionType Action<Eigen::Vector2f>::type() const
{
    return XR_ACTION_TYPE_VECTOR2F_INPUT;
}

template<>
XrResult Action<bool>::update(XrSession session)
{
    XrActionStateBoolean action_state = {.type = XR_TYPE_ACTION_STATE_BOOLEAN, .next = NULL};
    XrActionStateGetInfo get_info = {.type = XR_TYPE_ACTION_STATE_GET_INFO,
                                     .next = NULL,
                                     .action = xrAction,
                                     .subactionPath = XR_NULL_PATH};

    XrResult output = xrGetActionStateBoolean(session, &get_info, &action_state);
    value = action_state.currentState;
    return output;
}

template<>
XrResult Action<float>::update(XrSession session)
{
    XrActionStateFloat action_state = {.type = XR_TYPE_ACTION_STATE_FLOAT, .next = NULL};
    XrActionStateGetInfo get_info = {.type = XR_TYPE_ACTION_STATE_GET_INFO,
                                     .next = NULL,
                                     .action = xrAction,
                                     .subactionPath = XR_NULL_PATH};

    XrResult output = xrGetActionStateFloat(session, &get_info, &action_state);
    value = action_state.currentState;
    return output;
}

template<>
XrResult Action<Eigen::Vector2f>::update(XrSession session)
{
    XrActionStateVector2f action_state = {.type = XR_TYPE_ACTION_STATE_VECTOR2F, .next = NULL};
    XrActionStateGetInfo get_info = {.type = XR_TYPE_ACTION_STATE_GET_INFO,
                                     .next = NULL,
                                     .action = xrAction,
                                     .subactionPath = XR_NULL_PATH};

    XrResult output = xrGetActionStateVector2f(session, &get_info, &action_state);
    value[0] = action_state.currentState.x;
    value[1] = action_state.currentState.y;
    return output;
}

XrResult PoseAction::create(XrSession session, XrActionSet actionSet, const std::string &inputName)
{
    name = inputName;
    XrActionCreateInfo action_info = {.type = XR_TYPE_ACTION_CREATE_INFO,
                                      .next = NULL,
                                      .actionType = XR_ACTION_TYPE_POSE_INPUT,
                                      .countSubactionPaths = 0,
                                      .subactionPaths = NULL};
    strcpy(action_info.actionName, name.c_str());
    strcpy(action_info.localizedActionName, name.c_str());

    XrResult result = xrCreateAction(actionSet, &action_info, &xrAction);

    if (!XR_SUCCEEDED(result))
    {
        return result;
    }

    XrPosef identity_pose =
    {
        .orientation = {.x = 0.0, .y = 0.0, .z = 0.0, .w = 1.0},
        .position = {.x = 0.0, .y = 0.0, .z = 0.0}
    };

    XrActionSpaceCreateInfo action_space_info = {.type = XR_TYPE_ACTION_SPACE_CREATE_INFO,
                                                 .next = NULL,
                                                 .action = xrAction,
                                                 .subactionPath = XR_NULL_PATH,
                                                 .poseInActionSpace = identity_pose};

    result = xrCreateActionSpace(session, &action_space_info, &xrSpace);

    return result;
}

XrResult PoseAction::update(XrSession session, XrSpace referenceSpace, XrTime time)
{
    XrActionStatePose action_state = {.type = XR_TYPE_ACTION_STATE_POSE, .next = NULL};
    XrActionStateGetInfo get_info = {.type = XR_TYPE_ACTION_STATE_GET_INFO,
                                     .next = NULL,
                                     .action = xrAction,
                                     .subactionPath = XR_NULL_PATH};

    XrResult result = xrGetActionStatePose(session, &get_info, &action_state);

    if (!XR_SUCCEEDED(result))
    {
        return result;
    }

    XrSpaceLocation xrSpaceLocation;
    XrSpaceVelocity xrSpaceVelocity;

    xrSpaceVelocity.type = XR_TYPE_SPACE_VELOCITY;
    xrSpaceVelocity.next = NULL;
    xrSpaceVelocity.velocityFlags = 0;
    xrSpaceLocation.type = XR_TYPE_SPACE_LOCATION;
    xrSpaceLocation.next = &xrSpaceVelocity;

    result = xrLocateSpace(xrSpace, referenceSpace, time,
                           &xrSpaceLocation);

    if (!XR_SUCCEEDED(result))
    {
        return result;
    }

    pose = XrSpaceLocationToPose(xrSpaceLocation);
    velocity = XrSpaceVelocityToVelocity(xrSpaceVelocity);

    return result;
}

OpenXrInterface::Pose XrSpaceLocationToPose(const XrSpaceLocation &spaceLocation)
{
    OpenXrInterface::Pose output;
    output.positionValid = spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT;
    output.position = toEigen(spaceLocation.pose.position);

    output.rotationValid = spaceLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
    output.rotation = toEigen(spaceLocation.pose.orientation);

    return output;
}

OpenXrInterface::Velocity XrSpaceVelocityToVelocity(const XrSpaceVelocity &spaceVelocity)
{
    OpenXrInterface::Velocity output;
    output.linearValid = spaceVelocity.velocityFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT;
    output.linear = toEigen(spaceVelocity.linearVelocity);

    output.angularValid = spaceVelocity.velocityFlags & XR_SPACE_VELOCITY_ANGULAR_VALID_BIT;
    output.angular = toEigen(spaceVelocity.angularVelocity);

    return output;
}

void InputActions::clear()
{
    buttons.clear();
    axes.clear();
    thumbsticks.clear();
}

XrBool32 OpenXrInterface::Implementation::OpenXrDebugCallback(XrDebugUtilsMessageSeverityFlagsEXT severity, XrDebugUtilsMessageTypeFlagsEXT, const XrDebugUtilsMessengerCallbackDataEXT *data, void *) {
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

void OpenXrInterface::Implementation::glfwErrorCallback(int error, const char *description)
{
    yCError(OPENXRHEADSET) << "GLFW Error:" << error << description;
}

void OpenXrInterface::Implementation::GLMessageCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei, const GLchar *message, const void *) {
    yCError(OPENXRHEADSET, "GL CALLBACK: %s source = 0x%x, type = 0x%x, id = 0x%x, severity = 0x%x, message = %s",
            ( type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : "" ),
            source, type, id, severity, message );
}

void OpenXrInterface::Implementation::submitLayer(const XrCompositionLayerBaseHeader *layer)
{
    layer_count++;
    if (layer_count > submitted_layers.size())
    {
        submitted_layers.resize(layer_count);
    }
    submitted_layers[layer_count-1] = layer;
}

bool OpenXrInterface::Implementation::suggestInteractionProfileBindings(const std::string &interactionProfileName,
                                                                        const std::vector<XrActionSuggestedBinding> &poseBindings,
                                                                        std::initializer_list<std::pair<const char*, const char*>> buttonsList,
                                                                        std::initializer_list<std::pair<const char*, const char*>> axisList,
                                                                        std::initializer_list<std::pair<const char*, const char*>> thumbStickList)
{
    std::vector<XrActionSuggestedBinding> bindings = poseBindings;
    InputActions& inputsEntry = inputActions[interactionProfileName];

    inputsEntry.clear();

    for (auto& input : buttonsList)
    {
        XrPath path;
        XrResult result = xrStringToPath(instance, input.first, &path);

        if (!checkXrOutput(result, "Failed to get path of %s for %s.", input.first, interactionProfileName.c_str()))
        {
            return false;
        }

        inputsEntry.buttons.emplace_back();
        result = inputsEntry.buttons.back().create(actionset, input.second);
        if (!checkXrOutput(result, "Failed to create action %s for %s.", input.second, interactionProfileName.c_str()))
        {
            return false;
        }
        bindings.emplace_back();
        bindings.back().action = inputsEntry.buttons.back().xrAction;
        bindings.back().binding = path;
    }

    for (auto& input : axisList)
    {
        XrPath path;
        XrResult result = xrStringToPath(instance, input.first, &path);

        if (!checkXrOutput(result, "Failed to get path of %s for %s.", input.first, interactionProfileName.c_str()))
        {
            return false;
        }

        inputsEntry.axes.emplace_back();
        result = inputsEntry.axes.back().create(actionset, input.second);
        if (!checkXrOutput(result, "Failed to create action %s for %s.", input.second, interactionProfileName.c_str()))
        {
            return false;
        }
        bindings.emplace_back();
        bindings.back().action = inputsEntry.axes.back().xrAction;
        bindings.back().binding = path;
    }

    for (auto& input : thumbStickList)
    {
        XrPath path;
        XrResult result = xrStringToPath(instance, input.first, &path);

        if (!checkXrOutput(result, "Failed to get path of %s for %s.", input.first, interactionProfileName.c_str()))
        {
            return false;
        }

        inputsEntry.thumbsticks.emplace_back();
        result = inputsEntry.thumbsticks.back().create(actionset, input.second);
        if (!checkXrOutput(result, "Failed to create action %s for %s.", input.second, interactionProfileName.c_str()))
        {
            return false;
        }
        bindings.emplace_back();
        bindings.back().action = inputsEntry.thumbsticks.back().xrAction;
        bindings.back().binding = path;
    }


    XrPath interaction_profile_path;
    XrResult result = xrStringToPath(instance, interactionProfileName.c_str(),
                                     &interaction_profile_path);
    if (!checkXrOutput(result, "Failed to get the path of the interaction profile %s.", interactionProfileName.c_str()))
        return false;

    const XrInteractionProfileSuggestedBinding suggested_bindings = {
        .type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING,
        .next = NULL,
        .interactionProfile = interaction_profile_path,
        .countSuggestedBindings = static_cast<uint32_t>(bindings.size()),
        .suggestedBindings = bindings.data()};

    result = xrSuggestInteractionProfileBindings(instance, &suggested_bindings);
    if (!checkXrOutput(result, "Failed to suggest bindings for %s.", interactionProfileName.c_str()))
        return false;

    return true;
}



