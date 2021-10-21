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

bool OpenXrInterface::Implementation::fillActionBindings(const std::vector<InteractionProfileDeclaration> &interactionProfilesPrefixes, const std::vector<TopLevelPathDeclaration> &topLevelPaths)
{

    //Create the actionset
    XrActionSetCreateInfo actionset_info = {
        .type = XR_TYPE_ACTION_SET_CREATE_INFO, .next = NULL, .priority = 0};
    strcpy(actionset_info.actionSetName, "yarp_device_actionset");
    strcpy(actionset_info.localizedActionSetName, "YARP Device Actions");

    XrResult result = xrCreateActionSet(instance, &actionset_info, &actionset);
    if (checkXrOutput(result, "Failed to create actionset"))
        return false;

    //Allocate all the top level paths
    top_level_paths.clear();
    for (auto& level : topLevelPaths)
    {
        top_level_paths.emplace_back();
        TopLevelPath& added = top_level_paths.back();
        added.stringPath = level.stringPath;
        XrResult result = xrStringToPath(instance, added.stringPath.c_str(), &added.xrPath);

        if (!checkXrOutput(result, "Failed to get path of %s.", added.stringPath.c_str()))
        {
            return false;
        }
        added.interactionProfileActions.clear();
    }

    // Parse all the poses, buttons, axes and thumbsticks for each interaction profile
    for (auto& interactionProfile : interactionProfilesPrefixes)
    {
        std::vector<XrActionSuggestedBinding> bindings;
        const std::string& profileName = interactionProfile.path;
        const std::string& profilePrefix = interactionProfile.actionNamePrefix;


        for (size_t i = 0; i < top_level_paths.size(); ++i)
        {
            const TopLevelPathDeclaration& currentTopLevelPathDeclaration = topLevelPaths[i];
            const std::string& pathPrefix = currentTopLevelPathDeclaration.stringPath;
            const std::string& pathActionNamePrefix = currentTopLevelPathDeclaration.actionNamePrefix;
            auto actionDeclarationIterator = currentTopLevelPathDeclaration.inputsDeclarations.find(profileName);
            if (actionDeclarationIterator != currentTopLevelPathDeclaration.inputsDeclarations.end())
            {
                const InputActionsDeclaration& actionDeclaration = actionDeclarationIterator->second;
                InputActions& inputsEntry = top_level_paths[i].interactionProfileActions[profileName];
                inputsEntry.clear();


                for (auto& input : actionDeclaration.poses)
                {
                    XrPath xrPath;
                    std::string fullActionPath = pathPrefix + input.path;
                    std::string fullActionName = profilePrefix + pathActionNamePrefix + input.nameSuffix;
                    XrResult result = xrStringToPath(instance, fullActionPath.c_str(), &xrPath);

                    if (!checkXrOutput(result, "Failed to get path of %s for %s.", fullActionPath.c_str(), profileName.c_str()))
                    {
                        return false;
                    }

                    inputsEntry.poses.emplace_back();
                    PoseAction& newAction = inputsEntry.poses.back();
                    result = newAction.create(session, actionset, fullActionName);
                    if (!checkXrOutput(result, "Failed to create action %s for %s.", fullActionName.c_str(), profileName.c_str()))
                    {
                        return false;
                    }
                    bindings.emplace_back();
                    bindings.back().action = newAction.xrAction;
                    bindings.back().binding = xrPath;
                }

                for (auto& input : actionDeclaration.buttons)
                {
                    XrPath xrPath;
                    std::string fullActionPath = pathPrefix + input.path;
                    std::string fullActionName = profilePrefix + pathActionNamePrefix + input.nameSuffix;
                    XrResult result = xrStringToPath(instance, fullActionPath.c_str(), &xrPath);

                    if (!checkXrOutput(result, "Failed to get path of %s for %s.", fullActionPath.c_str(), profileName.c_str()))
                    {
                        return false;
                    }

                    inputsEntry.buttons.emplace_back();
                    result = inputsEntry.buttons.back().create(actionset, fullActionName);
                    if (!checkXrOutput(result, "Failed to create action %s for %s.", fullActionName.c_str(), profileName.c_str()))
                    {
                        return false;
                    }
                    bindings.emplace_back();
                    bindings.back().action = inputsEntry.buttons.back().xrAction;
                    bindings.back().binding = xrPath;
                }

                for (auto& input : actionDeclaration.axes)
                {
                    XrPath xrPath;
                    std::string fullActionPath = pathPrefix + input.path;
                    std::string fullActionName = profilePrefix + pathActionNamePrefix + input.nameSuffix;
                    XrResult result = xrStringToPath(instance, fullActionPath.c_str(), &xrPath);

                    if (!checkXrOutput(result, "Failed to get path of %s for %s.", fullActionPath.c_str(), profileName.c_str()))
                    {
                        return false;
                    }

                    inputsEntry.axes.emplace_back();
                    result = inputsEntry.axes.back().create(actionset, fullActionName);
                    if (!checkXrOutput(result, "Failed to create action %s for %s.", fullActionName.c_str(), profileName.c_str()))
                    {
                        return false;
                    }
                    bindings.emplace_back();
                    bindings.back().action = inputsEntry.axes.back().xrAction;
                    bindings.back().binding = xrPath;
                }

                for (auto& input : actionDeclaration.thumbsticks)
                {
                    XrPath xrPath;
                    std::string fullActionPath = pathPrefix + input.path;
                    std::string fullActionName = profilePrefix + pathActionNamePrefix + input.nameSuffix;
                    XrResult result = xrStringToPath(instance, fullActionPath.c_str(), &xrPath);

                    if (!checkXrOutput(result, "Failed to get path of %s for %s.", fullActionPath.c_str(), profileName.c_str()))
                    {
                        return false;
                    }

                    inputsEntry.thumbsticks.emplace_back();
                    result = inputsEntry.thumbsticks.back().create(actionset, fullActionName);
                    if (!checkXrOutput(result, "Failed to create action %s for %s.", fullActionName.c_str(), profileName.c_str()))
                    {
                        return false;
                    }
                    bindings.emplace_back();
                    bindings.back().action = inputsEntry.thumbsticks.back().xrAction;
                    bindings.back().binding = xrPath;
                }
            }
        }

        // Suggest the bindings to OpenXR
        XrPath interaction_profile_path;
        XrResult result = xrStringToPath(instance, profileName.c_str(),
                                         &interaction_profile_path);
        if (!checkXrOutput(result, "Failed to get the path of the interaction profile %s.", profileName.c_str()))
            return false;

        const XrInteractionProfileSuggestedBinding suggested_bindings = {
            .type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING,
            .next = NULL,
            .interactionProfile = interaction_profile_path,
            .countSuggestedBindings = static_cast<uint32_t>(bindings.size()),
            .suggestedBindings = bindings.data()};

        result = xrSuggestInteractionProfileBindings(instance, &suggested_bindings);
        if (!checkXrOutput(result, "Failed to suggest bindings for %s.", profileName.c_str()))
            return false;

    }

    return true;
}

std::string OpenXrInterface::Implementation::getInteractionProfileShortTag(const std::string &interactionProfile)
{
    if (interactionProfile == KHR_SIMPLE_CONTROLLER_INTERACTION_PROFILE)
    {
        return "khr_simple_controller";
    }

    if (interactionProfile == HTC_VIVE_INTERACTION_PROFILE_TAG)
    {
        return "htc_vive_controller";
    }

    if (interactionProfile == OCULUS_TOUCH_INTERACTION_PROFILE_TAG)
    {
        return "oculus_touch_controller";
    }

    if (interactionProfile == HTC_VIVE_TRACKER_INTERACTION_PROFILE_TAG)
    {
        return "htc_vive_tracker";
    }

    return "none";
}


InputActions &TopLevelPath::currentActions()
{
    return interactionProfileActions[currentInteractionProfile];
}
