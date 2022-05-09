# yarp-device-openxrheadset

``yarp-device-openxrheadset`` is a [``YARP``](https://github.com/robotology/yarp) device allowing to use VR devices through [``OpenXR``](https://www.khronos.org/openxr/).

It can be used, for example, to display the images coming from a robot camera in VR, while retrieving the position and orientation of the headset and the joysticks used by the operator.

# Installation
## Windows

## Linux

# Running

# Reference frames

# Buttons order

# Configuration Parameters

- name
- gui_elements
    - width
    - height
    - x
    - y
    - z
    - opens port with name /name/gui_*

labels
    - width
    - height
    - x
    - y
    - z
    - opens port with name /name/label_*
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

# RPC interface

# Trackers
