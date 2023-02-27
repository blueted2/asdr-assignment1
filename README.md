# Packages

| Package | Description |
| --- | --- |
| [light_position](src/light_position/) | Contains nodes for detecting the light's position. |
| [light_follow](src/light_follow/) | Contains nodes for controlling Jiwy and following the light. |


## light_position

### Nodes

| Node | Description |
| --- | --- |
| [brightness](src/light_position/src/brightness.cpp) | Subscribes to the camera image and determines whether the light is on or off. |
| [fake_light](src/light_position/src/fake_light.cpp) | Publishes an image with a fake light that moves around. Useful for testing. |
| [light_position](src/light_position/src/light_position.cpp) | Subscribes to the camera image and determines the position of the light. Gives the light's position both in pixel coordiantes and a centered-normalized offset. |
| [show_light_position](src/light_position/src/show_light_position.cpp) | Subscribes to the camera image and the light position and displays the light's position on the image. |

### Launch

| Launch | Description |
| --- | --- |
| [show_position](src/light_position/launch/show_position.yaml) | Visualize the light position detection feature. |


## light_follow

| Node | Description |
| --- | --- |
| [closed_light_follow](src/light_follow/src/closed_light_follow.cpp) | Subscribes to the light position and Jiwy's moving camera and controls Jiwy to follow the light in a closed-loop system. |
| [open_light_follow](src/light_follow/src/open_light_follow.cpp) | Subscribes to the light position and the static camera and controls Jiwy to follow the light in an open-loop system. |
| [setpoint_generator](src/light_follow/src/setpoint_generator.cpp) | Cycles between a number of setpoints for Jiwy to follow. |
| [show_jiwy_pos](src/light_follow/src/show_jiwy_pos.cpp) | Subscribes to Jiwy's position and the static camera, and overlays Jiwy field of view. |

### Launch
| Launch | Description |
| --- | --- |
| [setpoint_test](src/light_follow/launch/setpoint_test.yaml) | Run the setpoint generator and watch Jiwy move towards the points. |
| [open](src/light_follow/launch/open.yaml) | Run the open-loop light following system. |
| [closed](src/light_follow/launch/closed.yaml) | Run the closed-loop light following system. |

