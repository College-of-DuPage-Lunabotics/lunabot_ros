# lunabot_nav

This package contains action servers and clients required for autonomous navigation and operation.

## Source Files

### Hardware Servers
- **excavation_server.cpp**: Hardware excavation server that controls bucket actuators and manages excavation workflow.
- **dumping_server.cpp**: Hardware dumping server that controls bucket actuators for dumping operations.
- **homing_server.cpp**: Hardware homing server that establishes actuator zero position at full extension.

### Simulation Servers
- **excavation_server_sim.cpp**: Simulation-compatible excavation server that controls bucket joint position.
- **dumping_server_sim.cpp**: Simulation-compatible dumping server that controls bucket joint position.
- **homing_server_sim.cpp**: Simulation-compatible homing server that sets bucket to known home position.

### Navigation and Localization
- **localization_server.cpp**: Provides localization using AprilTags and manages an action server for localization feedback.
- **navigation_client.cpp**: Runs autonomous one cycle sequence: localization, navigation, excavation, and dumping actions.