# lunabot_nav

This package contains action servers and clients required for autonomous navigation and operation.

## Source Files

### Assisted Servers (Teleop)
- **assisted_excavation_server.cpp**: Assisted excavation server for teleop that controls bucket actuators and manages excavation workflow with vibration.
- **assisted_depositing_server.cpp**: Assisted depositing server for teleop that controls bucket actuators for depositing operations.

### Autonomous Servers
- **auto_excavation_server.cpp**: Autonomous excavation server that drives forward with minimal vibration for quick excavation passes.
- **auto_depositing_server.cpp**: Autonomous depositing server that drives forward to position over construction zone before depositing.

### Simulation Servers
- **excavation_server_sim.cpp**: Simulation-compatible excavation server that controls bucket joint position.
- **depositing_server_sim.cpp**: Simulation-compatible depositing server that controls bucket joint position.

### Navigation Client
- **navigation_client.cpp**: Runs autonomous one cycle sequence: navigation, excavation, and depositing actions.