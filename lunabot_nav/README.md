# lunabot_nav

This package contains action servers and clients required for autonomous navigation.

## Source Files
- **excavation_server.cpp**: Performs excavation sequence after receiving request from the navigation client.
- **localization_server.cpp**: Provides localization using AprilTags and manages an action server for localization feedback.
- **navigation_client.cpp**: Handles localization responses and sends navigation and excavation requests.
