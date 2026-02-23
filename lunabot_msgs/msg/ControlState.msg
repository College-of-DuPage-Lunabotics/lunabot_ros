# Robot control state message
# Published by controller_teleop and GUI

# Control mode
uint8 MODE_MANUAL = 0
uint8 MODE_AUTO = 1
uint8 mode

# System states (true = enabled)
bool hardware_enabled
bool pointlio_enabled
bool mapping_enabled  
bool nav2_enabled
bool localization_enabled

# Active operations
bool is_excavating
bool is_dumping
bool is_navigating
bool emergency_stop

# Additional status
string status_message
