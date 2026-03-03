# Power monitor telemetry message
# Published by power_monitor node

std_msgs/Header header

# Device identification
uint8 device_id

# Electrical measurements
float32 voltage        # Bus voltage (V)
float32 current        # Current draw (A)
float32 power          # Power consumption (W)
float32 temperature    # Temperature (°C)

# Energy integration
float32 energy_kwh     # Total energy consumed (kWh)

# Alert flags
uint16 alert_flags
uint16 ALERT_TEMP_OVER_LIMIT     = 0x80
uint16 ALERT_CURRENT_OVER_LIMIT  = 0x40
uint16 ALERT_CURRENT_UNDER_LIMIT = 0x20
uint16 ALERT_BUS_OVER_LIMIT      = 0x10
uint16 ALERT_BUS_UNDER_LIMIT     = 0x08
uint16 ALERT_POWER_OVER_LIMIT    = 0x04

# Human-readable alert status
string alert_status
