Header header

# MASTER or SLAVE
string type
# Interlock
bool interlock
# Automatic Mode
bool mode_auto
# Manual Mode
bool mode_manual
# Active if fault
bool on_fault
# Fault code
int32 fault_code
# Fault code description
string fault_description
# Amps
float32 current_rms
# Amps
float32 battery_current
# Volts
float32 keyswitch_voltage
# %
float32 bdi_percentage
# RPM
int32 motor_rpm
# Km/h
float32 speed
# C
float32 controller_temp
# C
float32 motor_temp
# % Desired throttle 
float32 desired_throttle

# Work modes
int32 MODE_THROTTLE=0
int32 MODE_TWIST=1
