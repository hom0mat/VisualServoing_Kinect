# This format is suitable for the following services
#   - set_vacuum_gripper

bool on
bool wait               false
float32 timeout         3
float32 delay_sec       0
# sync: whether to execute in the motion queue 
bool sync               true
# hardware_version==1: Plug-in Connection, default
# hardware_version==2: Contact Connection
# default is 0, equivalent to hardware_version==1 at first
# when the hardware_version parameter is specified as 1 or 2, the value will be recorded in the current environment.
# If the parameter is not specified in the next call or call the get_vacuum_gripper, the previous value will be used.
int16 hardware_version  0

