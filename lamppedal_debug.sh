# A debug configuration script using st-util and gdb 
#   Developed and tested on Linux

echo "Starting ST-Link Server"
st-util &
ST_UTIL_PID=$!

# Connect to ST Link Server with GDB w/ configuration from debug command file
arm-none-eabi-gdb -x debug_commands 

echo "Shutting down ST-Link Server"
kill $ST_UTIL_PID 
