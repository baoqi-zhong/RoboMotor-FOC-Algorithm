# Motor.mk
#
# @author:
#	- baoqi-zhong (zzhongas@connect.ust.hk)
#
# RoboMotor FOC Algorithm Library Makefile
# ----------------

C_INCLUDES += \
-I$(FOCAlgorithmDir) \
-I$(FOCAlgorithmDir)/Control \
-I$(FOCAlgorithmDir)/Control/PID \
-I$(FOCAlgorithmDir)/Control/Interboard \
-I$(FOCAlgorithmDir)/Drivers/Generic \
-I$(FOCAlgorithmDir)/Drivers/ST \
-I$(FOCAlgorithmDir)/Drivers/ST/LED \
-I$(FOCAlgorithmDir)/HAL \
-I$(FOCAlgorithmDir)/Utils \
-I$(FOCAlgorithmDir)/Sensor

CPP_SOURCES +=  \
$(wildcard $(FOCAlgorithmDir)/Control/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Control/PID/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Control/Interboard/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Drivers/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Drivers/Generic/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Drivers/ST/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Drivers/ST/LED/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Utils/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Sensor/*.cpp)
