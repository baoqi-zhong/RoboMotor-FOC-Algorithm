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
-I$(FOCAlgorithmDir)/Drivers \
-I$(FOCAlgorithmDir)/HAL \
-I$(FOCAlgorithmDir)/Utils \
-I$(FOCAlgorithmDir)/Sensor

CPP_SOURCES +=  \
$(wildcard $(FOCAlgorithmDir)/Control/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Control/PID/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Control/Interboard/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Drivers/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Utils/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Sensor/*.cpp)
