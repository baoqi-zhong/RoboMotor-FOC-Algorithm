# MotorControl.mk
#
# @author:
#	- baoqi-zhong (zzhongas@connect.ust.hk)
#
# RoboMotor FOC Algorithm Library Makefile
# ----------------

C_INCLUDES += -I$(FOCAlgorithmDir)
C_INCLUDES += -I$(FOCAlgorithmDir)/Control
C_INCLUDES += -I$(FOCAlgorithmDir)/Control/PID
C_INCLUDES += -I$(FOCAlgorithmDir)/Control/Interboard
C_INCLUDES += -I$(FOCAlgorithmDir)/Drivers
C_INCLUDES += -I$(FOCAlgorithmDir)/Drivers/LED
C_INCLUDES += -I$(FOCAlgorithmDir)/Utils
C_INCLUDES += -I$(FOCAlgorithmDir)/Sensor
C_INCLUDES += -I$(FOCAlgorithmDir)/Boards

CPP_SOURCES +=  \
$(wildcard $(FOCAlgorithmDir)/Drivers/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Drivers/LED/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Control/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Control/PID/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Control/Interboard/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Sensor/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Utils/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Boards/*.cpp)

