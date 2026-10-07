# Motor.mk
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
C_INCLUDES += -I$(FOCAlgorithmDir)/Drivers/Generic
C_INCLUDES += -I$(FOCAlgorithmDir)/Drivers/ST
C_INCLUDES += -I$(FOCAlgorithmDir)/Drivers/ST/LED
C_INCLUDES += -I$(FOCAlgorithmDir)/Utils
C_INCLUDES += -I$(FOCAlgorithmDir)/Sensor
C_INCLUDES += -I$(FOCAlgorithmDir)/Boards

CPP_SOURCES +=  \
$(wildcard $(FOCAlgorithmDir)/Drivers/Generic/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Drivers/ST/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Drivers/ST/LED/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Control/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Control/PID/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Control/Interboard/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Sensor/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Utils/*.cpp) \
$(wildcard $(FOCAlgorithmDir)/Boards/*.cpp)

