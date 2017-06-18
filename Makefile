#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := smart-clock

include $(IDF_PATH)/make/project.mk

CPPFLAGS += -DARCH_ESP32
