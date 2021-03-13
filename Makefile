#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := jpgcam

SOLUTION_PATH ?= $(abspath $(shell pwd))

include $(IDF_PATH)/make/project.mk

