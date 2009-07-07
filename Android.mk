LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE := libaudio
LOCAL_PRELINK_MODULE := false

LOCAL_SRC_FILES += \
    AudioHardwareVogue.cpp

LOCAL_SHARED_LIBRARIES := \
    libcutils \
    libutils \

LOCAL_STATIC_LIBRARIES := \
    libaudiointerface

include $(BUILD_SHARED_LIBRARY)
