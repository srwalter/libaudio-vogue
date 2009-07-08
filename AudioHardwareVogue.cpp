/*
**
** Copyright 2007, The Android Open Source Project
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/

#include <stdint.h>
#include <sys/types.h>
#include <linux/msm_audio.h>

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#define LOG_TAG "AudioHardware"
#include <utils/Log.h>
#include <utils/String8.h>

#include "AudioHardwareVogue.h"

namespace android {

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------

AudioHardwareVogue::AudioHardwareVogue()
    : mOutput(0), mInput(0),  mFd(-1), mFdIn(-1), mMicMute(false)
{
    mFd = ::open("/dev/msm_pcm_out", O_RDWR);
    mFdIn = ::open("/dev/msm_pcm_in", O_RDWR);
}

AudioHardwareVogue::~AudioHardwareVogue()
{
    if (mFd >= 0) ::close(mFd);
    if (mFdIn >= 0) ::close(mFdIn);
    delete mOutput;
    delete mInput;
}

status_t AudioHardwareVogue::initCheck()
{
    if (mFd >= 0) {
        if (::access("/dev/msm_pcm_out", O_RDWR) == NO_ERROR)
            return NO_ERROR;
    }
    if (mFdIn >= 0) {
        if (::access("/dev/msm_pcm_in", O_RDWR) == NO_ERROR)
            return NO_ERROR;
    }
    return NO_INIT;
}

AudioStreamOut* AudioHardwareVogue::openOutputStream(
        int format, int channelCount, uint32_t sampleRate, status_t *status)
{
    AutoMutex lock(mLock);

    // only one output stream allowed
    if (mOutput) {
        if (status) {
            *status = INVALID_OPERATION;
        }
        return 0;
    }

    // create new output stream
    AudioStreamOutVogue* out = new AudioStreamOutVogue();
    status_t lStatus = out->set(this, mFd, format, channelCount, sampleRate);
    if (status) {
        *status = lStatus;
    }
    if (lStatus == NO_ERROR) {
        mOutput = out;
    } else {
        delete out;
    }
    return mOutput;
}

void AudioHardwareVogue::closeOutputStream(AudioStreamOutVogue* out) {
    if (out == mOutput) mOutput = 0;
}

AudioStreamIn* AudioHardwareVogue::openInputStream(
        int format, int channelCount, uint32_t sampleRate, status_t *status,
        AudioSystem::audio_in_acoustics acoustics)
{
    AutoMutex lock(mLock);

    // only one input stream allowed
    if (mInput) {
        if (status) {
            *status = INVALID_OPERATION;
        }
        return 0;
    }

    // create new output stream
    AudioStreamInVogue* in = new AudioStreamInVogue();
    status_t lStatus = in->set(this, mFdIn, format, channelCount, sampleRate, acoustics);
    if (status) {
        *status = lStatus;
    }
    if (lStatus == NO_ERROR) {
        mInput = in;
    } else {
        delete in;
    }
    return mInput;
}

void AudioHardwareVogue::closeInputStream(AudioStreamInVogue* in) {
    if (in == mInput) mInput = 0;
}

status_t AudioHardwareVogue::setVoiceVolume(float v)
{
    unsigned scaled_vol = v * (float)0x3000;
    int rc;

    rc = ioctl(mFd, AUDIO_SET_VOLUME, scaled_vol);
    if (rc < 0)
        return UNKNOWN_ERROR;
    return NO_ERROR;
}

status_t AudioHardwareVogue::setMasterVolume(float v)
{
    // Implement: set master volume
    // return error - software mixer will handle it
    return INVALID_OPERATION;
}

status_t AudioHardwareVogue::setMicMute(bool state)
{
    mMicMute = state;
    return NO_ERROR;
}

status_t AudioHardwareVogue::getMicMute(bool* state)
{
    *state = mMicMute;
    return NO_ERROR;
}

size_t AudioHardwareVogue::getInputBufferSize(uint32_t rate, 
        int format, int chan)
{
    unsigned bytes_per_sample = 1;

    switch (format) {
        case AudioSystem::PCM_8_BIT:
            bytes_per_sample *= 1;
            break;

        case AudioSystem::PCM_16_BIT:
        case AudioSystem::FORMAT_DEFAULT:
            bytes_per_sample *= 2;
            break;
    }

    return 2048 / bytes_per_sample;
}

status_t AudioHardwareVogue::dumpInternals(int fd, const Vector<String16>& args)
{
    const size_t SIZE = 256;
    char buffer[SIZE];
    String8 result;
    result.append("AudioHardwareVogue::dumpInternals\n");
    snprintf(buffer, SIZE, "\tmFd: %d mMicMute: %s\n",  mFd, mMicMute? "true": "false");
    result.append(buffer);
    ::write(fd, result.string(), result.size());
    return NO_ERROR;
}

status_t AudioHardwareVogue::dump(int fd, const Vector<String16>& args)
{
    dumpInternals(fd, args);
    if (mInput) {
        mInput->dump(fd, args);
    }
    if (mOutput) {
        mOutput->dump(fd, args);
    }
    return NO_ERROR;
}

// ----------------------------------------------------------------------------

status_t AudioStreamOutVogue::set(
        AudioHardwareVogue *hw,
        int fd,
        int format,
        int channels,
        uint32_t rate)
{
    // fix up defaults
    if (format == 0) format = AudioSystem::PCM_16_BIT;
    if (channels == 0) channels = channelCount();
    if (rate == 0) rate = sampleRate();

    // check values
    if ((format != AudioSystem::PCM_16_BIT) ||
            (channels != channelCount()) ||
            (rate != sampleRate()))
        return BAD_VALUE;

    mAudioHardware = hw;
    mFd = fd;
    return NO_ERROR;
}

AudioStreamOutVogue::~AudioStreamOutVogue()
{
    if (mAudioHardware)
        mAudioHardware->closeOutputStream(this);
}

ssize_t AudioStreamOutVogue::write(const void* buffer, size_t bytes)
{
    Mutex::Autolock _l(mLock);
    return ssize_t(::write(mFd, buffer, bytes));
}

status_t AudioStreamOutVogue::standby()
{
    // Implement: audio hardware to standby mode
    return NO_ERROR;
}

status_t AudioStreamOutVogue::dump(int fd, const Vector<String16>& args)
{
    const size_t SIZE = 256;
    char buffer[SIZE];
    String8 result;
    snprintf(buffer, SIZE, "AudioStreamOutVogue::dump\n");
    result.append(buffer);
    snprintf(buffer, SIZE, "\tsample rate: %d\n", sampleRate());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tbuffer size: %d\n", bufferSize());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tchannel count: %d\n", channelCount());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tformat: %d\n", format());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmAudioHardware: %p\n", mAudioHardware);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmFd: %d\n", mFd);
    result.append(buffer);
    ::write(fd, result.string(), result.size());
    return NO_ERROR;
}

// ----------------------------------------------------------------------------

// record functions
status_t AudioStreamInVogue::set(
        AudioHardwareVogue *hw,
        int fd,
        int format,
        int channels,
        uint32_t rate,
        AudioSystem::audio_in_acoustics acoustics)
{
    // FIXME: remove logging
    LOGD("AudioStreamInVogue::set(%p, %d, %d, %d, %u)", hw, fd, format, channels, rate);
    // check values
    if ((format != AudioSystem::PCM_16_BIT) ||
            (channels != channelCount()) ||
            (rate != sampleRate())) {
        LOGE("Error opening input channel");
        return BAD_VALUE;
    }

    mAudioHardware = hw;
    mFd = fd;
    return NO_ERROR;
}

AudioStreamInVogue::~AudioStreamInVogue()
{
    // FIXME: remove logging
    LOGD("AudioStreamInVogue destructor");
    if (mAudioHardware)
        mAudioHardware->closeInputStream(this);
}

ssize_t AudioStreamInVogue::read(void* buffer, ssize_t bytes)
{
    // FIXME: remove logging
    LOGD("AudioStreamInVogue::read(%p, %d) from fd %d", buffer, bytes, mFd);
    AutoMutex lock(mLock);
    if (mFd < 0) {
        LOGE("Attempt to read from unopened device");
        return NO_INIT;
    }
    return ::read(mFd, buffer, bytes);
}

status_t AudioStreamInVogue::dump(int fd, const Vector<String16>& args)
{
    const size_t SIZE = 256;
    char buffer[SIZE];
    String8 result;
    snprintf(buffer, SIZE, "AudioStreamInVogue::dump\n");
    result.append(buffer);
    snprintf(buffer, SIZE, "\tsample rate: %d\n", sampleRate());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tbuffer size: %d\n", bufferSize());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tchannel count: %d\n", channelCount());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tformat: %d\n", format());
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmAudioHardware: %p\n", mAudioHardware);
    result.append(buffer);
    snprintf(buffer, SIZE, "\tmFd: %d\n", mFd);
    result.append(buffer);
    ::write(fd, result.string(), result.size());
    return NO_ERROR;
}

AudioHardwareInterface *createAudioHardware()
{
    return new AudioHardwareVogue();
}

// ----------------------------------------------------------------------------

}; // namespace android
