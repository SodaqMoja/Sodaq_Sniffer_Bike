/*
Copyright (c) 2016-18, SODAQ
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>
#include <Arduino.h>

typedef void(*VoidCallbackMethodPtr)(void);

struct ConfigParams
{
    uint16_t _header;
    uint16_t _gpsFixTimeout;

    uint8_t _accelerationPercentage;
    uint8_t _accelerationDuration;
    uint8_t _onTheMoveTimeout;

    char _apn[32 + 1];
    char _cdp[32 + 1];
    char _forceOperator[32 + 1];
    uint8_t _cid;

    uint8_t _band;

    char _targetIP[16]; // 4x3 = 12, + 3 dots, + nullchar = 16
    uint16_t _targetPort;

    uint8_t _gpsMinSatelliteCount;
    
    uint8_t _isDebugOn;
    uint8_t _sensorsSampleInterval;

    uint16_t _crc16;

public:
    void read();
    void commit(bool forced = false);
    void reset();

    bool execCommand(const char* line);

    uint16_t getGpsFixTimeout() const { return _gpsFixTimeout; }

    uint8_t getAccelerationPercentage() const { return _accelerationPercentage; }
    uint8_t getAccelerationDuration() const { return _accelerationDuration; }
    uint8_t getOnTheMoveTimeout() const { return _onTheMoveTimeout; }
    
    const char* getApn() const { return _apn; }
    const char* getCdp() const { return _cdp; }
    const char* getForceOperator() const { return _forceOperator; }
    uint8_t getCid() const { return _cid; }

    uint8_t getBand() const { return _band; }

    const char* getTargetIP() const { return _targetIP; }

    uint16_t getTargetPort() const { return _targetPort; }
    
    uint8_t getGpsMinSatelliteCount() const { return _gpsMinSatelliteCount; }
    
    uint8_t getIsDebugOn() const { return _isDebugOn; }
    uint8_t getSensorsSampleInterval() const { return _sensorsSampleInterval; }

    static void showConfig(Stream* stream);
    bool checkConfig(Stream& stream);
    void setConfigResetCallback(VoidCallbackMethodPtr callback);
};

extern ConfigParams params;

#endif
