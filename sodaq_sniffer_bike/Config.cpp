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

#include "Config.h"
#include "Command.h"
#include "FlashStorage.h"

#define DEFAULT_HEADER 0xBEEF

ConfigParams params;

FlashStorage(flash, ConfigParams);
static bool needsCommit;
static VoidCallbackMethodPtr configResetCallback;

static uint16_t crc16ccitt(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0;
    while (len--) {
        crc ^= (*buf++ << 8);
        for (uint8_t i = 0; i < 8; ++i) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            }
            else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

void ConfigParams::read()
{
    flash.read(this);

    // check header and CRC
    uint16_t calcCRC16 = crc16ccitt((uint8_t*)this, (uint32_t)&params._crc16 - (uint32_t)&params._header);
    if (_header != DEFAULT_HEADER || _crc16 != calcCRC16) {
        reset();
    }
}

void ConfigParams::reset()
{
    _gpsFixTimeout = 120;

    memset(_apn, 0x30, sizeof(_apn) - 1);
    _apn[sizeof(_apn) - 1] = '\0';

    memset(_cdp, 0x30, sizeof(_cdp) - 1);
    _cdp[sizeof(_cdp) - 1] = '\0';

    memset(_forceOperator, 0x30, sizeof(_forceOperator) - 1);
    _forceOperator[sizeof(_forceOperator) - 1] = '\0';

    _band = 0;

    _cid = 1;
    
    memcpy(_targetIP, "0.0.0.0", sizeof("0.0.0.0"));
    _targetIP[sizeof(_targetIP) - 1] = '\0';

    _targetPort = 0;

    _accelerationPercentage = 20;
    _accelerationDuration = 0;

    _onTheMoveTimeout = 10;

    _gpsMinSatelliteCount = 4;
    _isDebugOn = 0;

    if (configResetCallback) {
        configResetCallback();
    }

    _sensorsSampleInterval = 10;

    needsCommit = true;
}

/*
 * Write the configuration parameters to NVM / Dataflash
 */
void ConfigParams::commit(bool forced)
{
    if (!forced && !needsCommit) {
        return;
    }

    _header = DEFAULT_HEADER;
    _crc16 = crc16ccitt((uint8_t*)this, (uint32_t)&params._crc16 - (uint32_t)&params._header);

    flash.write(*this);

    needsCommit = false;
}

static const Command args[] = {
    { "GPS                           ", 0,      0,                  Command::show_title, 0 },
    { "GPS Fix Timeout (sec)         ", "gft=", Command::set_uint16, Command::show_uint16, &params._gpsFixTimeout },
    { "Minimum sat count             ", "sat=", Command::set_uint8, Command::show_uint8, &params._gpsMinSatelliteCount },
    { "On-the-move Functionality     ", 0,      0,                  Command::show_title, 0 },
    { "Acceleration% (100% = 8g)     ", "acc=", Command::set_uint8, Command::show_uint8, &params._accelerationPercentage },
    { "Acceleration Duration         ", "acd=", Command::set_uint8, Command::show_uint8, &params._accelerationDuration },
    { "Timeout (min)                 ", "act=", Command::set_uint8, Command::show_uint8, &params._onTheMoveTimeout },
    { "Cellular                      ", 0,      0,                  Command::show_title, 0 },
    { "APN                           ", "apn=", Command::set_string, Command::show_string, params._apn, sizeof(params._apn) },
    { "CDP                           ", "cdp=", Command::set_string, Command::show_string, params._cdp, sizeof(params._cdp) },
    { "Force Operator                ", "opr=", Command::set_string, Command::show_string, params._forceOperator, sizeof(params._forceOperator) },
    { "CID                           ", "cid=", Command::set_uint8, Command::show_uint8, &params._cid },
    { "Band                          ", "bnd=", Command::set_uint8, Command::show_uint8, &params._band },
    { "Target IP                     ", "ip=",  Command::set_string, Command::show_string, params._targetIP, sizeof(params._targetIP) },
    { "Target port                   ", "prt=", Command::set_uint16, Command::show_uint16, &params._targetPort },
    { "Misc                          ", 0,      0,                  Command::show_title, 0 },
    { "Sensors sample interval (sec) ", "ssi=", Command::set_uint8, Command::show_uint8, &params._sensorsSampleInterval },
    { "Debug (OFF=0 / ON=1)          ", "dbg=", Command::set_uint8, Command::show_uint8, &params._isDebugOn }
};

void ConfigParams::showConfig(Stream* stream)
{
    stream->println();
    stream->println("Settings:");
    for (size_t i = 0; i < sizeof(args) / sizeof(args[0]); ++i) {
        const Command* a = &args[i];
        if (a->show_func) {
            a->show_func(a, stream);
        }
    }
}

/*
 * Execute a command from the commandline
 *
 * Return true if it was a valid command
 */
bool ConfigParams::execCommand(const char* line)
{
    bool done = Command::execCommand(args, sizeof(args) / sizeof(args[0]), line);
    if (done) {
        needsCommit = true;
    }

    return done;
}

/*
 * Check if all required config parameters are filled in
 */
bool ConfigParams::checkConfig(Stream& stream)
{
    bool fail = false;

    if (_isDebugOn > 1) {
        stream.println("Debug must be either 0 or 1");
        fail = true;
    }

    if (_accelerationPercentage > 100) {
        stream.println("Acceleration% must not be more than 100");
        fail = true;
    }

    return !fail;
}

void ConfigParams::setConfigResetCallback(VoidCallbackMethodPtr callback)
{
    configResetCallback = callback;
}
