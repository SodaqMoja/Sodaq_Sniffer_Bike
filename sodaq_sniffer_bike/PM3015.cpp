/*
Copyright (c) 2018, SODAQ
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

#include "PM3015.h"

PM3015::PM3015()
{
}

PM3015::~PM3015() 
{
}

void PM3015::init(Stream& stream)
{
    this->_sensorStream = &stream;

    _sensorStream->setTimeout(1000);
}

bool PM3015::openMeasurement()
{
    uint8_t openResponse[5];
    memset(openResponse, 0, sizeof(openResponse));

    _sendCommand(PM3015_OPEN_MEASUREMENT, openResponse, sizeof(openResponse));

    return _isDataValid(openResponse, sizeof(openResponse));
}

bool PM3015::closeMeasurement()
{
    uint8_t closeResponse[5];
    memset(closeResponse, 0, sizeof(closeResponse));

    _sendCommand(PM3015_CLOSE_MEASUREMENT, closeResponse, sizeof(closeResponse));
    
    return _isDataValid(closeResponse, sizeof(closeResponse));
}

bool PM3015::readMeasurements(uint32_t* pm1_0, uint32_t* pm2_5, uint32_t* pm10)
{
    uint8_t response[56];
    _sendCommand(PM3015_READ_MEASUREMENT, response, sizeof(response));

    if (_isDataValid(response, sizeof(response))) {
        uint8_t df0 = 2;
        uint32_t pm1_0grimm = response[df0 + 1] * pow(256, 3) + response[df0 + 2] * pow(256, 2) + response[df0 + 3] * 256 + response[df0 + 4];
        uint32_t pm2_5grimm = response[df0 + 5] * pow(256, 3) + response[df0 + 6] * pow(256, 2) + response[df0 + 7] * 256 + response[df0 + 8];
        uint32_t pm10grimm = response[df0 + 9] * pow(256, 3) + response[df0 + 10] * pow(256, 2) + response[df0 + 11] * 256 + response[df0 + 12];

        *pm1_0 = pm1_0grimm;
        *pm2_5 = pm2_5grimm;
        *pm10 = pm10grimm;

        return true;
    }
    else {
        return false;
    }
}

bool PM3015::_isDataValid(const uint8_t* data, size_t size)
{
    if (data[0] == 0x16) {
        // check checksum
        uint8_t sum = 0;

        for (uint8_t i = 0; i < size - 1; i++) {
            sum += data[i];
        }
        uint8_t checksum = 256 - sum;

        return (data[size - 1] == checksum);
    }
    else {
        return false;
    }
}

void PM3015::_sendCommand(Command command, uint8_t* response, size_t size)
{
    if (command == Command::PM3015_READ_MEASUREMENT) {
        uint8_t commandBuffer[] = { 0x11, 0x02, 0x0B, 0x07, 0xDB };
        _sensorStream->write(commandBuffer, sizeof(commandBuffer));

    }
    else if (command == Command::PM3015_OPEN_MEASUREMENT) {
        uint8_t commandBuffer[] = { 0x11, 0x03, 0x0C, 0x02, 0x1E, 0xC0 };
        _sensorStream->write(commandBuffer, sizeof(commandBuffer));
    }
    else if (command == Command::PM3015_CLOSE_MEASUREMENT) {
        uint8_t commandBuffer[] = { 0x11, 0x03, 0x0C, 0x01, 0x1E, 0xC1 };
        _sensorStream->write(commandBuffer, sizeof(commandBuffer));
    }
    else {
        return;
    }

    _sensorStream->readBytes(response, size);
}
