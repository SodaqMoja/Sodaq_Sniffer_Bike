/*
Copyright (c) 2015-18, SODAQ
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

#include "Sodaq_AT_Device.h"

#define DEBUG

#ifdef DEBUG
#define debugPrintLn(...) { if (!this->_disableDiag && this->_diagStream) this->_diagStream->println(__VA_ARGS__); }
#define debugPrint(...) { if (!this->_disableDiag && this->_diagStream) this->_diagStream->print(__VA_ARGS__); }
#warning "Debug mode is ON"
#else
#define debugPrintLn(...)
#define debugPrint(...)
#endif

#define CR "\r"
#define LF "\n"
#define CRLF "\r\n"

// TODO this needs to be set in the compiler directives. Find something else to do
#define SODAQ_AT_DEVICE_TERMINATOR CRLF

#ifndef SODAQ_AT_DEVICE_TERMINATOR
#warning "SODAQ_AT_DEVICE_TERMINATOR is not set"
#define SODAQ_AT_DEVICE_TERMINATOR CRLF
#endif

#define SODAQ_AT_DEVICE_TERMINATOR_LEN (sizeof(SODAQ_AT_DEVICE_TERMINATOR) - 1) // without the NULL terminator

#define SODAQ_AT_DEVICE_DEFAULT_INPUT_BUFFER_SIZE 250

// Constructor
Sodaq_AT_Device::Sodaq_AT_Device() :
    _txEnablePin(-1),
    _modemStream(0),
    _diagStream(0),
    _disableDiag(false),
    _inputBufferSize(SODAQ_AT_DEVICE_DEFAULT_INPUT_BUFFER_SIZE),
    _inputBuffer(0),
    _onoff(0),
    _baudRateChangeCallbackPtr(0),
    _appendCommand(false),
    _startOn(0)
{
    this->_isBufferInitialized = false;
}

// Turns the modem on and returns true if successful.
bool Sodaq_AT_Device::on()
{
    _startOn = millis();

    if (!isOn()) {
        if (_onoff) {
            _onoff->on();
        }
    }
	
	setTxPowerIfAvailable(true);

    // wait for power up
    bool timeout = true;

    for (uint8_t i = 0; i < 15; i++) {
        if (isAlive()) {
            timeout = false;
            break;
        }
    }

    if (timeout) {
        debugPrintLn("Error: No Reply from Modem");
        return false;
    }    

    return isOn(); // this essentially means isOn() && isAlive()
}

// Turns the modem off and returns true if successful.
bool Sodaq_AT_Device::off()
{
    // No matter if it is on or off, turn it off.
    if (_onoff) {
        _onoff->off();
    }

    setTxPowerIfAvailable(false);

    return !isOn();
}

// Returns true if the modem is on.
bool Sodaq_AT_Device::isOn() const
{
    if (_onoff) {
        return _onoff->isOn();
    }

    // No onoff. Let's assume it is on.
    return true;
}

void Sodaq_AT_Device::setTxPowerIfAvailable(bool on)
{
    if (_txEnablePin != -1) {
        digitalWrite(_txEnablePin, on);
    }
}

void Sodaq_AT_Device::writeProlog()
{
    if (!_appendCommand) {
        debugPrint(">> ");
        _appendCommand = true;
    }
}

// Write a byte, as binary data
size_t Sodaq_AT_Device::writeByte(uint8_t value)
{
    return _modemStream->write(value);
}

size_t Sodaq_AT_Device::print(const String& buffer)
{
    writeProlog();
    debugPrint(buffer);

    return _modemStream->print(buffer);
}

size_t Sodaq_AT_Device::print(const char buffer[])
{
    writeProlog();
    debugPrint(buffer);

    return _modemStream->print(buffer);
}

size_t Sodaq_AT_Device::print(char value)
{
    writeProlog();
    debugPrint(value);

    return _modemStream->print(value);
};

size_t Sodaq_AT_Device::print(unsigned char value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_AT_Device::print(int value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_AT_Device::print(unsigned int value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_AT_Device::print(long value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_AT_Device::print(unsigned long value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_AT_Device::println(const __FlashStringHelper* ifsh)
{
    size_t n = print(ifsh);
    n += println();
    return n;
}

size_t Sodaq_AT_Device::println(const String& s)
{
    size_t n = print(s);
    n += println();
    return n;
}

size_t Sodaq_AT_Device::println(const char c[])
{
    size_t n = print(c);
    n += println();
    return n;
}

size_t Sodaq_AT_Device::println(char c)
{
    size_t n = print(c);
    n += println();
    return n;
}

size_t Sodaq_AT_Device::println(unsigned char b, int base)
{
    size_t i = print(b, base);
    return i + println();
}

size_t Sodaq_AT_Device::println(int num, int base)
{
    size_t i = print(num, base);
    return i + println();
}

size_t Sodaq_AT_Device::println(unsigned int num, int base)
{
    size_t i = print(num, base);
    return i + println();
}

size_t Sodaq_AT_Device::println(long num, int base)
{
    size_t i = print(num, base);
    return i + println();
}

size_t Sodaq_AT_Device::println(unsigned long num, int base)
{
    size_t i = print(num, base);
    return i + println();
}

size_t Sodaq_AT_Device::println(double num, int digits)
{
    writeProlog();
    debugPrint(num, digits);

    return _modemStream->println(num, digits);
}

size_t Sodaq_AT_Device::println(const Printable& x)
{
    size_t i = print(x);
    return i + println();
}

size_t Sodaq_AT_Device::println(void)
{
    debugPrintLn();
    size_t i = print('\r');
    _appendCommand = false;
    return i;
}

// Initializes the input buffer and makes sure it is only initialized once.
// Safe to call multiple times.
void Sodaq_AT_Device::initBuffer()
{
    debugPrintLn("[initBuffer]");

    // make sure the buffers are only initialized once
    if (!_isBufferInitialized) {
        this->_inputBuffer = static_cast<char*>(malloc(this->_inputBufferSize));

        _isBufferInitialized = true;
    }
}

// Sets the modem stream.
void Sodaq_AT_Device::setModemStream(Stream& stream)
{
    this->_modemStream = &stream;
}

// Sets the optional tx enable pin.
void Sodaq_AT_Device::setTxEnablePin(int8_t txEnablePin)
{
    _txEnablePin = txEnablePin;

    if (_txEnablePin != -1) {
        pinMode(_txEnablePin, OUTPUT);
        digitalWrite(_txEnablePin, LOW);
    }
}

// Returns a character from the modem stream if read within _timeout ms or -1 otherwise.
int Sodaq_AT_Device::timedRead(uint32_t timeout) const
{
    int c;
    uint32_t _startMillis = millis();

    do {
        c = _modemStream->read();

        if (c >= 0) {
            return c;
        }
    } while (millis() - _startMillis < timeout);

    return -1; // -1 indicates timeout
}

// Fills the given "buffer" with characters read from the modem stream up to "length"
// maximum characters and until the "terminator" character is found or a character read
// times out (whichever happens first).
// The buffer does not contain the "terminator" character or a null terminator explicitly.
// Returns the number of characters written to the buffer, not including null terminator.
size_t Sodaq_AT_Device::readBytesUntil(char terminator, char* buffer, size_t length, uint32_t timeout)
{
    if (length < 1) {
        return 0;
    }

    size_t index = 0;

    while (index < length) {
        int c = timedRead(timeout);

        if (c < 0 || c == terminator) {
            break;
        }

        *buffer++ = static_cast<char>(c);
        index++;
    }

    if (index < length) {
        *buffer = '\0';
    }

    // TODO distinguise timeout from empty string?
    // TODO return error for overflow?
    return index; // return number of characters, not including null terminator
}

// Fills the given "buffer" with up to "length" characters read from the modem stream.
// It stops when a character read times out or "length" characters have been read.
// Returns the number of characters written to the buffer.
size_t Sodaq_AT_Device::readBytes(uint8_t* buffer, size_t length, uint32_t timeout)
{
    size_t count = 0;

    while (count < length) {
        int c = timedRead(timeout);

        if (c < 0) {
            break;
        }

        *buffer++ = static_cast<uint8_t>(c);
        count++;
    }

    // TODO distinguise timeout from empty string?
    // TODO return error for overflow?
    return count;
}

// Reads a line (up to the SODAQ_GSM_TERMINATOR) from the modem stream into the "buffer".
// The buffer is terminated with null.
// Returns the number of bytes read, not including the null terminator.
size_t Sodaq_AT_Device::readLn(char* buffer, size_t size, uint32_t timeout)
{
    // Use size-1 to leave room for a string terminator
    size_t len = readBytesUntil(SODAQ_AT_DEVICE_TERMINATOR[SODAQ_AT_DEVICE_TERMINATOR_LEN - 1], buffer, size - 1, timeout);

    // check if the terminator is more than 1 characters, then check if the first character of it exists
    // in the calculated position and terminate the string there
    if ((SODAQ_AT_DEVICE_TERMINATOR_LEN > 1) && (buffer[len - (SODAQ_AT_DEVICE_TERMINATOR_LEN - 1)] == SODAQ_AT_DEVICE_TERMINATOR[0])) {
        len -= SODAQ_AT_DEVICE_TERMINATOR_LEN - 1;
    }

    // terminate string, there should always be room for it (see size-1 above)
    buffer[len] = '\0';

    return len;
}
