# SODAQ Sniffer Bike

Note: to be able to compile this application you need to add the right board file to your Arduino IDE.

Go to File, Preferences and set the following URL for the additional board files:

http://downloads.sodaq.net/package_sodaq_samd_index.json

##  Configuration Menu

After compiling the source code and uploading it to the board you will be able to configure the board through a menu.

Just open the Arduino Serial Monitor (at 115200 baud) and you will get this menu:
```
** SODAQ - Sniffer Bike - 1.0.0 **

IMEI: 
 -> CPU reset by External [16]


Commands:
  Reset DevAddr / DevEUI to the Hardware EUI (EUI): 
  Commit Settings (CS): 

Settings:

GPS                           
  GPS Fix Timeout (sec)          (gft=): 120
  Minimum sat count              (sat=): 4

On-the-move Functionality     
  Acceleration% (100% = 8g)      (acc=): 20
  Acceleration Duration          (acd=): 0
  Timeout (min)                  (act=): 10

Cellular                      
  APN                            (apn=): cdp.iot.t-mobile.nl
  CDP                            (cdp=): 172.27.131.100
  Force Operator                 (opr=): 20416
  CID                            (cid=): 0
  Band                           (bnd=): 8
  Target IP                      (ip=): 172.27.131.100
  Target port                    (prt=): 15683

Misc                          
  Sensors sample interval (sec)  (ssi=): 10
  Debug (OFF=0 / ON=1)           (dbg=): 0
Enter command: 
```

Entering commands is just a matter of typing the command as given in brackets with the right value. For example:

ssi=15

Will set the sensors sample interval to 15 seconds

#### Sleep

After the startup the device by default will be in sleep mode. It will wake up when the device senses it is on the move and goes back to sleep when the device senses it stopped moving.

#### GPS fixes

An initial gps fix is attempted when the device is started. Subsequent GPS fixes are attempted when gathering environment data. If no fix is obtained, the sample will have a latitude and longitude of 0.

#### On-the-move functionality

The firmware is set to turn on the device when the accelerometer senses motion. At that point, the sensors are turned on and the device gathers samples at a set interval, saving them along with a current GPS fix (if fix is possible). The obtained samples are saved and only sent when enough samples are available to send a full UDP payload. When the device stops moving, it goes to sleep and no more samples are gathered.

#### Additional functionality

The device stops all sensors when battery levels are low. Any samples that have not been sent, are kept and once the battery is recharged, they are sent along with new samples or are lost if battery discharges completely. 
The device also has an led that indicates its state: 
 - RED indicates battery is low
 - BLUE indicates device is gathering samples
 - GREEN for 2 seconds indicates a UDP message is sent

#### Sensor sample content

| Description | Length |
| --- | --- |
| Epoch Timestamp | uint32 (4) |
| Lat | int32 (4) |
| Long | int32 (4) |
| Horizontal Accuracy | uint32 (4) |
| Vertical Accuracy | uint32 (4) |
| PM1.0 Measurement | uint32 (4) |
| PM2.5 Measurement | uint32 (4) |
| PM10 Measurement | uint32 (4) |
| Temperature Measurement | int16 (2) |
| Pressure Measurement | uint32 (4) |
| VOC Measurement | uint16 (2) |
| Humidity Measurement | uint8 (1) |
| Battery Voltage | uint8 (1) | 

## License

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