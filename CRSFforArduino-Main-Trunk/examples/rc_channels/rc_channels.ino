/**
 * @file rc_channels.ino
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief Example of how to read rc channels from a receiver.
 * @version 1.0.3
 * @date 2024-7-20
 *
 * @copyright Copyright (c) 2024, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 * @section License GNU Affero General Public License v3.0
 * This example is a part of the CRSF for Arduino library.
 * CRSF for Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CRSF for Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with CRSF for Arduino.  If not, see <https://www.gnu.org/licenses/>.
 * 
 */

#include "CRSFforArduino.hpp"
#define USE_SERIAL_PLOTTER 0

volatile int channel1         = 0;
volatile int roll_channel     = 0;
volatile int channel2         = 0;
volatile int throttle_channel = 0;
volatile int channel3         = 0;
volatile int pitch_channel    = 0;
volatile int channel4         = 0;
volatile int yaw_channel      = 0;
volatile int channel5         = 0;
volatile int ch5_channel      = 0;
volatile int channel6         = 0;
volatile int ch6_channel      = 0;
volatile int channel7         = 0;
volatile int ch7_channel      = 0;
volatile int channel8         = 0;
volatile int ch8_channel      = 0;
volatile int channel9         = 0;
volatile int ch9_channel      = 0;

CRSFforArduino *crsf = nullptr;
int rcChannelCount = crsfProtocol::RC_CHANNEL_COUNT;
HardwareSerial Serial2(PC7,PC6); // define Serial1 pin in this section
uint32_t timeProgram, previousTimeProgram;

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels);

void setup()
{
    // Initialise the serial port & wait for the port to open.
    Serial.begin(115200);

    // Initialise CRSF for Arduino.
    crsf = new CRSFforArduino(&Serial2, PC7,PC6);
    if (!crsf->begin())
    {
        crsf->end();

        delete crsf;
        crsf = nullptr;

        Serial.println("CRSF for Arduino initialisation failed!");
        while (1)
        {
            delay(10);
        }
    }

    rcChannelCount = rcChannelCount > crsfProtocol::RC_CHANNEL_COUNT ? crsfProtocol::RC_CHANNEL_COUNT : rcChannelCount;

    crsf->setRcChannelsCallback(onReceiveRcChannels);

    // Show the user that the sketch is ready.
    Serial.println("RC Channels Example");
    delay(1000);
    Serial.println("Ready");
    delay(1000);
}

void loop()
{
    crsf->update();
  if (timeProgram - previousTimeProgram >= 100000)
  {
    Serial.print(roll_channel)        ;Serial.print("\t");
    Serial.print(pitch_channel)        ;Serial.print("\t");
    Serial.print(yaw_channel)        ;Serial.print("\t");
    Serial.print(throttle_channel)        ;Serial.print("\t");
    Serial.print(ch5_channel)        ;Serial.print("\t");
    Serial.print(ch6_channel)        ;Serial.print("\t");
    Serial.print(ch7_channel)        ;Serial.print("\t");
    Serial.print(ch8_channel)        ;Serial.print("\t");
}}

void onReceiveRcChannels(serialReceiverLayer::rcChannels_t *rcChannels)
{
    if (rcChannels->failsafe == false)
    {
        /* Print RC channels every 100 ms. */
        unsigned long thisTime = millis();
        static unsigned long lastTime = millis();

        /* Compensate for millis() overflow. */
        if (thisTime < lastTime)
        {
            lastTime = thisTime;
        }

        if (thisTime - lastTime >= 100)
        {
            lastTime = thisTime;
            roll_channel = crsf->rcToUs(crsf->getChannel(1));
            pitch_channel = crsf->rcToUs(crsf->getChannel(2));
            throttle_channel = crsf->rcToUs(crsf->getChannel(3));
            yaw_channel = crsf->rcToUs(crsf->getChannel(4));
            ch5_channel = crsf->rcToUs(crsf->getChannel(5));
            ch6_channel = crsf->rcToUs(crsf->getChannel(6));
            ch7_channel = crsf->rcToUs(crsf->getChannel(7));
            ch8_channel = crsf->rcToUs(crsf->getChannel(8));
        }
    }
}
