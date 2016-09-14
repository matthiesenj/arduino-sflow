/*
 *  Copyright (c) 2015, Johannes Winkelmann, jw@smts.ch
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <Wire.h>
#include <Arduino.h>

#include "sensirionflow.h"
#include "flowi2chelper.h"


SensirionFlow::SensirionFlow(uint8_t i2cAddress)
  : mI2cAddress(i2cAddress), 
    mScaleFactor(1),
    mDimension(-1),
    mTimeBase(-1),
    mVolumePressureUnit(-1)

{
}

bool SensirionFlow::init()
{
  const uint8_t DATA_LENGTH = 6; // 2 data, 1 crc
  uint8_t data[DATA_LENGTH];
  
  // - read user register
  register_value_t baseAddress;
  if (!readRegister(user_reg, &baseAddress)) {
      return false;
  }

  baseAddress &= 0x70; // EEPROM base address is bits <6..4>
  baseAddress >>= 4;
  baseAddress *= 0x300;

  // - read scale factor
  uint16_t scaleFactorAddress = (baseAddress + 0x02B6);
  scaleFactorAddress <<= 4;  // address is a left aligned 12-bit value

  uint8_t cmdReadRegister[] = { 0xFA, (uint8_t)(scaleFactorAddress >> 8), (uint8_t)(scaleFactorAddress & 0x00FF) };
  if (!I2CHelper::readFromI2C(mI2cAddress, cmdReadRegister, 3, data, DATA_LENGTH)) {
    return false;
  }
  mScaleFactor = (data[0] << 8) + data[1]; // data[2] = crc

  uint16_t measurementUnit = (data[3] << 8) + data[4];  // data[2] = crc
  mDimension = measurementUnit & 0xF;
  mTimeBase = (measurementUnit >> 4) & 0xF;
  mVolumePressureUnit = (measurementUnit >> 8) & 0x1F;
  return true;
}

void SensirionFlow::reset()
{
  const uint8_t CMD_LENGTH = 1;
  const uint8_t CMD_RESET[CMD_LENGTH] = { 0xFE };
  I2CHelper::readFromI2C(mI2cAddress, CMD_RESET, CMD_LENGTH, NULL, 0);
}

float SensirionFlow::readSample()
{
  float measurement;
  if (!triggerMeasurement(true) || !readMeasurement(&measurement))
  {
      measurement = 0;
  }
  return measurement;
}

bool SensirionFlow::triggerMeasurement(bool masterHold)
{
  const uint8_t cmdLength = 1;
  static uint8_t command[cmdLength] = { 0xF1 };

  // new
  Wire.beginTransmission(mI2cAddress);
  Wire.write(command, cmdLength);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  // if not master hold, we need to perform a read to trigger the measurement
  if (!masterHold) {
    Wire.requestFrom(mI2cAddress, (uint8_t)0);
  }

  return true;
}

bool SensirionFlow::readMeasurement(float *measurement)
{
  const uint8_t dataLength = 2;
  uint8_t data[dataLength];

  if (dataLength > Wire.requestFrom(mI2cAddress, dataLength))
    return false;

  for (int i = 0; i < dataLength; ++i) {
    data[i] = Wire.read();
  }
  
  float measurementValue = ((data[0] << 8) + data[1]);
  *measurement = measurementValue / mScaleFactor;
  return true;
}

bool SensirionFlow::readRegister(register_id_t reg, register_value_t *buffer)
{
  const static uint8_t commands[] = { 0xE3, 0xE5, 0xE7, 0xE9 };
  const uint8_t dataLength = 2;

  if (reg >= 4) {
    return false;
  }

  uint8_t data[dataLength];
  
  if (!I2CHelper::readFromI2C(mI2cAddress, &commands[reg], 1, data, dataLength)) {
    return false;
  }
  
  *buffer = (data[0] << 8) | data[1];
  return true;
}

bool SensirionFlow::writeRegister(register_id_t reg, register_value_t data)
{
  const static uint8_t commands[] = { 0xE2, 0xE4 };
  const uint8_t commandLength = 3;

  if (reg >= 2) {
    return false;
  }

  uint8_t command[commandLength];
  command[0] = commands[reg];
  command[1] = data >> 8;
  command[2] = data & 0x00FF;

  return I2CHelper::readFromI2C(mI2cAddress, &command[0], commandLength, NULL, 0);
}

bool SensirionFlow::modifyRegister(register_id_t reg, register_value_t data, register_value_t mask)
{
  register_value_t value;
  if (!readRegister(reg, &value)) {
    return false;
  }

  value &= ~mask; // zero out bits to modify
  value |= data & mask; // set 1-bits to modify
  return writeRegister(reg, value);
}
