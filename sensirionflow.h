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

#ifndef SENSIRIONFLOW_H
#define SENSIRIONFLOW_H

#include <inttypes.h>

class SensirionFlow
{
public:
  SensirionFlow(uint8_t i2cAddress);
  void init();
  void reset();

  /**
   * Original function kept for compatibility.
   */
  float readSample();

  /**
   * Trigger measurement, return true if successful.
   */
  bool triggerMeasurement(bool masterHold = true);
  /**
   * Master hold: Trigger a measurement with triggerMeasurement, then use readMeasurement to read it (blocking).
   * No master hold: Trigger a measurement with triggerMeasurement, then poll with readMeasurement until returning true.
   * Please note that you may need to modify the relevant sensor register first to disable master hold.
   */
  bool readMeasurement(float *measurement);

  uint8_t getDimension()          const { return mDimension;          };
  uint8_t getTimeBase()           const { return mTimeBase;           };
  uint8_t getVolumePressureUnit() const { return mVolumePressureUnit; };

  typedef enum
  {
	  user_reg = 0,
	  adv_user_reg,
	  readonly_1,
	  readonly_2
  } register_id_t;

  typedef uint16_t register_value_t;

  bool readRegister(register_id_t reg, register_value_t *buffer);
  bool writeRegister(register_id_t reg, register_value_t data);
  bool modifyRegister(register_id_t reg, register_value_t data, register_value_t mask);

private:
  uint8_t mI2cAddress;
  int16_t mScaleFactor;

  uint8_t mDimension;
  uint8_t mTimeBase;
  uint8_t mVolumePressureUnit;
};

#endif
