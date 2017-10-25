/*
 * Copyright (c) 2017, Bertold Van den Bergh (vandenbergh@bertold.org)
 * All rights reserved.
 * This work has been developed to support research funded by
 * "Fund for Scientific Research, Flanders" (F.W.O.-Vlaanderen).
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the author nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR DISTRIBUTOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "../i2csafe/i2csafe.h"
#include "../../util/syslog/syslog.h"
#include "hal.h"

#include <string.h>


static inline void i2cSafeRawSetClock(I2CDriver* i2c, bool level)
{
    i2cSafeConfig* config = (i2cSafeConfig*)i2c->i2cSafeConfig;
    gpioSetPin(config->sclPin, level);
}
static inline bool i2cSafeRawGetClock(I2CDriver* i2c)
{
    i2cSafeConfig* config = (i2cSafeConfig*)i2c->i2cSafeConfig;
    return gpioGetPin(config->sclPin);
}

static inline void i2cSafeRawSetData(I2CDriver* i2c, bool level)
{
    i2cSafeConfig* config = (i2cSafeConfig*)i2c->i2cSafeConfig;
    gpioSetPin(config->sdaPin, level);
}
static inline bool i2cSafeRawGetData(I2CDriver* i2c)
{
    i2cSafeConfig* config = (i2cSafeConfig*)i2c->i2cSafeConfig;
    return gpioGetPin(config->sdaPin);
}

static inline void i2cSafeSoftwareControl(I2CDriver* i2c)
{
    /* Stop the I2C peripheral, otherwise it crashes */
    i2cStop(i2c);

    /* Set SDA and SCL high */
    i2cSafeRawSetClock(i2c, true);
    i2cSafeRawSetData(i2c, true);

    /* Disconnect the pins from the peripheral and put them under software control */
    i2cSafeConfig* config = (i2cSafeConfig*)i2c->i2cSafeConfig;
    gpioSetPinMode(config->sdaPin, PAL_MODE_OUTPUT_OPENDRAIN);
    gpioSetPinMode(config->sclPin, PAL_MODE_OUTPUT_OPENDRAIN);
}

static inline void i2cSafeRawHardwareControl(I2CDriver* i2c)
{
    /* Set the pins back to their default state */
    i2cSafeConfig* config = (i2cSafeConfig*)i2c->i2cSafeConfig;

    /* This causes a glitch that confuses some devices, so we add a deliberate delay */
    i2cSafeRawSetData(i2c, false);
    osalThreadSleepMicroseconds(i2cSafe_US_DELAY);
    i2cSafeRawSetClock(i2c, false);
    osalThreadSleepMicroseconds(i2cSafe_US_DELAY);

    gpioSetPinMode(config->sclPin, config->peripheralMode);
    osalThreadSleepMicroseconds(i2cSafe_US_DELAY);

    gpioSetPinMode(config->sdaPin, config->peripheralMode);
    osalThreadSleepMicroseconds(i2cSafe_US_DELAY);

    i2cStart(i2c, &((const i2cSafeConfig*)i2c->i2cSafeConfig)->i2cfg);
}

/* This function will try to reset the I2C bus in case it got stuck */
static i2c_result i2cSafeRawUnclogBus(I2CDriver* i2c)
{
    uint8_t i,j, highCounter = 0;
    i2c_result retVal;

    /* Take control */
    i2cSafeSoftwareControl(i2c);

    osalThreadSleepMicroseconds(i2cSafe_US_DELAY);

    /* Send clocks until SDA stays high */
    for(i=0; i<32; i++) {
        /* Set SCL low */
        i2cSafeRawSetClock(i2c, false);
        osalThreadSleepMicroseconds(i2cSafe_US_DELAY);

        /* Check if it is indeed low */
        if(i2cSafeRawGetClock(i2c)) {
            retVal = I2C_BUS_STUCK_SCL_PULLED_HIGH;
            goto done;
        }

        /* Set SCL high */
        i2cSafeRawSetClock(i2c, true);
        osalThreadSleepMicroseconds(i2cSafe_US_DELAY);

        /* Wait for SCL to go high (in case of clock stretching) */
        for(j=100; !i2cSafeRawGetClock(i2c); j--) {
            osalThreadSleepMilliseconds(1);
            if(j == 0) {
                /* SCL totally jammed */
                retVal = I2C_BUS_STUCK_SCL_PULLED_LOW;
                goto done;
            }
        }

        /* If SDA is high we try to terminate the transaction */
        if(i2cSafeRawGetData(i2c)) {
            /* Send start condition followed by stop condition: SDA low and high again */
            i2cSafeRawSetData(i2c, false);
            osalThreadSleepMicroseconds(i2cSafe_US_DELAY);

            /* If data is not low now then it is shorted to VCC */
            if(i2cSafeRawGetData(i2c)) {
                retVal = I2C_BUS_STUCK_SDA_PULLED_HIGH;
                goto done;
            }

            /*
             * Clock is only driven from high to low by the master,
             * so if it is low now both wires are shorted together.
             */
            if(!i2cSafeRawGetClock(i2c)) {
                retVal = I2C_BUS_STUCK_SHORTED_TOGETHER;
                goto done;
            }

            i2cSafeRawSetData(i2c, true);
            osalThreadSleepMicroseconds(i2cSafe_US_DELAY);

            highCounter++;
        } else {
            highCounter = 0;
        }

        if(highCounter>=9) {
            /* We did it, SDA was high for entire transfer */
            retVal = I2C_BUS_OK;
            goto done;
        }
    }

    /* Did not manage to get SDA high for one whole transfer */
    retVal = I2C_BUS_STUCK_SDA_PULLED_LOW;

done:
    i2cSafeRawHardwareControl(i2c);

    return retVal;
}

i2c_result i2cSafeMasterTransmitTimeoutWithRetry (
    I2CDriver* i2c,
    i2caddr_t devAddr,
    const uint8_t* txbuf,
    size_t txbytes,
    uint8_t* rxbuf,
    size_t rxbytes,
    systime_t timeoutPerTry,
    unsigned int maxTries)
{

    msg_t status = I2C_BUS_OK;
    i2c_result i2c_status;
    unsigned int i;

    if(!devAddr) return MSG_RESET;

    for(i=0; i<maxTries; i++) {
        status = i2cMasterTransmitTimeout(i2c, devAddr, txbuf, txbytes, rxbuf, rxbytes, timeoutPerTry);
        if(status == MSG_OK) return status;

        i2cStop(i2c);

        osalSysLock();
        i2c->i2cErrors++;
        osalSysUnlock();

        /* Attempt to unclog */
        if((i2c_status = i2cSafeRawUnclogBus(i2c))) {
            syslog("I2C error, bus failure: %s.", i2cSafeResultToString(i2c_status));
            return i2c_status;
        }
    }

    syslog("I2C error, %u failed attempts.", maxTries);

    return I2C_BUS_RESET;
}

i2c_result i2cSafeReadRegBulkStandard(I2CDriver* i2c,
                                      uint8_t devAddr,
                                      uint8_t addr,
                                      uint8_t* values,
                                      unsigned int len)
{
    osalDbgAssert(i2c != NULL, "i2c == NULL");

    i2c_result i2c_status;
    uint8_t txBuf[1] = {addr};

    i2cAcquireBus(i2c);
    i2c_status = i2cSafeMasterTransmitTimeoutWithRetry(i2c, devAddr, txBuf,
                 sizeof(txBuf),
                 values,
                 len,
                 OSAL_MS2ST(5),
                 3);
    i2cReleaseBus(i2c);

    return i2c_status;
}


i2c_result i2cSafeWriteRegBulkStandard(I2CDriver* i2c,
                                       uint8_t devAddr,
                                       uint8_t addr,
                                       uint8_t* values,
                                       unsigned int len)
{
    osalDbgAssert(i2c != NULL, "i2c == NULL");

    i2c_result i2c_status;
    uint8_t txBuf[1 + len];

    txBuf[0]=addr;
    memcpy(&txBuf[1], values, len);

    i2cAcquireBus(i2c);
    i2c_status = i2cSafeMasterTransmitTimeoutWithRetry(i2c, devAddr, txBuf,
                 sizeof(txBuf),
                 NULL,
                 0,
                 OSAL_MS2ST(5),
                 3);
    i2cReleaseBus(i2c);

    return i2c_status;
}

i2c_result i2cSafeReadRegStandard(I2CDriver* i2c,
                                  uint8_t devAddr,
                                  uint8_t addr,
                                  uint8_t* value)
{
    osalDbgAssert(i2c != NULL, "i2c == NULL");

    /* The hardware/driver does not support 1 byte reads */
    uint8_t values[2];
    i2c_result result = i2cSafeReadRegBulkStandard(i2c, devAddr, addr, values, sizeof(values));
    *value=values[0];

    return result;
}

i2c_result i2cSafeWriteRegStandard(I2CDriver* i2c,
                                   uint8_t devAddr,
                                   uint8_t addr,
                                   uint8_t value)
{
    osalDbgAssert(i2c != NULL, "i2c == NULL");

    return i2cSafeWriteRegBulkStandard(i2c, devAddr, addr, &value, 1);
}

void i2cSafeInit(I2CDriver* i2c, const i2cSafeConfig* config)
{
    i2c->i2cSafeConfig = (void*)config;

    /* Configure IO for peripheral */
    i2cSafeRawHardwareControl(i2c);
}

i2c_result i2cSafeTestBus(I2CDriver* i2c)
{
    i2cAcquireBus(i2c);

    i2c_result retVal = i2cSafeRawUnclogBus(i2c);

    i2cReleaseBus(i2c);

    return retVal;
}


const char* i2cSafeResultToString(i2c_result result)
{
    if(result == I2C_BUS_OK) return "OK";
    else if(result == I2C_BUS_TIMEOUT) return "Timeout";
    else if(result == I2C_BUS_RESET) return "Transfer Error";
    else if(result == I2C_BUS_STUCK_SCL_PULLED_LOW) return "SCL stuck low";
    else if(result == I2C_BUS_STUCK_SDA_PULLED_LOW) return "SDA stuck low";
    else if(result == I2C_BUS_STUCK_SCL_PULLED_HIGH) return "SCL stuck high";
    else if(result == I2C_BUS_STUCK_SDA_PULLED_HIGH) return "SDA stuck high";
    else if(result == I2C_BUS_STUCK_SHORTED_TOGETHER) return "SDA/SCL shorted together";
    else return "Unknown";
}

uint32_t i2cSafeGetNumberOfErrors(I2CDriver* i2c)
{
    osalSysLock();
    uint32_t result = i2c->i2cErrors;
    osalSysUnlock();
    return result;
}

