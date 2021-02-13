/**
* Bill Siever
* 2018-10-10
* 2020-02-07 CODAL and V2 updates
*
* Development environment specifics:
* Written in Microsoft PXT
* Based on SparkFun weather:bit for micro:bit (https://github.com/sparkfun/pxt-weather-bit)
*
* This code is released under the [MIT License](http://opensource.org/licenses/MIT).
* Please review the LICENSE.md file included with this example. If you have any questions 
* or concerns with licensing, please contact techsupport@sparkfun.com.
* Distributed as-is; no warranty is given.
*/

#include "pxt.h"
#include <cstdint>
#include <math.h>
#include "app_error.h"
#include "nrf.h"
#include "MicroBitSystemTimer.h"


#define DEBUG 1
// DEBUG uses ioPin P1 to indicate sampling of read (for timing calibration)
using namespace pxt;


#if MICROBIT_CODAL
#else
#endif

#if MICROBIT_CODAL

    #define _wait_us(us)  target_wait_us(us)
    #define _GPIO int
    #define setToInput(pin)  ;
    #define setToOutput(pin) ;
    #define setPinValue(pin, val)  ;
    #define getPinValue(pin)    (1)

#else

    #define _wait_us(us)            wait_us(us-5)
    #define _GPIO                   gpio_t*
    #define setToInput(pin)         gpio_dir(pin, PIN_INPUT)
    #define setToOutput(pin)        gpio_dir(pin, PIN_OUTPUT)
    #define setPinValue(pin, val)   gpio_write(pin, val)
    #define getPinValue(pin)        gpio_read(pin)

#endif

namespace dstemp { 

    // ************* Forward Decalarations
    void loopUntilSent(ManagedString str);
    void loopUntilSent(int str);
    void waitOut(unsigned long start, unsigned long duration, bool yield = true);
    void writeBit(_GPIO ioPin, bool one, bool finalYield = true);
    void writeByte(_GPIO ioPin, uint8_t b, bool finalYield = true);
    bool readBit(_GPIO ioPin);
    bool readScratchpad(_GPIO ioPin, float& temp);
    bool resetAndCheckPresence(_GPIO ioPin);
    bool configure(_GPIO pin);
    bool startConversion(_GPIO ioPin);

    // ************* Timing related constants 
    // Times related to time slots and read/write operations
    const int TIME_SLOT = 90;   // Time slot length = 60-120uS T_SLOT; Rounded up from min
    const int TIME_RECOV =  15;   // Recovery time between bits = 1uS T_REC; Rounded up for pull-up time.
    const int TIME_ZERO_LOW = TIME_SLOT;  // Zero low time = 60-120uS;  T_LOW0; Assume 100% of time slot
    const int TIME_ONE_LOW = 0; // One Low Time = 1-15uS; T_LOW1;  Hand calibrated via scope to be ~11.5uS

    const int TIME_READ_START = 1; // Not used / Read start (>uS);  Not used;  Hand calibrated via extra call
    const int TIME_READ_OFFSET = 0; // Hand Calibrated via scope;  1uS+overhead ==> 15uS from beginning of cycle (no more than 15uS)
    const int TIME_SLAVE_WRITE_END = 61; // Time from start of cycle to the end of the slave impacting the bus: 15uS+45uS (rounded up)

    // Times related to reset / presence detection (all in uS)
    const int TIME_RESET_LOW = 500;       // Reset low = 480uS T_RSTL (rounded up)
    const int TIME_RESET_HIGH = 500;   // Reset High/Presence detection Time = 480 uS T_RSTH (rounded up)
    const int TIME_POWER_UP = 1000;    // Time for DS18B20 to power up before reset (not mentioned in Data Sheet; 1ms assumed sufficient)
    const int TIME_POST_RESET_TO_DETECT = 10; // Time after reset to wait before checking for presence = 15uS T_PDHIGH (rounded down)
    const int TIME_PRESENCE_DETECT = 300; // Max time to wait after releasing reset for potential detect = 60uS+240uS T_PDHIGH+T_PDLOW

    const int TIME_CONVERSION = 760; // Conversion Time = 750mS T_CONV (rounded up)

    // TODO: Consider NaN here instead??
    const float ERROR_SENTINEL = -INFINITY;  // Sentinel value to return for errors. 

    const int HIGH_ALARM = 0xFF;  // High Alarm Value  (both set and confirmed; Should be non-zero)
    const int LOW_ALARM = 0x80;   // Low Alarm Value   (both set and confirmed; Should be non-zero)

    const int MAX_TRIES = 3;      // Max tries to attempt conversion before fail 

    // ************* State variables 
    // TODO: Can these be "shared", like functions
    int errorObjectIdx = 0;
    int errorPort = 0;
    Action errorHandler = NULL;


    // ************* Blocks 

    //% 
    void setErrorHandler(Action a) {
        // Release any prior error handler
       if(errorHandler)
         pxt::decr(errorHandler);
        errorHandler = a; 
        if(errorHandler)    
            pxt::incr(errorHandler);
    }
 

 
    /*
     * Helper method to send an actual error code to the registered handler.
     * It will set error values and immediately call the handler (i.e., no race condition should occur) 
     */
    void error(int objIndex, int port) {
#if DEBUG
        loopUntilSent("err C\n"); 
#endif
        // TODO: Sharing variables in this way is risky.
        //       Race conditions could occur (not likely in current design though
        //       error handler is called immediately)
        errorObjectIdx = objIndex;
        // TODO: Find better approach to reverse pin mapping (from mbed pin ID back to microbit pins)
        //       This approach works for P0-P20 since pins IDs are contiguous from 0-20
        errorPort = port - MICROBIT_ID_IO_P0;
        if(errorHandler) {
#if DEBUG
        loopUntilSent("calling handler\n"); 
#endif
            pxt::runAction0(errorHandler);            
        }
    }

    /*  Configure the device for 12-bit conversion and set High/Low Alarm Values (Magic numbers too.)
     *  @param ioPin the IO pin to use
     * @returns true on (assumed) success; false on known failure
     */
    bool configure(_GPIO ioPin) {
        if(resetAndCheckPresence(ioPin) == false) {
#if DEBUG
            loopUntilSent("No Device Present\n");
#endif
            // Call error Handler
            // TODO: Unify device code
           
            return false;
        }
        // Write ROM command: Skip ROM Command CCh: To address all devices
        writeByte(ioPin, 0xCC);

        // Write: Function Command Write Scratchpad 4Eh 
        writeByte(ioPin, 0x4E);

        // Write: Data for Function command - Alarm High Byte
        writeByte(ioPin, HIGH_ALARM); 
        // Write: Data for Function command - Alarm Low Byte
        writeByte(ioPin, LOW_ALARM); 
        // Write: Data for Function command - Conversion Configuration Byte
        writeByte(ioPin, 0x7F); // Write bits for 12-bit conversion.
        return true;
    }

    /*  Start a temperature conversion
     *  @param ioPin the IO pin to use
     *  @returns true on  success; false on failure
     */
    bool startConversion(_GPIO ioPin) {
        if(resetAndCheckPresence(ioPin) == false) {
            return false;
        }

        // Write ROM command: Skip ROM Command CCh: To address all devices
        writeByte(ioPin, 0xCC);

        // Write: Function command - Convert 44h 
        writeByte(ioPin, 0x44, false);

        // Read Time Slot
        return readBit(ioPin)==0;
    }

     //% 
    float celsius(int pin) {
        // Only needs to be done once, but done every call...

        // Get corresponding I/O ioPin Object
        MicroBitPin *mbp = getPin(pin);  // This returns a "uBit.io.P0" type thing
#if MICROBIT_CODAL
        PinNumber pinNumber = mbp->name;
        _GPIO gpio = 0;
#else
        gpio_t gpioObj;
        _GPIO gpio = &gpioObj;
        gpio_init(gpio, mbp->name);
#endif
 


#if 1

    MicroBitPin* indicate = &uBit.io.P1;

#if MICROBIT_CODAL
        _GPIO indicatePin = 0;
#else
        gpio_t indicateObj;
        _GPIO indicatePin = &indicateObj;
        gpio_init(indicatePin, indicate->name);
        setToOutput(indicatePin);
#endif

    // Output timing tests (calibrate _wait_us())

    // Misc tests.

    // Setup
    setPinValue(gpio, 1);
    setToOutput(gpio);
    setPinValue(indicatePin, 1);
    setToOutput(indicatePin);

    setPinValue(gpio, 1);
    _wait_us(100);
    setPinValue(gpio, 0);
    _wait_us(100);
    setPinValue(gpio, 1);
    _wait_us(200);
    setPinValue(gpio, 0);
    _wait_us(400);
    setPinValue(gpio, 1);


    // Calibrate input loop
    // v1: 341 uS for 200 iterations; 1.705/iteration
    setPinValue(indicatePin, 0);
    setToInput(gpio);
    uint32_t maxCounts =  200;
    bool b = true;
    do {
        // If the bus goes low, its a 0
        b = b && getPinValue(gpio);
    } while(maxCounts-- > 0);
    setPinValue(indicatePin, 1);
    _wait_us(100);


    setPinValue(indicatePin, 0);

    // v1: 341 uS for 200 iterations; 1.705 uS/Iteration
    maxCounts = 200;
    setToInput(gpio);
    // Check for presence pulse
    bool presence = false;        
    do {
        presence =  presence || (getPinValue(gpio) == 0);
    } while (maxCounts-- > 0);
    setPinValue(indicatePin, 1);
   
    _wait_us(200);
    setPinValue(indicatePin, 0);
    // v1: Aim for exactly 200uS
    maxCounts = (int)(200/1.705);
    setToInput(gpio);
    // Check for presence pulse
    presence = false;        
    do {
        presence =  presence || (getPinValue(gpio) == 0);
    } while (maxCounts-- > 0);
    setPinValue(indicatePin, 1);
   
    setToOutput(gpio);

    return 0;
#endif 






#if DEBUG 
{
char buffer[24];
sprintf(buffer, "pin: %d\n", mbp->name);
loopUntilSent(buffer);
}
#endif 

#if DEBUG
        loopUntilSent("Celsius Block\n");
#endif
        bool success = false;
        // 1. Check for valid device, configure it for conversion, and start conversion
        for(int tries=0;tries<MAX_TRIES;tries++) {
            // A. Configure Device
            if(configure(gpio)==false) {
                error(1, pin);
                goto return_error;
            }
            
            // B. Start conversion
            if(startConversion(gpio)) {
                success = true;
                break; // Leave the loop
            }            
        }

        // If unable to start conversion, return error
        if(success==false) {
            error(2, pin);
            goto return_error;
        }

        // 2. Wait for conversion to complete 
        while(readBit(gpio) == 0) {
            // Wait for conversion to complete. 
            uBit.sleep(0);
        }
     //   uBit.sleep(TIME_CONVERSION); // let other tasks run

        // 3. Retrieve Data 
        for(int tries=0;tries<MAX_TRIES;tries++) {
            // If reset is successful, request and read data
            if(resetAndCheckPresence(gpio)) {
                // Write ROM command: Skip ROM Command CCh: To address all devices
                writeByte(gpio, 0xCC);
                // Write Function command - Read scratchpad 
                writeByte(gpio, 0xBE);        
                // Read 8 bytes of scratch pad
                float temp;
                success = readScratchpad(gpio, temp);

                if(success) {
#if DEBUG
                    loopUntilSent("s");
#endif
                    errorObjectIdx = 0;
                    errorPort = pin;
                    // Return to input
                    setToInput(gpio);
                    return temp;
                }
            }
        } 
        // ERROR: Max Read Tries 
        // Call error Handler
        // TODO: Unify device code 
        error(3, pin);
return_error:
        // Return to input
        setToInput(gpio);
        // Return special sentinel value
        return ERROR_SENTINEL;
    }

    //%
    int getErrorObjectIdx() {
        return errorObjectIdx;
    } 

    //%
    int getErrorPort() {
        return errorPort;
    }

    // ************* Helper Functions 
  
#if DEBUG
    /**
     * 
     */
// https://www.forward.com.au/pfod/microbit/gettingStarted.html
    void loopUntilSent(ManagedString str) {
    int rtn = uBit.serial.send(str);
    while(rtn == MICROBIT_SERIAL_IN_USE) {
       uBit.sleep(0); // let other tasks run
       rtn = uBit.serial.send(str); 
    }
}
    void loopUntilSent(int str) {
    int rtn = uBit.serial.send(str);
    while(rtn == MICROBIT_SERIAL_IN_USE) {
       uBit.sleep(0); // let other tasks run
       rtn = uBit.serial.send(str); 
    }
}

#endif 

    /*
     * Wait until the designated duration has elapsed from the start time.  This uses sleep() to 
     * allow other fibers to run and may exceed the given time (i.e., not for time sensitive waits)
     * @param start the start time in uS based on system_timer_current_time_us()
     * @param duration the total duration in uS
     */
    // Wait out any remaining time to ensure the expected duration elapses after the start event.
    void waitOut(unsigned long start, unsigned long duration, bool yield) {
        // Yield to other threads
        if(yield) 
          uBit.sleep(0);

        // If there's any time left to wait
        long  waitTime = duration-(system_timer_current_time_us()-start);
        if(waitTime>0)
            _wait_us(waitTime);
    }

    /*
     * Write a bit
     * @param ioPin The MicroBitPin pin to use. 
     * @param one A boolean: true indicates send a 1; false indicates send a 0
     */
    void writeBit(_GPIO ioPin, bool one, bool finalYield) {
        // Ensure recovery time
        setPinValue(ioPin, 1);
        _wait_us(TIME_RECOV);

        // Start bus transaction
        unsigned long startTime = system_timer_current_time_us();
        setPinValue(ioPin, 0);
        // Time sensitive delay
        _wait_us(one ? TIME_ONE_LOW : TIME_ZERO_LOW);
        // Restore the bus
        setToOutput(ioPin);
        setPinValue(ioPin, 1);
    
        // Wait out rest of slot 
        waitOut(startTime, TIME_SLOT, finalYield);
    }

    /* 
     * Write a full byte
     * @param ioPin the MicroBitPin pin to use.
     * @param b the byte to send
     */
    void writeByte(_GPIO ioPin, uint8_t b, bool finalYield) {
        for(int i=0;i<8;i++,b>>=1) {
            writeBit(ioPin, (b & 0x01), i!=7);
        }
    }

    /*
     * Read a single bit
     * @param ioPin the MicroBitPin pin to read from
     * @return true if the bit is a 1; false otherwise
     */
    bool readBit(_GPIO ioPin) {

#if DEBUG
    MicroBitPin* indicate = &uBit.io.P1;

#if MICROBIT_CODAL
        _GPIO indicatePin = 0;
#else
        gpio_t indicateObj;
        _GPIO indicatePin = &indicateObj;
        gpio_init(indicatePin, indicate->name);
        setToOutput(indicatePin);
#endif
        setPinValue(indicatePin, 0);
#endif 
        // Ensure recovery time
        setToOutput(ioPin);
        setPinValue(ioPin,1);
        _wait_us(TIME_RECOV);
 
        // Start the transaction 
        setPinValue(ioPin, 0);

        setToInput(ioPin);
        setPinValue(ioPin, 1);
        // Start high (default) 
        bool b = true; 

        // Sample for ~70uS after releasing 
#if MICROBIT_CODAL
        uint32_t maxCounts = 25;
#else
        // v1: 115 uS for 200 iterations; 0.575uS/iteration
        _wait_us(0);  // Wait for ~6uS
        uint32_t maxCounts = (int)(TIME_SLOT/1.705);
#endif
        do {
            // If the bus goes low, its a 0
            b = b && getPinValue(ioPin);
        } while(maxCounts-->0);

#if DEBUG       
        setPinValue(indicatePin, 0);
#endif 
        // Switch back to output
        setPinValue(ioPin,1);

#if DEBUG       
        setPinValue(indicatePin, 0);
#endif 

        return b;
    }

    /* 
     *  Read the DS18B20 Temperature from Scratch pad and confirm success (via High/Low and CRC).
     *  @param ioPin the MicroBitPin pin to use
     *  @param temp the temperature (on success) or 
     */
    // Assumes configuration and HIGH/LOW set already.
    bool readScratchpad(_GPIO ioPin, float& temp) {
        uint8_t data[9];
        int16_t value;
        uint8_t crc=0;
        // Read each byte
        for(int j = 0; j<9; j++) {
            // read each bit (LSB to MSB)
            uint8_t b = 0;
            for(int i=0; i<8; i++) {
                bool bit = readBit(ioPin);
                b |= (bit<<i);
                bool lsb = crc & 0x1;
                crc >>= 1;  // Shift CRC to left
                if(bit != lsb)  // bit xor lsb 
                  crc ^= 0x8C;
            }
            data[j] = b;
        }     
        value = data[1];
        value <<= 8;
        value |= data[0];
        temp = value;
        temp /= 16.0;
#if DEBUG 
{
char buffer[24];
sprintf(buffer, "data: %X %X %X %X %X %X %X %X %X\n", data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8]);
loopUntilSent(buffer);
sprintf(buffer, "crc: %X\n", crc);
loopUntilSent(buffer);
}
#endif 
        return crc==0 && data[2]==HIGH_ALARM && data[3]==LOW_ALARM;
    }

    /*
     * Reset the DS18B20 on the designated pin.
     * @param ioPin the pin to use for the bus
     * @returns true if a device is detected on the bus following the reset; false otherwise
     */
    bool resetAndCheckPresence(_GPIO ioPin) {
        // Set pin to High (and get I/O object)
        setToOutput(ioPin);
        setPinValue(ioPin, 1);
        _wait_us(TIME_POWER_UP); // Possible power-up time

        // Set ioPin to output / apply reset signal
        setPinValue(ioPin, 0);
        _wait_us(TIME_RESET_LOW);     // Wait for duration of reset pulse

        // Return pin to input for presence detection 
        // TODO: Review Trick below (scope)
        // Trick to improve absence detection when floating

        setToInput(ioPin);
        setPinValue(ioPin, 1);
        _wait_us(TIME_POST_RESET_TO_DETECT);


// TODO:  Update this based on CODAL vs DAL
#if MICROBIT_CODAL
        int maxCounts = 14;   // Hand tuned values...Full presence period sample
#else 
        // v1: 165 uS for 200 iterations; 0.825 uS/Iteration
        int maxCounts = (int)(TIME_PRESENCE_DETECT/1.705);
#endif

        // Check for presence pulse
        bool presence = false;        
        do {
            presence =  presence || (getPinValue(ioPin) == 0);
        } while (maxCounts-- > 0);

        // Confirm that it's released
        bool release = getPinValue(ioPin);

        // Success if the pin was pulled low and went high again
        bool success = presence && release;

#if DEBUG
loopUntilSent("\n B pres= ");
loopUntilSent(presence);
loopUntilSent(" relese= ");
loopUntilSent(release);
loopUntilSent("\n");
#endif 
        return success; // Return success or failure
    }
}