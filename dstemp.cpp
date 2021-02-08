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

uint64_t wait_us(uint64_t delay) {
    uint64_t start = system_timer_current_time();
    while(system_timer_current_time()-start < delay) {
        // Intentionally empty;
    }
    return 0;
}

namespace dstemp { 

    // ************* Forward Decalarations
    void loopUntilSent(ManagedString str);
    void loopUntilSent(int str);
    void waitOut(unsigned long start, unsigned long duration, bool yield = true);
    void writeBit(MicroBitPin* ioPin, bool one, bool finalYield = true);
    void writeByte(MicroBitPin* ioPin, uint8_t b, bool finalYield = true);
    bool readBit(MicroBitPin* ioPin);
    bool readScratchpad(MicroBitPin* ioPin, float& temp);
    bool reset(MicroBitPin* ioPin);
    bool configure(MicroBitPin* pin);
    bool startConversion(MicroBitPin* ioPin);

    // ************* Timing related constants 
    // Times related to time slots and read/write operations
    const int TIME_SLOT = 90;   // Time slot length = 60-120uS T_SLOT; Rounded up from min
    const int TIME_RECOV = 15;   // Recovery time between bits = 1uS T_REC; Rounded up for pull-up time.
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
    bool configure(MicroBitPin* ioPin) {
        if(reset(ioPin) == false) {
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
    bool startConversion(MicroBitPin* ioPin) {
        if(reset(ioPin) == false) {
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
        // Get corresponding I/O ioPin Object
        MicroBitPin *mbp = getPin(pin);

        // Set to input by default
        (void)mbp->getDigitalValue();

#if DEBUG
        loopUntilSent("Celsius Block\n");
#endif
        bool success = false;
        // 1. Check for valid device, configure it for conversion, and start conversion
        for(int tries=0;tries<MAX_TRIES;tries++) {
            // A. Configure Device
            if(configure(mbp)==false) {
                error(1, pin);
                return ERROR_SENTINEL;
            }
            
            // B. Start conversion
            if(startConversion(mbp)) {
                success = true;
                break; // Leave the loop
            }            
        }

        // If unable to start conversion, return error
        if(success==false) {
            error(2, pin);
            return ERROR_SENTINEL;
        }

        // 2. Wait for conversion to complete 
        while(readBit(mbp) == 0) {
            // Wait for conversion to complete. 
            uBit.sleep(0);
        }
     //   uBit.sleep(TIME_CONVERSION); // let other tasks run

        // 3. Retrieve Data 
        for(int tries=0;tries<MAX_TRIES;tries++) {
            // If reset is successful, request and read data
            if(reset(mbp)) {
                // Write ROM command: Skip ROM Command CCh: To address all devices
                writeByte(mbp, 0xCC);
                // Write Function command - Read scratchpad 
                writeByte(mbp, 0xBE);        
                // Read 8 bytes of scratch pad
                float temp;
                success = readScratchpad(mbp, temp);

                if(success) {
#if DEBUG
                    loopUntilSent("s");
#endif
                    errorObjectIdx = 0;
                    errorPort = pin;
                    return temp;
                }
            }
        } 
        // ERROR: Max Read Tries 
        // Call error Handler
        // TODO: Unify device code 
        error(3, pin);
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

        // If there's any time left to wait, wait
        int  waitTime = duration-(system_timer_current_time_us()-start);
            if(waitTime>0) {
                wait_us(waitTime);
            }
    }

    /*
     * Write a bit
     * @param ioPin The MicroBitPin pin to use. 
     * @param one A boolean: true indicates send a 1; false indicates send a 0
     */
    void writeBit(MicroBitPin* ioPin, bool one, bool finalYield) {
        // Ensure recovery time
        ioPin->setDigitalValue(1);
        wait_us(TIME_RECOV);

        // Start bus transaction
        unsigned long startTime = system_timer_current_time_us();
        ioPin->setDigitalValue(0);
        // Time sensitive delay
        wait_us(one ? TIME_ONE_LOW : TIME_ZERO_LOW);
        // Restore the bus
        ioPin->setDigitalValue(1);
    
        // Wait out rest of slot 
        waitOut(startTime, TIME_SLOT, finalYield);
    }

    /* 
     * Write a full byte
     * @param ioPin the MicroBitPin pin to use.
     * @param b the byte to send
     */
    void writeByte(MicroBitPin* ioPin, uint8_t b, bool finalYield) {
        for(int i=0;i<8;i++,b>>=1) {
            writeBit(ioPin, (b & 0x01), i!=7);
        }
    }

    /*
     * Read a single bit
     * @param ioPin the MicroBitPin pin to read from
     * @return true if the bit is a 1; false otherwise
     */
    bool readBit(MicroBitPin* ioPin) {

#if DEBUG
        MicroBitPin* indicate = getPin(MICROBIT_PIN_P1);
        indicate->setDigitalValue(0);
#endif 
        // Ensure recovery time
        ioPin->setDigitalValue(1);
        wait_us(TIME_RECOV);
 
        // Start the transaction 
        unsigned long startTime = system_timer_current_time_us();
#if DEBUG       
        indicate->setDigitalValue(1);
#endif 
        ioPin->setDigitalValue(0);

        // Not needed; Switching takes 1uS or more...
        //wait_us(TIME_READ_START);

        // Switch to input 
        (void)ioPin->getDigitalValue();
        // Check for a "0" 
        bool b = true; 
        while(system_timer_current_time_us() - startTime < TIME_SLAVE_WRITE_END) {
            // If the bus goes low, its a 0
            if(ioPin->getDigitalValue() == 0) {
                b = false;
            }
        }
        // wait_us(TIME_READ_OFFSET);

        // Sample:  Timing has been hand calibrated.  
        //           Tests indicate that sample occurs at ~14.2uS from low (~end of 15uS window spec)
        // b = (ioPin->getDigitalValue() == 1);
#if DEBUG       
        indicate->setDigitalValue(0);
#endif 

        // Wait out rest of slave access time 
        // waitOut(startTime, TIME_SLAVE_WRITE_END);

        // Switch back to output
        ioPin->setDigitalValue(1);

        // Wait out rest of slot 
        waitOut(startTime, TIME_SLOT);
#if DEBUG       
        indicate->setDigitalValue(0);
#endif 

        return b;
    }

    /* 
     *  Read the DS18B20 Temperature from Scratch pad and confirm success (via High/Low and CRC).
     *  @param ioPin the MicroBitPin pin to use
     *  @param temp the temperature (on success) or 
     */
    // Assumes configuration and HIGH/LOW set already.
    bool readScratchpad(MicroBitPin* ioPin, float& temp) {
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
    bool reset(MicroBitPin* ioPin) {
        // Set pin to High (and get I/O object)
        ioPin->setDigitalValue(1);
        wait_us(TIME_POWER_UP); // Possible power-up time

        // Set ioPin to output / apply reset signal
        ioPin->setDigitalValue(0);
        wait_us(TIME_RESET_LOW);     // Wait for duration of reset pulse

        // Return pin to input for presence detection 
        // TODO: Review Trick below (scope)
        // Trick to improve absence detection when floating
        ioPin->setDigitalValue(1);  
        (void)ioPin->getDigitalValue();  // Turn to input
        wait_us(TIME_POST_RESET_TO_DETECT);

        // Check for presence pulse
        unsigned long startTime = system_timer_current_time_us();
        bool presence = false;        
        do {
            presence =  (ioPin->getDigitalValue() == 0);
        } while ((presence == false) && (system_timer_current_time_us() - startTime < TIME_PRESENCE_DETECT));


// THIS IS THE PROBLEM....THE RELEASE SEEMS TO TRIGGER A BIT / READ 
      //  wait_us(25);
        // Hold it down
        ioPin->setDigitalValue(0);
        waitOut(startTime, TIME_PRESENCE_DETECT);
        bool release = true;
        // // If the pulse was present, wait for release
        // bool release = false;
        // if(presence) {
        //     do {
        //         release = (ioPin->getDigitalValue() == 1);
        //     } while((release==false) && (system_timer_current_time_us() - startTime < TIME_PRESENCE_DETECT));
        // }

        // // Write a 1 to avoid glitches on ROM command write
        ioPin->setDigitalValue(1);

        // Success if the pin was pulled low and went high again
        bool success = presence && release;
        if(success) {
            waitOut(startTime, TIME_RESET_HIGH);
        }

#if DEBUG
loopUntilSent("\n B pres= ");
loopUntilSent(presence);
loopUntilSent(" relese= ");
loopUntilSent(release);
loopUntilSent("\n");
#endif 
        return success; // Return success or failure
    }








/*

 inline float round(float v, int d) {
        // Round away from 0. 
        int sign = 1;
        if (v == 0.0) {
            return v;
        } else if(v<0) {
            sign = -1;
        }
        float frac = sign / (2.0 * d);
        int intv = (int)((v + frac) * d);
        return  intv / ((float)d);
    }


*/

    /*

      float roundTests(int case) {
        switch(case) {
            case 0:
                return round(.12,4);  // A: 0
            case 1:
                return round(-.12, 4);// B: 0
            case 2:
                return round(.125,4);// C: 0.25
            case 3:
                return round(-.125, 4);// D: -0.25
            case 4:
                return round(.126,4); // E: .25
            case 5:
                return round(-.126, 4); // F: -0.25
            case 6:
                return round(0.0, 4); // G: 0
            case 7:
                return round(-0.0, 4); // H: 0
            default:  // I: 1
                return 2;
            }
      }
    */

    /*    int count = 6;
    ManagedString error("c err"); 
    StringData *msg; 
*/


/*
    // % 
    void ctest() {
        if (ble_running())
          uBit.display.scroll("BLE");

  //      uBit.display.scroll("go");
        pxt::runAction0(errorHandler);



    }

*/

   /**

   */

    /**
    Phase 1:  Use external power and external Pull-up
    Phase 2:  Try internal pull-up
    Phase 3:  Parasitic power and internal pull-up (crazy!)
    */
}

/*
    void setupioPin(int ioPin) {
        // Setup the ioPin (Phase 1)
       // ioPin = new MicroBitPin(ioPin, PullUp, OpenDrain, 1);

    }


    // % 
    void setup(int ioPin) {
        uBit.display.scroll("p");
        uBit.display.scroll(ioPin);
// Setup ioPin
//  Input Pullup: CNF[ioPin].PULL = 3  // Pullup
//  CNF[ioPin].DIR = 
//  CNF[ioPin].Input = 
//  CNF[ioPin].Drive = 6  // Std 0 &  Disconnect 1  

    } 
*/