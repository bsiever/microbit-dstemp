/**
* Bill Siever
* 2018-10-10
* 2021-02-27 CODAL and V2 updates
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


// Enable debugging: Debugging uses #ifdefs, so uncomment or comment out 
//#define DEBUG 1
// DEBUG uses ioPin P1 to indicate sampling of read (for timing calibration)
using namespace pxt;


#if MICROBIT_CODAL
// ********************* V2/CODAL Specific Functions ***********************

// Example of port mapping from V1 codebase 
// From: codal-nrf52/source/NRF52Pin.cpp
#ifdef NRF_P1   
    #define PORT (pin < 32 ? NRF_P0 : NRF_P1)
    #define PIN ((pin) & 31)
    #define NUM_PINS 48
#else
    #define PORT (NRF_P0)
    #define PIN (pin)
    #define NUM_PINS 32
#endif

    #define _wait_us(us)          system_timer_wait_cycles((us)==0? 1: (((10*500*(us))/470))) // Adjusted to 11
    #define _GPIO                   int
    static void setToInput(_GPIO pin)     { PORT->PIN_CNF[PIN] &= 0xfffffffc; }
    static void setToOutput(_GPIO pin)    { PORT->PIN_CNF[PIN] |= 3; }
    static void setPinValue(_GPIO pin, int val) { if (val) PORT->OUTSET = 1 << PIN; else PORT->OUTCLR = 1 << PIN;}
    static bool getPinValue(_GPIO pin)    { return (PORT->IN & (1 << PIN)) ? 1 : 0; }

    static void configTimer() {
        // Ensure that the external crystal is being used (higher precision)
        // and that the system timer is in 32-bit mode
        static NRF_TIMER_Type *timer = NULL;

        // If we haven't gotten the timer yet, do startup tasks, including getting the timer.
        if(timer == NULL) {
            NVIC_DisableIRQ(TIMER1_IRQn);
            // Ensure the HFCLOCK is running
            NRF_CLOCK_Type *clock = NRF_CLOCK;
            clock->TASKS_HFCLKSTART = 1;

            // Get the timer (ensures this is only done once)
            timer = NRF_TIMER1;
            // Disable timer
            timer->TASKS_STOP = 1;
            // Set bit mode
            timer->BITMODE = 3;  // Ensure 32-bit mode (error in CODAL was resulting in 24-bit mode)
            // Restart it
            timer->TASKS_START = 1;
            NVIC_EnableIRQ(TIMER1_IRQn);

            // Call timer calibration
            //system_timer_calibrate_cycles();
        }
    }

#else
// ********************* V1/AL Specific Functions ***********************
    // Map to NRF library 
    #define _wait_us(us)            wait_us(((us)>5)?(us)-5:0)
    #define _GPIO                   gpio_t*
    #define setToInput(pin)         gpio_dir((pin), PIN_INPUT)
    #define setToOutput(pin)        gpio_dir((pin), PIN_OUTPUT)
    #define setPinValue(pin, val)   gpio_write((pin), (val))
    #define getPinValue(pin)        gpio_read((pin))
#endif


#ifdef DEBUG
// If debugging is enabled create an "indicatePin" to assist timing tests
#if MICROBIT_CODAL
        _GPIO indicatePin = uBit.io.P1.name;
#else
        gpio_t indicateObj;
        _GPIO indicatePin = &indicateObj;
#endif
#endif

namespace dstemp { 

    // ************* Forward Decalarations
    void loopUntilSent(ManagedString str);
    void loopUntilSent(int str);
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

    const float ERROR_SENTINEL = -INFINITY;  // Sentinel value to return for errors. 

    const int HIGH_ALARM = 0xFF;  // High Alarm Value  (both set and confirmed; Should be non-zero)
    const int LOW_ALARM = 0x80;   // Low Alarm Value   (both set and confirmed; Should be non-zero)

    const int MAX_TRIES = 3;      // Max tries to attempt conversion before fail 

    // ************* State variables 
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
        errorObjectIdx = objIndex;
        errorPort = port - MICROBIT_ID_IO_P0;
        if(errorHandler) {
            pxt::runAction0(errorHandler);            
        }
    }

    /*  Configure the device for 12-bit conversion and set High/Low Alarm Values (Magic numbers too.)
     *  @param ioPin the IO pin to use
     * @returns true on (assumed) success; false on known failure
     */
    bool configure(_GPIO ioPin) {
        if(resetAndCheckPresence(ioPin) == false) {
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


// Calibration code that can be used to debug timing issues and 
// identify values for constants
#ifdef DEBUG
    void calibrate(_GPIO gpio) {
        // Misc tests.
#if MICROBIT_CODAL 
        loopUntilSent("CODAL");
#else
        loopUntilSent("DAL");
#endif
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
        _wait_us(1000);
        setPinValue(gpio, 1);

        // Calibrate input loop
        // v1: 1147 uS for 2000 iterations; 0.5735/iteration
        // v2: 127 uS for 2000 iterations ; 0.0635uS/ iteration
        setPinValue(indicatePin, 0);
        setToInput(gpio);
        uint32_t maxCounts =  2000;
        bool b = true;
        do {
            // If the bus goes low, its a 0
            b = b && getPinValue(gpio);
        } while(maxCounts-- > 0);
        setPinValue(indicatePin, 1);
        _wait_us(100);


        setPinValue(indicatePin, 0);

        // v1: 3551 uS for 2000 iterations; 1.775 uS/Iteration
        // v2: 346 for 2000 iterations ; 0.173uS/iter
        maxCounts = 2000;
        setToInput(gpio);
        // Check for presence pulse
        bool presence = false;        
        do {
            presence = presence || (getPinValue(gpio) == 0);
        } while (maxCounts-- > 0);
        setPinValue(indicatePin, 1);
    
        _wait_us(200);
        // v1: Aim for exactly 200uS
        //maxCounts = (int)(200/1.775);
        // v2: Aim for exactly 200uS
        maxCounts = (int)(200/0.173);
        setPinValue(indicatePin, 0);
        setToInput(gpio);
        // Check for presence pulse
        presence = false;        
        do {
            presence = presence || (getPinValue(gpio) == 0);
        } while (maxCounts-- > 0);
        setPinValue(indicatePin, 1);
    
        setToOutput(gpio);
    }
#endif



     //% 
    float celsius(int pin) {
        // Only needs to be done once, but done every call...
#if MICROBIT_CODAL
        // CODAL may not be using external crystal by default; Update it
        // May also be using 24-bit timer
#ifdef SOFTDEVICE_PRESENT
        if (!ble_running()) // Only configTimer if either no soft-dev or no ble
#endif
                configTimer();
#endif
        // Get corresponding I/O ioPin Object
        MicroBitPin *mbp = getPin(pin);  // This returns a "uBit.io.P0" type thing
#if MICROBIT_CODAL
        _GPIO gpio = mbp->name;
#else
        gpio_t gpioObj;
        _GPIO gpio = &gpioObj;
        gpio_init(gpio, mbp->name);
#endif
 
 // If debugging, configure the indicate pin
#ifdef DEBUG
#if MICROBIT_CODAL
#else
    MicroBitPin* indicate = &uBit.io.P1;
    gpio_init(indicatePin, indicate->name);
#endif

    setToOutput(indicatePin);
    setPinValue(indicatePin, 0);
#endif


#ifdef DEBUG
    // Optional calibration (rather than actually running)
    // calibrate(gpio);
    // return 0;
#endif 


#ifdef DEBUG 
        {
        char buffer[24];
        sprintf(buffer, "pin: %d\n", mbp->name);
        loopUntilSent(buffer);
        loopUntilSent("Celsius Block\n");
        }
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
        success = false;
        for(int maxIterations = 20; maxIterations>0; maxIterations--) {
            if(readBit(gpio) == 0) {
                success = true;
                break;
            } else {
                // Wait for conversion to complete. 
                uBit.sleep(0);
            }
        }
        // If not successful, error
        if(success==false) {
            error(4, pin);
            goto return_error;
        }

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
                    errorObjectIdx = 0;
                    errorPort = pin;
                    // Return to input
                    setToInput(gpio);
                    return temp;
                }
            }
        } 
        // ERROR: Max Read Tries 
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
  
#ifdef DEBUG
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
     * Write a bit
     * @param ioPin The MicroBitPin pin to use. 
     * @param one A boolean: true indicates send a 1; false indicates send a 0
     */
    void writeBit(_GPIO ioPin, bool one, bool finalYield) {
        // Ensure recovery time
        setPinValue(ioPin, 1);
        setToOutput(ioPin);
        _wait_us(TIME_RECOV);

        // Start bus transaction
        setPinValue(ioPin, 0);
        // Time sensitive delay
        _wait_us(one ? TIME_ONE_LOW : TIME_ZERO_LOW);
        // Restore the bus
        setToInput(ioPin);
        setPinValue(ioPin, 1);
    
        // Wait out rest of slot 
        _wait_us(one ? TIME_SLOT : 1);
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
        // Ensure recovery time
        setToOutput(ioPin);
        setPinValue(ioPin,1);
        _wait_us(TIME_RECOV);
 
        // Start the transaction 
        setPinValue(ioPin, 0);
        _wait_us(1); // Updated to 1 for minimum wait
        setPinValue(ioPin, 1);
        setToInput(ioPin);

        // Start high (default) 
        bool b = true; 

#ifdef DEBUG       
        setPinValue(indicatePin, 1);
#endif 

        // Sample for ~70uS after releasing 
#if MICROBIT_CODAL
        // v2: 156 uS for 2000 iterations ; 0.077uS/ iteration
        uint32_t maxCounts = (int)(TIME_SLOT/0.0635);
#else
        // v1: 115 uS for 200 iterations; 0.575uS/iteration
        _wait_us(0);  // Wait for ~6uS
        uint32_t maxCounts = (int)(TIME_SLOT/0.57);
#endif
        do {
            // If the bus goes low, its a 0
            b = b && getPinValue(ioPin);
        } while(maxCounts-->0);

        // Switch back to output
        setPinValue(ioPin,1);

#ifdef DEBUG       
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
#ifdef DEBUG 
        {
        char buffer[40];
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
        setPinValue(ioPin, 1);
        setToInput(ioPin);
        _wait_us(TIME_POST_RESET_TO_DETECT);

#if MICROBIT_CODAL
        // v2: 462.5 for 2000 iterations ; 0.231uS/iter
        // Padded down (the "release" needs to be complete)
        int maxCounts = (int)(TIME_PRESENCE_DETECT/0.1);   // Hand tuned values...Full presence period sample
#else 
        // v1: 1.705 uS/Iteration
        int maxCounts = (int)(TIME_PRESENCE_DETECT/1);
#endif

        // Check for presence pulse (pulling line low)
#ifdef DEBUG
        setPinValue(indicatePin,1);
#endif
        bool presence = false;        
        do {
            presence = presence || (getPinValue(ioPin) == 0);
        } while (maxCounts-- > 0);

        // Confirm that it's released
#ifdef DEBUG
        setPinValue(indicatePin,0);
#endif
        bool release = getPinValue(ioPin)==1;

        // Success if the pin was pulled low and went high again
        bool success = presence && release;

#ifdef DEBUG
        loopUntilSent("\npres= ");
        loopUntilSent(presence);
        loopUntilSent(" relese= ");
        loopUntilSent(release);
        loopUntilSent("\n");
#endif 
        return success; // Return success or failure
    }
}