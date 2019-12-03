//% color=#0000FF 
//% icon="\uf2c8"
//% block="DS Temp"
namespace dstemp {
    // TODO: Review better approach.  These are redundant when on the platform (C-Side)
    // TODO: Is there a way to "share" fields/variables (without an object)
    //% whenUsed
    let errorHandler:Action = null;
    //% whenUsed
    let errorObjectIdx : number = 0;
    //% whenUsed
    let errorPort : number = -1;

    // TODO: Are these in ROM?  Is there a way to unify with C-Code constants
    // TODO: Localization
    const errorMsgs  = [ "No Error", "Not Connected", "Start Error", "Read Timeout"];

    //% blockId="celsius" block="Temp. in Celsius on %pin|"
    //% shim=dstemp::celsius
    //% parts=dstemp trackArgs=0
    export function celsius(pin: DigitalPin) : number {
        return 32.6;
    }

    // Helper function
    //% shim=dstemp::setErrorHandler
    export function setErrorHandler(a: Action) {
        errorHandler = a; 
    }

    // Helper function
    //% shim=dstemp::getErrorObjectIdx
    export function getErrorObjectIdx() : number {
        return errorObjectIdx;
    }

    // Helper function
    //% shim=dstemp::getErrorPort
    export function getErrorPort() : number {
        return errorPort;
    }

    /**
     * Set a handler for errors 
     * @param errCallback The error handler 
     */
    //% blockId="error" block="Temp. Sensor Error"
    //% draggableParameters="reporter"
    export function sensorError(errCallback: (ErrorMessage: string, ErrorCode: number, Port: number) => void) { 
        if(errCallback) {
            errorHandler = () => {
                let i  = getErrorObjectIdx(); 
                let p = getErrorPort();
                errCallback(errorMsgs[i], i, p);          
            };
        } else {
            errorHandler = null;
        }
        setErrorHandler(errorHandler);
    };
}


/*


    /*
    let count : number = 1;

    // This is the function that will be used in the simulation.  The c-function (ctest()) 
    // will be used on the actual platform
    // % blockId="dstemp_ctest" block="Temp. Sensor Ctest"
    // % shim=dstemp::ctest
    export function ctest() {
        console.log("test")
        error = "c"+count
        count++
        errorHandler()
    }
    
    //% blockId="setup" block="DS Temp. Probe at pin %pin"
    //% shim=dstemp::setup
    export function setup(pin: DigitalPin) {
        console.log("test")
        basic.showString("S4!")
    }

if (ble_running())
        return MICROBIT_NOT_SUPPORTED;
        
          if (status & MICROBIT_RADIO_STATUS_INITIALISED)

 if (radioEnable() != MICROBIT_OK) return 0;         
       
 
  //   // % blockId="celsius4" block="Temp. in Celsius 3"
 //   // % shim=dstemp::celsius
 //  export function celsius(): number {
 //       return 32.6;
 //    }

        */
