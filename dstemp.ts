 //% color=#0000FF 
//% icon="\uf2c8"
//% block="DS Temp"
namespace dstemp {
    //% whenUsed
    let errorHandler:Action = null;
    //% whenUsed
    let errorObjectIdx : number = 0;
    //% whenUsed
    let errorPort : number = -1;

    // TODO: Localization
    const errorMsgs  = [ "No Error", "Not Connected", "Start Error", "Read Timeout", "Conversion Failure"];

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
