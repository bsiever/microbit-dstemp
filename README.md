# microbit-pxt-dstemp

This extension allows the micro:bit to use the Dallas Semiconductor DS18B20 temperature probe.

# Getting the Temperature

```sig
dstemp.celsius(pin: DigitalPin) : number 
```

Get the current temperature in Calsius.  Returns -Infinity on error.
# Errors


```sig
dstemp.sensorError(errCallback: (ErrorMessage: string, ErrorCode: number, Port: number)
```

Report on any errors.  

# Recommended usage

It's best to capture the temperature in a variable and only use it if the value isn't -Infinity.  For example:...


