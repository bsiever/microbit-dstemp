# dstemp

```package
dstemp=github:bsiever/microbit-dstemp2w
```

This extension allows the micro:bit to use the Dallas Semiconductor DS18B20 temperature sensor with just two wires (parasite power mode).

# Hardware

This module supports use of one or more DS18B20 temperature sensors.  Each sensor must be connected to a separate pin and must be configured for parasite power mode, where the sensor's power and ground are both connected to ground.

Common wiring is:

- Micro:bit GND to GND (black wire) and Vdd (red wire) of the sensor
- Micro:bit I/O pin, like ||P0||, to the data in/out (white wire) of the sensor

# Getting the Temperature

```sig
dstemp.celsius(pin: DigitalPin) : number 
```

Get the current temperature in Celsius.  Returns `-Infinity` on error.
# Errors


```sig
dstemp.sensorError(errCallback: (ErrorMessage: string, ErrorCode: number, Port: number) => void) { 
```

Report on any errors

- `ErrorMessage` will be a string describing the error
- `ErrorCode` will be a numeric code
- `Port` will indicate which specific port encountered the error (if multiple sensors are connected)

# Recommended usage

It's best to capture the temperature in a variable and only use it if the value isn't `-Infinity`.  Since -300 C is below absolute zero, ensuring the temperature is over -300 is sufficient.  For example:

```block
temp = dstemp.celsius(DigitalPin.P0)
if (temp > -300) {
    basic.showString("" + (temp))
}
```


<script src="https://makecode.com/gh-pages-embed.js"></script>
<script>makeCodeRender("{{ site.makecode.home_url }}", "{{ site.github.owner_name }}/{{ site.github.repository_name }}");</script>
