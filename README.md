# dstemp

```package
dstemp=github:bsiever/microbit-dstemp
```


This extension allows the micro:bit to use the Dallas Semiconductor DS18B20 temperature sensor.

# Hardware

This module supports use of one or more DS18B20 temperature sensors.  Each sensor must be connected to a separate pin and must include a 4.7k Ohm pull-up resistor between 3V and the signal. 

Common wiring is:

- Micro:bit GND to GND (black wire) of the sensor
- Micro:bit 3V to Vdd (red wire) of the sensor
- Micro:bit I/O pin, like `P0`, to the data in/out (white wire) of the sensor

# Getting the Temperature

```sig
dstemp.celsius(pin: DigitalPin) : number 
```

Get the current temperature in Celsius.  Returns `-Infinity` on error.
# Errors


```sig
dstemp.sensorError(errCallback: (errorMessage: string, errorCode: number, port: number) => void) { 
```

Report on any errors

- `||errorMessage||` will be a string describing the error
- `||errorCode||` will be a numeric code
- `||port||` will indicate which specific port encountered the error (if multiple sensors are connected)

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
