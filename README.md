# ds-temp

This extension allows the micro:bit to use the Dallas Semiconductor DS18B20 temperature sensor.

# Getting the Temperature

```sig
dstemp.celsius(pin: DigitalPin) : number 
```

Get the current temperature in Celsius.  Returns -Infinity on error.
# Errors


```sig
dstemp.sensorError(errCallback: (ErrorMessage: string, ErrorCode: number, Port: number) => void) { 
```

Report on any errors

- ||ErrorMessage|| will be a string describing the error
- ||ErrorCode|| will be a numeric code
- ||Port|| will indicate which specific port encountered the error (if multiple sensors are connected)

# Recommended usage

It's best to capture the temperature in a variable and only use it if the value isn't -Infinity.  For example:

```block
temp = dstemp.celsius(DigitalPin.P0)
if (temp > -300) {
    basic.showString("" + (temp))
}
```


<script src="https://makecode.com/gh-pages-embed.js"></script>
<script>makeCodeRender("{{ site.makecode.home_url }}", "{{ site.github.owner_name }}/{{ site.github.repository_name }}");</script>
