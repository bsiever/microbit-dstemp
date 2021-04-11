serial.writeLine("starting...")
serial.writeLine("Press A to show temperature")
serial.writeLine("If sensor is connected it should show the temperature")
serial.writeLine("If sensor is NOT connected it should show -Infinity (for temp) and an error message")

input.onButtonPressed(Button.A, function () {
    serial.writeString("temp = ")
    serial.writeNumber(dstemp.celsius(DigitalPin.P0))
    serial.writeString("\n")
//      basic.showNumber(dstemp.celsius(DigitalPin.P0))
})

dstemp.sensorError(function (errorMessage, errorCode, port) {
    serial.writeString("\n");
    serial.writeString(errorMessage);
    serial.writeString(" code=");
    serial.writeNumber(errorCode);
    serial.writeString(" port=");
    serial.writeNumber(port);
    serial.writeString("\n");
//    basic.showString("err")
//    basic.showNumber(ErrorCode)
})

