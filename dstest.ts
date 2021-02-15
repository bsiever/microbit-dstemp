serial.writeLine("starting...")
// basic.forever(function () {
//     serial.writeString("loop\n")
//     basic.pause(1000)
// })

input.onButtonPressed(Button.A, function () {
    serial.writeString("temp = ")
    serial.writeNumber(dstemp.celsius(DigitalPin.P0))
    serial.writeString("\n")
//      basic.showNumber(dstemp.celsius(DigitalPin.P0))
})

dstemp.sensorError(function (ErrorMessage, ErrorCode, Port) {
    serial.writeString("\n");
    serial.writeString(ErrorMessage);
    serial.writeString(" code=");
    serial.writeNumber(ErrorCode);
    serial.writeString(" port=");
    serial.writeNumber(Port);
    serial.writeString("\n");
//    basic.showString("err")
//    basic.showNumber(ErrorCode)
})

/*

let temp = 0
bluetooth.onBluetoothConnected(function () {
    basic.showIcon(IconNames.House)
})
dstemp.sensorError(function (ErrorMessage, ErrorCode, Port) {
    serial.writeLine(ErrorMessage)
    bluetooth.uartWriteString(ErrorMessage)
    if (ErrorCode == 1) {
        basic.showIcon(IconNames.No)
    } else if (ErrorCode == 2) {
        basic.showIcon(IconNames.Sad)
    }
})
bluetooth.onBluetoothDisconnected(function () {
    basic.showIcon(IconNames.Sword)
})
bluetooth.startUartService()
basic.forever(function () {
    temp = dstemp.celsius(DigitalPin.P0)
    if (temp > -20) {
        basic.showNumber(Math.round(temp))
        serial.writeValue("tmp", temp)
        bluetooth.uartWriteValue("t", temp)
    }
})

*/