#   import libraries for UART communication
#   board to simplify pin Assignments
import board
import busio
import digitalio
import time



#   configure led
led = digitalio.DigitalInOut(board.D13)
led.direction = digitalio.Direction.OUTPUT

#   create UART instance for baudrate 9600 kb/s
gps_uart = busio.UART(board.TX, board.RX, baudrate=9600)


def get_NMEA_Message(datastring):
    #Isolate NMEA meassge
    nmea = datastring[datastring.find("$")+1:datastring.find(",")]
    #determine message
    return nmea

#Satelite Definition, assuming first part of message is blocked off

while True:
    data = gps_uart.read(32)

    if data is not None:
        led.value = True
        datastring = ''.join([chr(b) for b in data])  # convert to string
        print(datastring)
        print("Message: "+get_NMEA_Message(datastring))
        led.value = False
        time.sleep(1)
