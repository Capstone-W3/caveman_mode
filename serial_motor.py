# serial_motor is the control of the collection mechanism's motor

import serial
import sys
import os
import time

class SerialMotor():
    def __init__(self):
        # Serial connections in linux are accessed as files, this looks
        # for the correct formatted file name, which is typically in
        # /dev/ttyACM<0,1,2>, because sometimes the name changes when
        # the arduino is unplugged and plugged back in.  Stores this
        # in a text file
        os.system('ls /dev/ttyACM* > port.txt')

        # Get the port name and remove the trailing newline that the '>'
        # operator puts into the text file
        self.port_name = open("port.txt").read()[:-1]
        
        # delete the text file with the port name
        os.system('rm port.txt')

        print("motor serial port: %s" % self.port_name)

        # store the arduino and the connectedness of it
        self.arduino = None
        self.connected = False

        self.baud_rate = 9600
        
        
    # Attempt to connect to the motor over serial
    # returns bool, true if successful, false otherwise
    def Connect(self):
        # Serial connections (or /dev/?) can only ba acccessed with superuser
        # permissions
        try:
            self.arduino = serial.Serial(self.port_name, self.baud_rate)
        except Exception as e:
            print("Exception: %s" % str(e))
            print("Must have SuperUser permissions to open serial port")
            print("Did you use \'sudo\'?")
            self.arduino = None

        if self.arduino == None:
            self.connected = False
        else:
            self.connected = True
            # if the connection is complete, pause a second to let
            # the serial handshake complete otherwise the first
            # command might get lost
            time.sleep(1)

        
        return self.connected

    def StartMotor(self):
        if self.connected:        
            self.arduino.write('yyy')

    def StopMotor(self):
        if self.connected:
            self.arduino.write('nnn')

    def Disconnect(self):   
        if self.connected:
            self.arduino.close()
            self.connected = False

        
if __name__ == '__main__':
    motor = SerialMotor()
    connected = motor.Connect()
    print('Connected: %s' % connected)
    motor.StartMotor()
    time.sleep(5)
    motor.StopMotor()
    time.sleep(1)
    motor.Disconnect()
