import serial
import threading
import time
import subprocess
import rospy


# Taken from:
# https://bitbucket.org/MartinAppo/diploaf-2017/src/master/hardware_module/src/hardware_module/comport_mainboard.py?fileviewer=file-view-default


class ComportMainboard(threading.Thread):

    connection = None
    connection_opened = False

    def __init__(self):
        threading.Thread.__init__(self)

    def open(self):
        try:
            ports = subprocess.check_output('ls /dev/ttyACM0', shell=True).split('\n')[:-1]
        except:
            print('mainboard: /dev/ttyACM empty')
            return False
        self.connection_opened = False
        for port in ports:  # analyze serial ports
            try:
                while not self.connection_opened and not rospy.is_shutdown():
                    self.connection = serial.Serial(port, baudrate=115200, timeout=0.8, dsrdtr=True)
                    self.connection_opened = self.connection.isOpen()
                    time.sleep(0.5)
                self.connection.flush()
                print("mainboard: Port opened successfully")
            except Exception as e:
                print(e)
                continue

        return self.connection_opened

    def write(self, comm):
        if self.connection is not None:
            try:
                self.connection.write(comm + '\n')
            except:
                print('mainboard: err write ' + comm)

    def read(self):
        full_command, single_character = "", ""
        while single_character != "\n":
            single_character = self.connection.read()
            full_command += single_character
        return full_command

    #launch_wheel_motors(-20,20,0)
    def launch_wheel_motors(self,speed1, speed2, speed3):
        if self.connection_opened:
            self.write("sd:{}:{}:{}".format(speed1, speed2, speed3))

    #launch_thrower(125-250)
    def launch_thrower(self, speed):
        if self.connection_opened:
            self.write("d:{}".format(speed))

    #launch servos(1: 720-825; 2: <700 - continuous mode; 2300 -> 100 - fast movement)
    def launch_servos(self, servo1, servo2):
        if self.connection_opened:
            self.write("sv:{}:{}".format(servo1, servo2))

    #get_wheel_speeds()
    def get_wheel_speeds(self):
        if self.connection_opened:
            self.write("gs\n")
            return self.connection.readline()


    #send_referee_signal_string("START")
    def send_referee_signal_string(self, signal_string):
        if self.connection_opened:
            self.write("rf:{}".format(signal_string))

    def close(self):
        if self.connection is not None and self.connection.isOpen():  # close coil
            try:
		self.launch_servos(0,0)
                self.connection.close()
		
                print('mainboard: connection closed')
            except:
                print('mainboard: err connection close')
            self.connection = None

    def run(self):
        if self.open():  # open serial connections
            print('mainboard: opened')
        else:
            print('mainboard: opening failed')
            self.close()
            return
