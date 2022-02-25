# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import serial
import serial.tools.list_ports
import time
import atexit
import wx

STM32_USB_PID_FS = 22336


class UsbManager:
    UsbCommands = {
        "INFO": "INFO=0\n",
        "BAT": "INFO=1\n",
        "EUI": "INFO=2\n"
    }

    def __init__(self, memoryManager):
        self.m_mM = memoryManager
        self.m_port = None
        atexit.register(self.cleanup)

    def connecting(self):
        l_listPort = []
        l_indexBuffer = 0
        l_flagFound = True;
        while (l_flagFound):
            ports = serial.tools.list_ports.comports()
            for p in ports:
                if (p.pid == STM32_USB_PID_FS):
                    l_listPort = p.device
                    ++l_indexBuffer
                    l_flagFound = False
        return l_listPort

    def cleanup(self):
        print("cleanup")
        # close connections, ...

    # Добавить сканирование всех юсб, выделить только стм и пробовать к ним подключатся

    def readInfo(self, port):
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity='N',
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=100,
        )

        self.m_port = ser

        if (self.sendCommand(self.UsbCommands["INFO"])):
            answer = self.m_port.readline()

        encoding = 'utf-8'
        str1 = str(answer, encoding)

        str2 = str1.split(";")
        str3 = str2[0].split("=")
        str4 = str2[1].split("=")
        if (str3[0] == 'DEV'):
            self.m_mM.setNameDevice(str3[1])
        if (str4[0] == 'VERS'):
            self.m_mM.setVersionDevice(str4[1])
        print(str3, str4);

        self.readBattery();
        self.readNetworkData();
        return True;
        # if str1[0] == "VER":
        #     ver = str1[1]

    def readBattery(self):
        if (self.sendCommand(self.UsbCommands["BAT"])):
            answer = self.m_port.readline()

        encoding = 'utf-8'
        str5 = str(answer, encoding)

        # str2 = str1.split(";")
        str6 = str5.split("=")

        if (str6[0] == 'BAT'):
            self.m_mM.setBatteryPercent(str6[1])

    def readNetworkData(self):
        if (self.sendCommand(self.UsbCommands["EUI"])):
            answer = self.m_port.readline()

        encoding = 'utf-8'
        str1 = str(answer, encoding)

        str2 = str1.split(";")

        if (str2[0] == 'AppKey'):
            self.m_mM.setAppKey(str2[1])

        if (str2[2] == 'DevEui'):
            self.m_mM.setDevEui(str2[3])

        if (str2[4] == 'JoinEui'):
            self.m_mM.setJoinEui(str2[5])

    def sendCommand(self, command):

        if (self.m_port.is_open == False):
            return False

        self.m_port.write(command.encode())
        timeout = time.time() + 1 * 60

        while True:
            if time.time() > timeout:
                return False
            if self.m_port.in_waiting:
                return True
