from evdev import InputDevice, categorize, ecodes, ff
import serial
from connect import Arduino
import os
import time
import glob
import sys
import cursor

# Connect controller via bluetooth:
# https://pimylifeup.com/raspberry-pi-playstation-controllers/
# and trust device in bluetoothctl

# Controller Constants
# For all Buttons, 1 = Press, 0 = Lift

up_r = 307
down_r = 304
left_r = 305
right_r = 308

up_l = 544
down_l = 545
left_l = 546
right_l = 547

ps = 316
start = 315
select = 314

l_trig = 310
r_trig = 311
lb_trig = 312
rb_trig = 313

l_stick = 317
r_stick = 318

deadZoneMin = 115
deadZoneMax = 140

selectedDevice = 0
comPorts = []
arms = []
connectedPorts = []
formatVals = 1
gripperToggle = False


def printJoyValues(x):
    if(formatVals == 1):
        print("\r{}  \t{}  \t{}  \t{}  \t{}  \t{}  " .format(
            x[0], x[1], x[2], x[3], x[4], x[5]), end='')
        sys.stdout.flush()
        cursor.hide()
    else:
        print("\r{}  \t{}  \t{}  \t{}  \t{}  \t{}  " .format(
            abs(x[0] - 127) * 4, abs(x[1] - 127) * 4, abs(x[2] - 127) * 4, abs(x[3] - 127) * 4, x[4] * 2, x[5] * 2), end='')
        sys.stdout.flush()
        cursor.hide()


def printInterface(gamepad, arms, connectedPorts, selectedDevice):
    os.system("clear")
    print("\u001b[1m\u001b[34;4mManual Control Interface\033[0m")
    # print("Input Device: {}".format(gamepad.name))
    print("\033[1m\nID\tPort\033[0m")
    print("---------------------------------------------")
    for i in range(len(arms)):
        if i == selectedDevice:
            print(
                "\033[1m\033[92mArm_{}:\t{}\t <- Selected\033[0m".format(i, connectedPorts[int(i)]))
        else:
            print("Arm_{}:\t{}".format(i, connectedPorts[int(i)]))
    print("---------------------------------------------")
    print("\u001b[1mDevice:\u001b[0m Arm_{} (Sel)\t\u001b[1mBaudRate:\u001b[0m {:,}bps".format(
        selectedDevice, arms[selectedDevice].baudrate))
    print("\u001b[1mFormat:\u001b[0m {} (Strt)\t\u001b[1mGripper:\u001b[0m {} (RTrig)".format(
        'Literal' if formatVals else 'Mapped', 'Yes' if gripperToggle else 'No'))
    print("---------------------------------------------")
    print("\u001b[1mX:\tY:\tRX:\tRY:\tZ:\tRZ:\u001b[0m")
    print("\r0\t0\t0\t0\t0\t0  ", end='')
    sys.stdout.flush()
    cursor.hide()


def printComPorts(connectedPorts):
    comPorts.clear()
    os.system("clear")
    print("\u001b[1m\u001b[34;4mSelect COM Ports\n\033[0m")
    print("\u001b[1mID\tPort\t\tBaud\tStatus\033[0m")
    print("---------------------------------------------")

    for i in glob.glob('/dev/tty[A-Z]*'):
        if i != '/dev/ttyAMA0':
            comPorts.insert(0, i)
            for j in range(len(glob.glob('/dev/tty[A-Z]*')) - len(connectedPorts) - 1):
                connectedPorts.append(None)

    if len(comPorts) > 0:
        for i in range(len(comPorts)):
            displayed = False
            for j in range(len(comPorts)):
                if comPorts[i] == connectedPorts[j] and displayed == False:
                    displayed = True
                    for k in range(len(arms)):
                        if arms[k].name == connectedPorts[j]:
                            print("\033[92m{}:\t{}\t{}\tConnected\033[0m".format(
                                i, comPorts[i], arms[(k)].baudrate))
                elif j == len(comPorts) - 1 and displayed == False:
                    print("{}:\t{}\t".format(i, comPorts[i]))

    else:
        print("Error: No COM ports found")
        sys.exit()
    print("---------------------------------------------")


cursor.show()
flag = 0
while(1):
    printComPorts(connectedPorts)
    counter = 0
    for i in connectedPorts:
        if i != None:
            counter += 1
    if counter > 0:
        print("\033[92mEnter \"Y/y\" to confirm and continue\033[0m")
    else:
        if flag == 1:
            print("\033[91mError: Must select at least one device\033[0m")
            flag = 0
        else:
            print("Toggle connections on/off")
    print("---------------------------------------------")
    comSelection = input("Enter ID to connect: ")
    if(comSelection == "y" or comSelection == "Y"):
        if (len(arms) == 0):
            print("Error: No Arms Selected")
        else:
            os.system("clear")
            break
    elif comSelection == "":
        flag = 1
    else:
        try:
            comSelection = int(comSelection)
        except ValueError:
            flag = 1

        try:
            stored = False
            for i in range(len(connectedPorts)):
                if connectedPorts[i] == comPorts[int(comSelection)]:
                    stored = True

            for i in range(len(connectedPorts)):
                if connectedPorts[i] == None and stored == False:
                    baudrate = input("Enter Baudrate (default: 500,000): ")
                    try:
                        baudrate = int(baudrate)
                    except ValueError:
                        baudrate = 500000

                    connectedPorts[i] = comPorts[int(comSelection)]
                    arms.append(Arduino(name='{}'.format(comPorts[int(comSelection)]),
                                        port=connectedPorts[i], baud=baudrate))
                    stored = True
                elif connectedPorts[i] == comPorts[int(comSelection)] and stored == True:
                    connectedPorts[i] = None
                    for i in range(len(arms)):
                        if arms[i].name == comPorts[int(comSelection)]:
                            arms[i].disconnect()
                            arms.pop(i)
                            break
            comSelection = None

        except Exception as e:
            print(e)

while (1):
    print("\u001b[1m\u001b[34;4mSelect Input Device\033[0m")
    print("\u001b[1m\nID\tName\033[0m")
    print("---------------------------------------------")
    inputDevices = []
    for i in os.listdir('/dev/input/'):
        try:
            inputDevices.insert(0, InputDevice('/dev/input/' + i).name)
            pass
        except Exception:
            pass

    for i in range(len(inputDevices)):
        print("{}:\t{}".format(i, inputDevices[i]))
    print("---------------------------------------------")

    if (len(inputDevices) == 0):
        print("\033[91mError: No input devices found\033[0m")
        inputSelection = input("\nEnter to Refresh")
    else:
        inputSelection = input("Enter device ID: ")
    try:
        gamepad = InputDevice('/dev/input/event' + inputSelection)
        os.system("clear")
        break
    except:
        os.system("clear")
        pass

printInterface(gamepad, arms, connectedPorts, selectedDevice)

previousValue = [0, 0, 0, 0]
values = [127, 127, 127, 127, 0, 0]
try:
    for event in gamepad.read_loop():
        if event.type == ecodes.EV_KEY:
            if event.value == 1:
                if event.code == up_r:
                    arms[selectedDevice].write(0)
                elif event.code == down_r:
                    arms[selectedDevice].write(1)
                elif event.code == left_r:
                    arms[selectedDevice].write(2)
                elif event.code == right_r:
                    arms[selectedDevice].write(3)
                elif event.code == up_l:
                    arms[selectedDevice].write(4)
                elif event.code == down_l:
                    arms[selectedDevice].write(5)
                elif event.code == left_l:
                    arms[selectedDevice].write(7)
                elif event.code == right_l:
                    arms[selectedDevice].write(6)

                elif event.code == ps:
                    # gamepad.write(ecodes.EV_FF, effect_id1, 1)
                    arms[selectedDevice].write(20)
                    os.system("clear")
                    cursor.show()
                    print("exit")
                    exit()
                elif event.code == start:
                    # arms[selectedDevice].write(8)
                    formatVals = not formatVals
                    printInterface(
                        gamepad, arms, connectedPorts, selectedDevice)
                elif event.code == select:
                    if len(arms) > 1:
                        arms[selectedDevice].write(9)
                        if selectedDevice < len(arms) - 1:
                            selectedDevice += 1
                        else:
                            selectedDevice = 0
                        printInterface(
                            gamepad, arms, connectedPorts, selectedDevice)
                elif event.code == l_trig:
                    # gripperToggle = True
                    # printInterface(gamepad, arms, connectedPorts, selectedDevice)
                    arms[selectedDevice].write(10)
                elif event.code == r_trig:
                    gripperToggle = not gripperToggle
                    printInterface(
                        gamepad, arms, connectedPorts, selectedDevice)
                    arms[selectedDevice].write(11)
                elif event.code == lb_trig:
                    arms[selectedDevice].write(12)
                elif event.code == rb_trig:
                    arms[selectedDevice].write(13)
                elif event.code == l_stick:
                    arms[selectedDevice].write(14)
                elif event.code == r_stick:
                    arms[selectedDevice].write(15)

        if event.type == ecodes.EV_ABS:
            absevent = categorize(event)
            if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_X":

                values[0] = absevent.event.value
                if (values[0] >= deadZoneMax or values[0] < deadZoneMin):
                    if (abs(previousValue[0] - values[0]) > 2):
                        arms[selectedDevice].write("U")
                        arms[selectedDevice].write(values[0])
                        previousValue[0] = values[0]
                        printJoyValues(values)
                elif (previousValue[0] != 127):
                    arms[selectedDevice].write("U")
                    arms[selectedDevice].write(127)
                    previousValue[0] = 127
                    values[0] = 127
                    printJoyValues(values)
                pass
            elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Y":
                values[1] = absevent.event.value
                if (values[1] >= deadZoneMax or values[1] < deadZoneMin):
                    if (abs(previousValue[1] - values[1]) > 2):
                        arms[selectedDevice].write("V")
                        arms[selectedDevice].write(values[1])
                        previousValue[1] = values[1]
                        printJoyValues(values)
                elif (previousValue[1] != 127):
                    arms[selectedDevice].write("V")
                    arms[selectedDevice].write(127)
                    previousValue[1] = 127
                    values[1] = 127
                    printJoyValues(values)
                pass
            elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Z":
                values[4] = absevent.event.value
                if gripperToggle:
                    arms[selectedDevice].write("S")
                else:
                    arms[selectedDevice].write("Y")
                arms[selectedDevice].write(values[4])
                printJoyValues(values)
                pass
            elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RX":
                values[2] = absevent.event.value
                if (values[2] >= deadZoneMax or values[2] < deadZoneMin):
                    if (abs(previousValue[2] - values[2]) > 2):
                        arms[selectedDevice].write("X")
                        arms[selectedDevice].write(values[2])
                        previousValue[2] = values[2]
                        printJoyValues(values)
                elif (previousValue[2] != 127):
                    arms[selectedDevice].write("X")
                    arms[selectedDevice].write(127)
                    previousValue[2] = 127
                    values[2] = 127
                    printJoyValues(values)
                pass
            elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RY":
                values[3] = absevent.event.value
                if (values[3] >= deadZoneMax or values[3] < deadZoneMin):
                    if (abs(previousValue[3] - values[3]) > 2):
                        arms[selectedDevice].write("W")
                        arms[selectedDevice].write(values[3])
                        previousValue[3] = values[3]
                        printJoyValues(values)
                elif (previousValue[3] != 127):
                    arms[selectedDevice].write("W")
                    arms[selectedDevice].write(127)
                    previousValue[3] = 127
                    values[3] = 127
                    printJoyValues(values)
                pass

            elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RZ":
                values[5] = absevent.event.value
                if gripperToggle:
                    arms[selectedDevice].write("T")
                else:
                    arms[selectedDevice].write("Z")
                arms[selectedDevice].write(values[5])
                printJoyValues(values)
                pass
except Exception as e:
    arms[selectedDevice].write(20)
    cursor.show()
    print("end")
    print(e)
    pass
