from evdev import InputDevice, categorize, ecodes, ff
import serial
from connect import Arduino
import os

# for i in os.listdir('/dev/'):
# print(i)
arm = Arduino(name='Container_Arm_0', port='/dev/ttyUSB0', baud=115200)

# For all Buttons, 1 = Press, 0 = Lift

# Controller Constants
# Shape Buttons
up_r = 307
down_r = 304
left_r = 305
right_r = 308
# D Pad
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

up_bound = 150
low_bound = 110

try:
    for i in os.listdir('/dev/input/'):
        try:
            gamepad = InputDevice('/dev/input/' + i)
            break
        except Exception:
            pass

    print(gamepad)

    for event in gamepad.read_loop():
        if event.type == ecodes.EV_KEY:
            if event.value == 1:
                if event.code == up_r:
                    arm.write(0)
                elif event.code == down_r:
                    arm.write(1)
                elif event.code == left_r:
                    arm.write(2)
                elif event.code == right_r:
                    arm.write(3)
                elif event.code == up_l:
                    arm.write(4)
                elif event.code == down_l:
                    arm.write(5)
                elif event.code == left_l:
                    arm.write(7)
                elif event.code == right_l:
                    arm.write(6)
                elif event.code == ps:
                    arm.write(20)
                    exit()
                    pass
                elif event.code == start:
                    arm.write(8)
                elif event.code == select:
                    arm.write(9)
                elif event.code == l_trig:
                    arm.write(10)
                elif event.code == r_trig:
                    arm.write(11)
                elif event.code == lb_trig:
                    arm.write(12)
                elif event.code == rb_trig:
                    arm.write(13)
                elif event.code == l_stick:
                    arm.write(14)
                elif event.code == r_stick:
                    arm.write(15)

        if event.type == ecodes.EV_ABS:
            absevent = categorize(event)
            if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_X":
                arm.write("U")
                arm.write(absevent.event.value)
                print(absevent.event.value)
                pass
            elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Y":
                arm.write("V")
                arm.write(absevent.event.value)
                pass
            elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Z":
                arm.write("Y")
                arm.write(absevent.event.value)
                pass
            elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RX":
                arm.write("X")
                arm.write(absevent.event.value)
                pass
            elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RY":
                arm.write("W")
                arm.write(absevent.event.value)
                pass

            elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RZ":
                arm.write("Z")
                arm.write(absevent.event.value)
                pass

# If control loop exits unexpectedly send b'20 to arm to stop loop.
except Exception as e:
    arm.write(20)
    print(e)
    pass
