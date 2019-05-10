from evdev import InputDevice, categorize, ecodes, ff
import serial

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

start = 315
select = 314

l_trig = 310
r_trig = 311
lb_trig = 312
rb_trig = 313

l_stick = 317
r_stick = 318

try:
    gamepad = InputDevice('/dev/input/event1')
    print(gamepad)

    rumble = ff.Rumble(strong_magnitude=0x1fff, weak_magnitude=0xffff)
    duration_ms = 150
    effect = ff.Effect(
        ecodes.FF_RUMBLE, -1, 0,
        ff.Trigger(up_l, 0),
        ff.Replay(duration_ms, 150),
        ff.EffectType(ff_rumble_effect=rumble)
    )
    # duration_ms=150
    # effect=ff.Effect(
    #     ecodes.FF_RUMBLE, -1, 0,
    #     ff.Trigger(up_l, 0),
    #     ff.Replay(duration_ms, 0),
    #     ff.EffectType(ff_rumble_effect=rumble)
    # )
    effect_id1 = gamepad.upload_effect(effect)

    for event in gamepad.read_loop():
        if event.type == ecodes.EV_KEY:
            if event.value == 1:
                if event.code == up_r:
                    print("triangle")
                elif event.code == down_r:
                    print("x")
                elif event.code == left_r:
                    print("square")
                elif event.code == right_r:
                    print("circle")

                elif event.code == up_l:
                    print("up")
                elif event.code == down_l:
                    print("down")
                elif event.code == left_l:
                    print("left")
                elif event.code == right_l:
                    print("right")

                elif event.code == start:
                    gamepad.write(ecodes.EV_FF, effect_id1, 1)
                    print("start")
                elif event.code == select:
                    print("select")

                elif event.code == l_trig:
                    print("left bumper")
                elif event.code == r_trig:
                    print("right bumper")
                elif event.code == lb_trig:
                    print("left back bumper")
                elif event.code == rb_trig:
                    print("right back bumper")
                elif event.code == l_stick:
                    print("left stick in")
                elif event.code == r_stick:
                    print("right stick in")

        if event.type == ecodes.EV_ABS:
            absevent = categorize(event)
            print(absevent.event.value)
            # print(ecodes.bytype[absevent.event.type][absevent.event.code])
            if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Y":
                print(absevent.event.value)
                pass
            elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_X":
                # if absevent.event.value < 110:
                print(absevent.event.value)
                pass
            elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RY":
                pass
            elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RX":
                pass
            else:
                print(ecodes.bytype[absevent.event.type][absevent.event.code])
                pass
            # elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Z":
            #     print("HERE")
            #     pass
            # elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RZ":
            #     print("HERE")
            #     pass

except Exception as e:
    pass
print("Controller Not Connected")


def arduino_read(*args, **kwargs):
    ser = serial.Serial('/dev/ttyACM0', 9600)
    input = ser.read()
    print(input)
