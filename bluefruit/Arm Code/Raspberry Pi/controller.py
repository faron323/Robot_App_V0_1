from evdev import InputDevice, categorize, ecodes, ff
import serial
from connect import Arduino

arm = Arduino(port=0)

# if event.value == 1:  # Press
#     pass
# else:  # Lift
#     pass

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
    gamepad = InputDevice('/dev/input/event2')
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
                    arm.write("a")
                elif event.code == down_r:
                    arm.write("b")
                elif event.code == left_r:
                    arm.write("c")
                elif event.code == right_r:
                    arm.write("d")
                elif event.code == up_l:
                    arm.write("e")
                elif event.code == down_l:
                    arm.write("f")
                elif event.code == left_l:
                    arm.write("g")
                elif event.code == right_l:
                    arm.write("h")

                elif event.code == ps:
                    gamepad.write(ecodes.EV_FF, effect_id1, 1)
                elif event.code == start:
                    arm.write("i")
                elif event.code == select:
                    arm.write("j")

                elif event.code == l_trig:
                    arm.write("k")
                elif event.code == r_trig:
                    arm.write("l")
                elif event.code == lb_trig:
                    arm.write("m")
                elif event.code == rb_trig:
                    arm.write("n")
                elif event.code == l_stick:
                    arm.write("o")
                elif event.code == r_stick:
                    arm.write("p")

        if event.type == ecodes.EV_ABS:
            absevent = categorize(event)
            if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_X":
                if absevent.event.value > up_bound or absevent.event.value < low_bound:
                    arm.write("x")
                    arm.write(str(absevent.event.value))
                pass
            elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Y":
                if absevent.event.value > up_bound or absevent.event.value < low_bound:
                    arm.write("y")
                    arm.write(str(absevent.event.value))
                pass
            elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Z":
                if absevent.event.value > up_bound or absevent.event.value < low_bound:
                    arm.write("z")
                    arm.write(str(absevent.event.value))
                pass
            elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RX":
                if absevent.event.value > up_bound or absevent.event.value < low_bound:
                    arm.write("u")
                    arm.write(str(absevent.event.value))
                pass
            elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RY":
                if absevent.event.value > up_bound or absevent.event.value < low_bound:
                    arm.write("v")
                    arm.write(str(absevent.event.value))
                pass

            elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RZ":
                if absevent.event.value > up_bound or absevent.event.value < low_bound:
                    arm.write("w")
                    arm.write(str(absevent.event.value))
                pass

except Exception as e:
    print(e)
    # print("Controller Not Connected")
    pass
