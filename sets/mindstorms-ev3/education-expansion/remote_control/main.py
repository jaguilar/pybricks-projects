from pybricks.hubs import ThisHub
from pybricks.parameters import Color, Port, Stop
from pybricks.ev3devices import Motor
from pybricks.tools import wait, StopWatch
from micropython import const
from umath import sin, pi

hub = ThisHub()
m = Motor(Port.A)

# These variables are used to compute a duty cycle that resists the user's
# attempt to turn the motor in a direction with a low duty cycle, while still
# allowing us to tell when the user stops turning the motor in that direction.
OSCILLATE_PERIOD = const(22)  # ms
STATIC_FRICTION_DUTY = const(20)
T_COEFF = 2 * pi / OSCILLATE_PERIOD
RESIST_MAX_DUTY = const(30)
DEAD_ZONE = const(10)


def resist_duty(base_duty, t) -> int:
    added_duty = sin(t * T_COEFF) * STATIC_FRICTION_DUTY
    return int(base_duty + added_duty)


w = StopWatch()
lw = StopWatch()


LOG_PERIOD = const(100)


def maybe_log(x):
    if lw.time() > LOG_PERIOD:
        print(x)


prev_duty = 0

while True:
    t = w.time()

    angle = m.angle()
    absangle = abs(angle)
    speed = m.speed()
    load = m.load()

    maybe_log(f"Angle: {angle}, Speed: {speed}, Load: {load}, Prev Duty: {prev_duty}")

    if absangle < DEAD_ZONE:
        hub.light.on(Color.BLUE)
        maybe_log("Near zero angle")
        if abs(speed) > 100:
            m.brake()
        else:
            m.stop()
    elif (angle > 0) != (speed > 0) and speed != 0:
        # If the motor is currently turned in one direction and turning back
        # toward zero, encourage that motion by running the motor toward zero.
        maybe_log("Turning toward zero")
        m.dc(40 if angle < 0 else -40)
        hub.light.on(Color.GREEN)
    else:
        # If the user is currently turning the motor away from zero, resist
        # that movement with a duty cycle that increases the further we get
        # from zero. Duty cycle of resistance maxes out at 15%.
        resistance_fraction = 0
        if absangle > 50:
            resistance_fraction = min(1, (absangle - 50) / 130)
        duty = resist_duty(RESIST_MAX_DUTY * resistance_fraction, t)
        duty = duty if angle < 0 else -duty
        maybe_log(f"Resisting with base_duty: {resistance_fraction} duty: {duty}")
        m.dc(duty)
        prev_duty = duty

        if resistance_fraction < 0.5:
            hub.light.on(Color.ORANGE)
        else:
            hub.light.on(Color.RED)

    if lw.time() > LOG_PERIOD:
        lw.reset()
    wait(1)
