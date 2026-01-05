from pybricks.hubs import EV3Brick
from pybricks.parameters import Color, Port, Stop
from pybricks.ev3devices import Motor
from pybricks.tools import wait, StopWatch
from micropython import const
from umath import sin, pi

hub = EV3Brick()
m = Motor(Port.A)

# These variables are used to compute a duty cycle that resists the user's
# attempt to turn the motor in a direction with a low duty cycle, while still
# allowing us to tell when the user stops turning the motor in that direction.
OSCILLATE_PERIOD = 22  # ms
STATIC_FRICTION_DUTY = 20
T_COEFF = 2 * pi / OSCILLATE_PERIOD
RESIST_MAX_DUTY = 30


def resist_duty(base_duty, t) -> int:
    added_duty = sin(t * T_COEFF) * STATIC_FRICTION_DUTY
    return int(base_duty + added_duty)


w = StopWatch()
lw = StopWatch()


LOG_PERIOD = const(100)


def maybe_log(x):
    if lw.time() > LOG_PERIOD:
        print(x)


while True:
    t = w.time()

    angle = m.angle()
    speed = m.speed()

    maybe_log(f"Angle: {angle}, Speed: {speed}")

    if abs(angle) < 10:
        hub.light.on(Color.BLUE)
        maybe_log("Near zero angle")
        m.brake()
        if abs(angle) < 1:
            m.stop()
    elif (angle > 0) != (speed > 0) and speed != 0:
        # If the motor is currently turned in one direction and turning back
        # toward zero, encourage that motion by running the motor toward zero.
        maybe_log("Turning toward zero")
        m.dc(-30 if angle > 0 else 30)
    else:
        # If the user is currently turning the motor away from zero, resist
        # that movement with a duty cycle that increases the further we get
        # from zero. Duty cycle of resistance maxes out at 15%.
        base_duty = 0
        if abs(angle) > 50:
            base_duty = min(RESIST_MAX_DUTY, RESIST_MAX_DUTY * (abs(angle) - 50) / 130)
        duty = resist_duty(base_duty, t)
        duty = duty if angle < 0 else -duty
        maybe_log(f"Resisting with base_duty: {base_duty} duty: {duty}")
        m.dc(duty)

        if abs(duty) < 8:
            hub.light.on(Color.ORANGE)
        else:
            hub.light.on(Color.RED)

    if lw.time() > LOG_PERIOD:
        lw.reset()
    wait(10)