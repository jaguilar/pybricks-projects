from pybricks.hubs import EV3Brick
from pybricks.parameters import Color, Port, Stop, Direction
from pybricks.ev3devices import Motor
from pybricks.tools import wait, StopWatch, run_task
from micropython import const
from umath import sin, pi

hub = EV3Brick()
axis1_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
axis2_motor = Motor(Port.D)
axis2_motor.stop()

# These variables are used to compute a duty cycle that resists the user's
# attempt to turn the motor in a direction with a low duty cycle, while still
# allowing us to tell when the user stops turning the motor in that direction.

OSCILLATE_PERIOD = const(22)  # ms
AXIS1_STATIC_FRICTION_DUTY = const(20)
AXIS2_STATIC_FRICTION_DUTY = const(10)
PI = const(3.141592653589793)
T_COEFF = 2 * PI / OSCILLATE_PERIOD
RESIST_MAX_DUTY = const(30)
STEERING_DEAD_ZONE = const(10)  # degrees
RESIST_START_ANGLE = const(50)  # degrees - angle where resistance begins
RESIST_RANGE = const(130)  # degrees - range over which resistance ramps to max
AXIS2_MAX_THROW = const(29)  # degrees - maximum throw for throttle

def resist_duty(base_duty, t, static_friction_duty) -> int:
    added_duty = sin(t * T_COEFF) * static_friction_duty
    return int(base_duty + added_duty)


def steering_wheel(t: int) -> int:
    """Drives the steering wheel process.

    1. Provides force-feedback for the steering wheel depending on how
       far it is from neutral.
    2. Returns an integer between -127 and 127 that expresses how far
       the steering wheel is from neutral.

    Args:
        t (int): The time in milliseconds since the process started.
    Returns:
        int: An integer between -127 and 127 representing the steering
                wheel position.
    """
    angle = axis1_motor.angle()
    speed = axis1_motor.speed()

    if abs(angle) < STEERING_DEAD_ZONE:
        hub.light.on(Color.BLUE)
        axis1_motor.brake()
        if abs(angle) < 1:
            axis1_motor.stop()
    elif (angle > 0) != (speed > 0) and speed != 0:
        # If the motor is currently turned in one direction and turning back
        # toward zero, encourage that motion by running the motor toward zero.
        axis1_motor.dc(-30 if angle > 0 else 30)
    else:
        # If the user is currently turning the motor away from zero, resist
        # that movement with a duty cycle that increases the further we get
        # from zero. Duty cycle of resistance maxes out at 15%.
        base_duty = 0
        if abs(angle) > RESIST_START_ANGLE:
            base_duty = min(RESIST_MAX_DUTY, RESIST_MAX_DUTY * (abs(angle) - RESIST_START_ANGLE) / RESIST_RANGE)
        duty = resist_duty(base_duty, t, AXIS1_STATIC_FRICTION_DUTY)
        duty = duty if angle < 0 else -duty
        axis1_motor.dc(duty)
    
    # Scale the return value: 0 in dead zone, ±127 at max resistance angle
    if abs(angle) < STEERING_DEAD_ZONE:
        return 0
    
    max_angle = RESIST_START_ANGLE + RESIST_RANGE
    scaled = int(127 * (abs(angle) - STEERING_DEAD_ZONE) / (max_angle - STEERING_DEAD_ZONE))
    scaled = min(127, scaled)  # Cap at 127
    return scaled if angle > 0 else -scaled


def throttle(t: int) -> int:
    """Returns an integer between 0 and 128 representing the throttle position."""
    angle = axis2_motor.angle()
    
    # This extra duty we apply helps keep the motor unstuck, which makes
    # it easier for the user to push it.
    axis2_motor.dc(resist_duty(0, t, AXIS2_STATIC_FRICTION_DUTY))
    
    scaled = int(127 * angle / AXIS2_MAX_THROW)
    return min(127, max(0, scaled))


UPDATE_PERIOD = const(16)  # ms

async def main():
    """Main remote control process."""
    stopwatch = StopWatch()
    update = StopWatch()
    while True:
        t = stopwatch.time()
        wheel = steering_wheel(t)
        throttle_value = throttle(t)
        print(f"AXIS1:{wheel} AXIS2:{throttle_value}")

        await wait(update.time() - UPDATE_PERIOD)
        update.reset()


run_task(main())