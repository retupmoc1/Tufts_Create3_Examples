'''
comments to be inserted
'''
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

robot = Create3(Bluetooth())

speed = 10.0

@event(robot.when_bumped, [True, False])
async def bumped(robot):
    await robot.set_lights_rgb(255, 0, 0)
    await robot.set_wheel_speeds(-speed, speed)
    await robot.wait(0.3)


@event(robot.when_bumped, [False, True])
async def bumped(robot):
    await robot.set_lights_rgb(0, 255, 0)
    await robot.set_wheel_speeds(speed, -speed)
    await robot.wait(0.3)

    
async def forward(robot):
    await robot.set_lights(Robot.LIGHT_SPIN, Color(255, 255, 0))
    await robot.set_wheel_speeds(speed, speed)


@event(robot.when_bumped, [])
async def move(robot):
    # This function will not be called again, since it never finishes.
    # Only task that are not currenctly running can be triggered.
    while True:
        await forward(robot)
        # No need of calling "await hand_over()" in this infinite loop, because robot methods are all called with await.
        await robot.wait(0.3)



robot.play()
