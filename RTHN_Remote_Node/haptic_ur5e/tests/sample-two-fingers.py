from time import sleep
from bhaptics import better_haptic_player as player
from bhaptics.better_haptic_player import BhapticsPosition
import keyboard

player.initialize()

# tact file can be exported from bhaptics designer
print("register CenterX")
player.register("CenterX", "CenterX.tact")
print("register Circle")
player.register("Circle", "Circle.tact")

interval = 0.1
durationMillis = 100

for i in range(20):
    player.submit_dot("gloveRFrame", BhapticsPosition.GloveR.value, [{"index": 0, "intensity": 100}, {"index": 1, "intensity": 100}], durationMillis)
    sleep(interval)

#all_actuators = [{"index": 0, "intensity": 100},{"index": 1, "intensity": 100},{"index": 2, "intensity": 100},{"index": 3, "intensity": 100},{"index": 4, "intensity": 100},{"index": 5, "intensity": 100}]
#player.submit_dot("gloveRFrame", BhapticsPosition.GloveR.value, all_actuators, durationMillis)

player.destroy()