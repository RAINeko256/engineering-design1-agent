import time
from gpiozero import Robot

#gpio端子の設定
#17,18は使わない
#19,20を小型のモーターの方にモータードライバを使って接続する
robot = Robot(left=(17,18),right=(19,20))

duty=1
robot.forward()
time.sleep(5)
robot.stop()

print("hoge")
time.sleep(2)

robot.forward()
time.sleep(2)
robot.stop()
