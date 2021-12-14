import sys
import RPi.GPIO as GPIO
from gpiozero import Motor
import time
import cv2
import io
import numpy as np 
import picamera

#-----initialize------

#camera

#顔検出の学習済みモデルファイル
cascade_path = "./haarcascade_frontalface_default.xml"

camera= picamera.PiCamera()#カメラのインスタンス
camera.resolution = (256,256)#解像度
camera.hflip = True#左右反転
camera.vflip = True#上下反転

#カスケード分類器の特徴量を取得する
cascade = cv2.CascadeClassifier(cascade_path)

#認識を行う秒数
how_long = 5

#----- fan settings-----
duty =1 #ファンの速度(0~1)
#gpio端子の設定
#19,20を小型のモーターの方にモータードライバを使って接続する
motor = Motor(forward=19,backward=20)

#servo
GPIO.setmode(GPIO.BCM)

gp_out1 = 18
gp_out2 = 27
GPIO.setup(gp_out1, GPIO.OUT)
GPIO.setup(gp_out2, GPIO.OUT)
try:
    servo1 = GPIO.PWM(gp_out1, 50)
except:
    print("servo1 error")
try:
    servo2 = GPIO.PWM(gp_out2, 50)
except:
    print("servo2 error")
    
STOP_TIMES = 10

#-----end initialize-----

#main、プログラム終了時に必ず実行される処理はfinallyに記述
def main():
    try:
        Robot()
    finally:
        GPIO.cleanup()
        cv2.destroyAllWindows()

#n秒間(how_long)カメラを起動し顔を認識する関数
def recognizeFace():
    end_time=time.time()+how_long#現在の時刻+how_long[秒]
    while(time.time()<end_time):
        stream = io.BytesIO()#バイナリデータ
        camera.capture(stream, format="jpeg")
        camera_data = np.frombuffer(stream.getvalue(),dtype = np.uint8)
        stream.seek(0)
        image = cv2.imdecode(camera_data, cv2.IMREAD_COLOR)
        #グレースケール変換
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        #検知
        facerect = cascade.detectMultiScale(image_gray, scaleFactor=1.1, minNeighbors=2, minSize=(30, 30))

        #print(facerect)
        color = (255, 255, 255) #白

        # 検出した場合
        if len(facerect)>0:
            #ファンを回す処理
            fan()
            break
        
            #検出した顔を四角形で囲む(最終的にはいらない)
            for rect in facerect:
                cv2.rectangle(image, tuple(rect[0:2]),tuple(rect[0:2]+rect[2:4]), color, thickness=2)
            
        cv2.imshow("capture",image)
        if cv2.waitKey(10) & 0xFF == ord(" "):
            break

    cv2.destroyAllWindows()

#ファンを回転
def fan():

    motor.forward()
    time.sleep(5)
    motor.stop()

#カメラ、ファンの向きを回転させるプログラムの基幹
def Robot():
    servo1.start(0.0)
    servo1.ChangeDutyCycle(2.5)
    servo2.start(0.0)
    servo2.ChangeDutyCycle(2.5)
    time.sleep(1.0)
    while(True):
        #n段階(STOP_TIMES)に分けて回転しては止まる処理
        #進む方向(-90~90)
        for deg in range(STOP_TIMES):
            duty = 2.5+((12.0-2.5)/(STOP_TIMES-1))*deg

            servo1.ChangeDutyCycle(duty)
            servo2.ChangeDutyCycle(duty)
            time.sleep(0.5)
            recognizeFace()
            time.sleep(0.5)
        #戻る方向(90~-90)
        for deg in range(STOP_TIMES):
            duty = 12.0-((12.0-2.5)/(STOP_TIMES-1))*deg

            servo1.ChangeDutyCycle(duty)
            servo2.ChangeDutyCycle(duty)
            time.sleep(0.5)
            recognizeFace()
            time.sleep(0.5)

if __name__ == "__main__":
    sys.exit(main())