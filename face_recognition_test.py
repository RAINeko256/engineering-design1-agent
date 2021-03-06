import cv2
import io
import numpy as np 
import picamera
import time
from gpiozero import Robot

#顔検出の学習済みモデルファイル
cascade_path = "./haarcascade_frontalface_default.xml"

stream = io.BytesIO()#バイナリデータ
camera= picamera.PiCamera()#カメラのインスタンス
camera.resolution = (256,256)#解像度
camera.hflip = True#左右反転
camera.vflip = True#上下反転

end_time=time.time()+10#現在の時刻+10[秒]
while(time.time()<end_time):
    camera.capture(stream, format="jpeg")
    data = np.frombuffer(stream.getvalue(),dtype = np.uint8)
    stream.seek(0)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    #グレースケール変換
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    #カスケード分類器の特徴量を取得する
    cascade = cv2.CascadeClassifier(cascade_path)

    #検知
    facerect = cascade.detectMultiScale(image_gray, scaleFactor=1.1, minNeighbors=2, minSize=(30, 30))

    #print(facerect)
    color = (255, 255, 255) #白

    # 検出した場合
    if len(facerect)>0:
        
        #検出した顔を四角形で囲む(最終的にはいらない)
        for rect in facerect:
            cv2.rectangle(image, tuple(rect[0:2]),tuple(rect[0:2]+rect[2:4]), color, thickness=2)
    
    cv2.imshow("capture",image)
    if cv2.waitKey(10) & 0xFF == ord(" "):
        break

cv2.destroyAllWindows()