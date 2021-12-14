import cv2
import io
import numpy as np 
import picamera

#顔検出の学習済みモデルファイル
cascade_path = "./haarcascade_frontalface_default.xml"

stream = io.BytesIO()#バイナリデータs
camera= picamera.PiCamera()#カメラのインスタンス
camera.resolution = (256,256)#解像度
camera.hflip = True#左右反転
camera.vflip = True#上下反転

while(True):
    camera.capture(stream, format="jpeg")
    data = np.frombuffer(stream.getvalue(),dtype = np.uint8)
    stream.seek(0)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    #グレースケール変換
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    #カスケード分類器の特徴量を取得する
    cascade = cv2.CascadeClassifier(cascade_path)

    #物体認識（顔認識）の実行
    #image – CV_8U 型の行列．ここに格納されている画像中から物体が検出されます
    #objects – 矩形を要素とするベクトル．それぞれの矩形は，検出した物体を含みます
    #scaleFactor – 各画像スケールにおける縮小量を表します
    #minNeighbors – 物体候補となる矩形は，最低でもこの数だけの近傍矩形を含む必要があります
    #flags – このパラメータは，新しいカスケードでは利用されません．古いカスケードに対しては，cvHaarDetectObjects 関数の場合と同じ意味を持ちます
    #minSize – 物体が取り得る最小サイズ．これよりも小さい物体は無視されます
    facerect = cascade.detectMultiScale(image_gray, scaleFactor=1.1, minNeighbors=2, minSize=(30, 30))

    #print(facerect)
    color = (255, 255, 255) #白

    # 検出した場合
    if len(facerect) > 0:

        
        #検出した顔を四角形で囲む(最終的にはいらない)
        for rect in facerect:
            cv2.rectangle(image, tuple(rect[0:2]),tuple(rect[0:2]+rect[2:4]), color, thickness=2)
    
    cv2.imshow("capture",image)
    if cv2.waitKey(10) & 0xFF == ord(" "):
        break

cv2.destroyAllWindows()