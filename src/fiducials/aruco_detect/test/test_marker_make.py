import cv2
import cv2.aruco as aruco

# 7x7 딕셔너리 선택
aruco_dict = aruco.Dictionary_get(aruco.DICT_7X7_250)

# 검출 파라미터 생성
parameters = aruco.DetectorParameters_create()

# 카메라에서 이미지 읽기
cap = cv2.VideoCapture(0)  # 카메라 ID 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 아루코 마커 검출
    corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    # 검출된 마커 그리기
    if ids is not None:
        frame = aruco.drawDetectedMarkers(frame, corners, ids)

    # 결과 출력
    cv2.imshow("7x7 Marker Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
