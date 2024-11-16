import cv2
import cv2.aruco as aruco

# 이미지 경로 설정
image_path = "image.png"  # 이미지 경로를 올바르게 지정하세요
image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# 테스트할 딕셔너리 목록
aruco_dicts = {
    "DICT_4X4_50": aruco.Dictionary_get(aruco.DICT_4X4_50),
    "DICT_4X4_100": aruco.Dictionary_get(aruco.DICT_4X4_100),
    "DICT_4X4_250": aruco.Dictionary_get(aruco.DICT_4X4_250),
    "DICT_4X4_1000": aruco.Dictionary_get(aruco.DICT_4X4_1000),
    "DICT_5X5_50": aruco.Dictionary_get(aruco.DICT_5X5_50),
    "DICT_5X5_100": aruco.Dictionary_get(aruco.DICT_5X5_100),
    "DICT_5X5_250": aruco.Dictionary_get(aruco.DICT_5X5_250),
    "DICT_5X5_1000": aruco.Dictionary_get(aruco.DICT_5X5_1000),
    "DICT_6X6_50": aruco.Dictionary_get(aruco.DICT_6X6_50),
    "DICT_6X6_100": aruco.Dictionary_get(aruco.DICT_6X6_100),
    "DICT_6X6_250": aruco.Dictionary_get(aruco.DICT_6X6_250),
    "DICT_6X6_1000": aruco.Dictionary_get(aruco.DICT_6X6_1000),
    "DICT_7X7_50": aruco.Dictionary_get(aruco.DICT_7X7_50),
    "DICT_7X7_100": aruco.Dictionary_get(aruco.DICT_7X7_100),
    "DICT_7X7_250": aruco.Dictionary_get(aruco.DICT_7X7_250),
    "DICT_7X7_1000": aruco.Dictionary_get(aruco.DICT_7X7_1000)
}

# 딕셔너리마다 마커 검출
for dict_name, aruco_dict in aruco_dicts.items():
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(image, aruco_dict, parameters=parameters)
    if ids is not None and len(ids) > 0:
        print(f"Detected markers using {dict_name}: IDs = {ids.flatten().tolist()}")
