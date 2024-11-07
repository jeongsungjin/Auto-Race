import rospy
import torch
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from model2 import *
from slidewindow_sungjin_ver import SlideWindow
from driving_es import *

class LanenetDetection:
    def __init__(self):
        rospy.init_node('lanenet_detection_node')

        # 메시지 퍼블리셔와 브리지 설정
        self.lane_pub = rospy.Publisher('lane_result', Image, queue_size=1)
        self.bridge = CvBridge()
        
        # 모델 로드 및 기타 변수 초기화
        self.image = None
        self.slidewindow = SlideWindow()
        self.x_location = 280
        self.last_x_location = 280
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model_path = '/Users/seongjinjeong/Auto-Race-/src/lanenet/lanenet_.model'
        self.LaneNet_model = Lanenet(2, 4)
        self.LaneNet_model.load_state_dict(torch.load(self.model_path, map_location=torch.device(self.device)))
        self.LaneNet_model.to(self.device)
        
        rospy.Subscriber('/usb_cam/image_rect_color', Image, self.camCB)

        self.rate = rospy.Rate(30)

    def camCB(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def inference(self, gt_img_org):
        # BGR 순서
        org_shape = gt_img_org.shape
        gt_image = cv2.resize(gt_img_org, dsize=(512, 256), interpolation=cv2.INTER_LINEAR)
        gt_image = gt_image / 127.5 - 1.0
        gt_image = torch.tensor(gt_image, dtype=torch.float)
        gt_image = np.transpose(gt_image, (2, 0, 1))
        gt_image = gt_image.to(self.device)
        # lane segmentation
        binary_final_logits, instance_embedding = self.LaneNet_model(gt_image.unsqueeze(0))
        binary_final_logits, instance_embedding = binary_final_logits.to('cpu'), instance_embedding.to('cpu')
        binary_img = torch.argmax(binary_final_logits, dim=1).squeeze().numpy()
        binary_img[0:65,:] = 0 #(0~85행)을 무시 - 불필요한 영역 제거      
        binary_img=binary_img.astype(np.uint8)
        binary_img[binary_img>0]=255
        # 차선 클러스터링, 색상 지정
        # rbg_emb, cluster_result = process_instance_embedding(instance_embedding, binary_img,distance=1.5, lane_num=2)
        # rbg_emb = cv2.resize(rbg_emb, dsize=(org_shape[1], org_shape[0]), interpolation=cv2.INTER_LINEAR)
        # a = 0.1
        # frame = a * gt_img_org[..., ::-1] / 255 + rbg_emb * (1 - a)
        # frame = np.rint(frame * 255)
        # frame = frame.astype(np.uint8)

        frame = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
        frame = cv2.resize(frame, dsize=(560, 480))

        return frame
    
    def run(self):
        while not rospy.is_shutdown():
            try:
                # pid = PID(0.2, 0.05, 0.01)
                cv2.imshow("lane_image", self.image)

                show_img = self.inference(self.image)
                # init_show_img(show_img)

                cv2.imshow("show", show_img)


                img_frame = show_img.copy() # img_frame변수에 카메라 이미지를 받아옵니다.  
                height,width,channel = img_frame.shape # 이미지의 높이,너비,채널값을 변수에 할당합니다.
               
                img_roi = img_frame[280:470,0:]   # y좌표 0~320 사이에는 차선과 관련없는 이미지들이 존재하기에 노이즈를 줄이기 위하여 roi설정을 해주었습니다.

                img_filtered = color_filter(img_roi)   #roi가 설정된 이미지를 color_filtering 하여 흰색 픽셀만을 추출해냅니다.

                height, width, channel = img_filtered.shape


                cv2.imshow("filter", img_filtered)
               

                # 카메라 중간 위치
                left_margin = 128
                top_margin =  81
                src_point1 = [0, top_margin]      # 왼쪽 아래
                src_point2 = [left_margin, 0]      #195,44
                src_point3 = [width - left_margin, 0]    #445,44
                src_point4 = [width , top_margin]  #640, 190

                src_points = np.float32([src_point1, src_point2, src_point3, src_point4])
               
                dst_point1 = [width//4, 190]    # 왼쪽 아래
                dst_point2 = [width//4, 0]      # 왼쪽 위
                dst_point3 = [width//4*3, 0]    # 오른쪽 위
                dst_point4 = [width//4*3, 190]  # 오른쪽 아래

                dst_points = np.float32([dst_point1, dst_point2, dst_point3, dst_point4])
               
                matrix = cv2.getPerspectiveTransform(src_points, dst_points)
                img_warped = cv2.warpPerspective(img_filtered, matrix, [width, height])

                img_warped = cv2.resize(img_warped,dsize=(640,480))

                _, L, _ = cv2.split(cv2.cvtColor(img_warped, cv2.COLOR_BGR2HLS))
                _, img_binary = cv2.threshold(L, 0, 255, cv2.THRESH_BINARY) 

                img_masked = region_of_interest(img_binary) 

                out_img, self.x_location, current_lane = self.slidewindow.slidewindow(img_masked)
                if self.x_location is None:
                    self.x_location = self.last_x_location
                else:
                    self.last_x_location = self.x_location
               
                img_masked_colored = cv2.cvtColor(img_masked, cv2.COLOR_GRAY2BGR)
           
                if out_img.shape == img_masked_colored.shape:
                    img_blended = cv2.addWeighted(out_img, 1, img_masked_colored, 0.6, 0)  # sliding window 결과를 시각화하기 위해 out_img와 시점 변환된 이미지를 merging 하였습니다.
                    cv2.imshow('img_blended', img_blended)
                    # 최종 이미지를 ROS 토픽에 발행
                    lane_msg = self.bridge.cv2_to_imgmsg(img_blended, "bgr8")
                    self.lane_pub.publish(lane_msg)
                else:
                    print(f"Shape mismatch: out_img {out_img.shape}, img_masked {img_masked_colored.shape}")

            except Exception as e:
                print(e)
           
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = LanenetDetection()
        node.run()
    except rospy.ROSInterruptException:
        pass
