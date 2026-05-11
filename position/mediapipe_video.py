# # Install dependencies
# # pip install mediapipe opencv-python
# import cv2
# import mediapipe as mp
# import argparse
# # Initialize MediaPipe Hands and Drawing utilities
# mp_hands = mp.solutions.hands
# mp_drawing = mp.solutions.drawing_utils

# # Parse command line arguments
# parser = argparse.ArgumentParser(description='Hand tracking with MediaPipe')
# parser.add_argument('--video', type=str, default='input.mp4', help='Path to input video file')
# args = parser.parse_args()

# # Open video file
# cap = cv2.VideoCapture(args.video)
# with mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
#    while cap.isOpened():
#        ret, frame = cap.read()
#        if not ret:
#            break
#        # Convert BGR to RGB for MediaPipe processing
#        # image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#        image = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
#        image.flags.writable = False
#        results = hands.process(image)
#        # Convert back to BGR for OpenCV display
#        image.flags.writable = True
#        #image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
#        # Draw hand landmarks if detected
#        if results.multi_hand_landmarks:
#            for hand_landmarks in results.multi_hand_landmarks:
#                mp_drawing.draw_landmarks(
#                    image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
#        # Display the output
#        cv2.imshow('Hand Tracking', image)
#        if cv2.waitKey(10) & 0xFF == ord('q'):
#            break
# cap.release()
# cv2.destroyAllWindows()


import cv2
import mediapipe as mp
import argparse
import time

# 1. 导入新版API的核心模块
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# 2. 指定下载好的模型路径
model_path = './hand_landmarker.task'
# 模型下载地址: https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
# 或搜索 "MediaPipe Hand Landmarker Task File"

parser = argparse.ArgumentParser(description='Hand tracking with MediaPipe')
parser.add_argument('--video', type=str, default='input.mp4', help='Path to input video file')
args = parser.parse_args()

# 3. 设置运行模式为视频流 (VIDEO)
base_options = python.BaseOptions(model_asset_path=model_path)
options = vision.HandLandmarkerOptions(
    base_options=base_options,
    running_mode=vision.RunningMode.VIDEO,  # 根据输入源选择 VIDEO, IMAGE 或 LIVE_STREAM
    num_hands=2,                             # 追踪的最大手部数量
    min_hand_detection_confidence=0.5,      # 手部检测的最低置信度
    min_hand_presence_confidence=0.5,       # 手部存在判断的最低置信度
    min_tracking_confidence=0.5             # 手部追踪的最低置信度
)

# 4. 初始化检测器 (使用 with 语句自动管理资源)
with vision.HandLandmarker.create_from_options(options) as landmarker:
    # 5. 打开摄像头并循环处理每一帧
    cap = cv2.VideoCapture(args.video)
    frame_timestamp_ms = 0

    while cap.isOpened():
        
        success, frame = cap.read()
        if not success:
            break

        # 将帧转化为 MediaPipe 可用的 Image 对象
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

        # 执行手部关键点检测
        detection_result = landmarker.detect_for_video(mp_image, frame_timestamp_ms)
        frame_timestamp_ms += 1

        # 处理检测结果...
        if detection_result.hand_landmarks:
            for hand_landmarks in detection_result.hand_landmarks:
                # 获取图像尺寸
                h, w, _ = frame.shape
                
                # 定义手部关键点连接关系
                connections = [
                    (0, 1), (1, 2), (2, 3), (3, 4),  # 拇指
                    (0, 5), (5, 6), (6, 7), (7, 8),  # 食指
                    (0, 9), (9, 10), (10, 11), (11, 12),  # 中指
                    (0, 13), (13, 14), (14, 15), (15, 16),  # 无名指
                    (0, 17), (17, 18), (18, 19), (19, 20)   # 小指
                ]
                
                # 存储关键点坐标
                points = []
                for landmark in hand_landmarks:
                    x = int(landmark.x * w)
                    y = int(landmark.y * h)
                    points.append((x, y))
                    # 绘制关键点
                    cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
                
                # 绘制连接线
                for connection in connections:
                    start_idx, end_idx = connection
                    cv2.line(frame, points[start_idx], points[end_idx], (0, 255, 0), 2)

        cv2.imshow('Hand Tracking', frame)
        time.sleep(0.04)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()