import cv2 as cv
import numpy as np
import sys
import matplotlib.pyplot as plt
import json
import os
import time
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from utils import triangulate_fisheye
from utils import read_camera_parameters, read_rotation_translation


# 指定下载好的模型路径
model_path = './hand_landmarker.task'

base_options = python.BaseOptions(model_asset_path=model_path)
options = vision.HandLandmarkerOptions(
    base_options=base_options,
    running_mode=vision.RunningMode.VIDEO,  # 根据输入源选择 VIDEO, IMAGE 或 LIVE_STREAM
    num_hands=2,                             # 追踪的最大手部数量
    min_hand_detection_confidence=0.1,      # 手部检测的最低置信度
    min_hand_presence_confidence=0.5,       # 手部存在判断的最低置信度
    min_tracking_confidence=0.3             # 手部追踪的最低置信度
)

# =========================
# Config
# =========================
DATA_DIR = "./build/data3D"
os.makedirs(DATA_DIR, exist_ok=True)
KEYPOINTS_FILE = os.path.join(DATA_DIR, "keypoints_3d.ndjson")
VIDEO_FILE = os.path.join(DATA_DIR, "output.mp4")

# Original stereo frame size
# frame_shape = [1200, 1600]
frame_shape = [480, 640]
# Reduced frame size for detection to speed up inference
detect_shape = (640, 480)
plot_interval = 1  # update 3D plot every N frames

# Hand skeleton for visualization
fingers = {
    "thumb": [0, 1, 2, 3, 4],
    "index": [0, 5, 6, 7, 8],
    "middle": [0, 9, 10, 11, 12],
    "ring": [0, 13, 14, 15, 16],
    "pinky": [0, 17, 18, 19, 20],
}
fingers_colors = {
    "thumb": "orange",
    "index": "black",
    "middle": "green",
    "ring": "blue",
    "pinky": "red",
}

# =========================
# Camera parameters
# =========================
def load_camera_params():
    K_left, D_left = read_camera_parameters(0)
    K_right, D_right = read_camera_parameters(1)
    R0, T0 = read_rotation_translation(0)
    R1, T1 = read_rotation_translation(1)
    R_rel = R0 @ R1.T
    T_rel = T1
    return K_left, D_left, K_right, D_right, R_rel, T_rel

# =========================
# Helpers
# =========================
def append_keypoints_frame(frame_id, kpts3d):
    entry = {"frame_id": frame_id, "keypoints3d": kpts3d.tolist()}
    with open(KEYPOINTS_FILE, "a") as f:
        f.write(json.dumps(entry) + "\n")

def update_3d_plot(kpts3d):
    ax.clear()
    ax.set_xlim(-30, 30)
    ax.set_ylim(-30, 30)
    ax.set_zlim(0, 100)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.view_init(elev=20, azim=-45)
    for name, ids in fingers.items():
        pts = [kpts3d[i] for i in ids if np.all(kpts3d[i] != -1)]
        if len(pts) == len(ids):
            pts = np.array(pts)
            ax.plot(
                pts[:, 0], pts[:, 1], pts[:, 2],
                color=fingers_colors[name], linewidth=2
            )
    plt.draw()
    plt.pause(0.001)

def extract_keypoints(result):
    """Extract first detected hand's 21 keypoints"""
    kpts = [[-1, -1]] * 21
    preds = result["predictions"][0]  # batch size 1
    if preds:
        hand = preds[0]
        if "keypoints" in hand:
            kpts = np.array(hand["keypoints"]).tolist()
    return kpts

def draw_keypoints(frame, keypoints, color=(0, 255, 0)):
    """Draw joints and skeleton"""
    for x, y in keypoints:
        if x > 0 and y > 0:
            cv.circle(frame, (int(x), int(y)), 3, color, -1)
    # Draw skeleton lines
    for name, ids in fingers.items():
        pts = [keypoints[i] for i in ids if keypoints[i][0] > 0]
        if len(pts) == len(ids):
            for i in range(len(pts)-1):
                cv.line(frame, tuple(map(int, pts[i])), tuple(map(int, pts[i+1])), color, 2)
    return frame

# =========================
# Main loop
# =========================
def run(video_path):
    with vision.HandLandmarker.create_from_options(options) as landmarker:
        K_left, D_left, K_right, D_right, R, T = load_camera_params()

        cap = cv.VideoCapture(video_path)
        if not cap.isOpened():
            raise RuntimeError(f"❌ Failed to open video file: {video_path}")

        fps = cap.get(cv.CAP_PROP_FPS)
        fps = fps if fps > 0 else 30

        out_video = cv.VideoWriter(
            VIDEO_FILE, cv.VideoWriter_fourcc(*"mp4v"),
            fps, (frame_shape[1]*2, frame_shape[0])
        )
        if not out_video.isOpened():
            raise RuntimeError("❌ VideoWriter failed to open.")

        plt.ion()
        global ax
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        frame_id = 0
        frame_timestamp_ms = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                print("✅ End of video reached.")
                break

            width = frame.shape[1] // 2
            frame0 = cv.resize(frame[:, :width], (frame_shape[1], frame_shape[0]))
            frame1 = cv.resize(frame[:, width:], (frame_shape[1], frame_shape[0]))

            mp_image_0 = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame0)
            mp_image_1 = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame1)

            # 执行手部关键点检测（时间戳必须单调递增）
            detection_result_0 = landmarker.detect_for_video(mp_image_0, frame_timestamp_ms)
            frame_timestamp_ms += 1  # 递增时间戳
            detection_result_1 = landmarker.detect_for_video(mp_image_1, frame_timestamp_ms)
            frame_timestamp_ms += 1  # 递增时间戳

            # 提取关键点坐标并转换为像素坐标
            def extract_keypoints_from_result(detection_result, frame_width, frame_height):
                kpts = [[-1, -1]] * 21
                if detection_result.hand_landmarks:
                    hand_landmarks = detection_result.hand_landmarks[0]
                    # 检查hand_landmarks是否有landmark属性，或者本身就是关键点列表
                    if hasattr(hand_landmarks, 'landmark'):
                        landmarks = hand_landmarks.landmark
                    else:
                        landmarks = hand_landmarks
                    for i, landmark in enumerate(landmarks):
                        if i >= 21:
                            break
                        # 检查landmark是否有x,y属性
                        if hasattr(landmark, 'x') and hasattr(landmark, 'y'):
                            kpts[i] = [landmark.x * frame_width, landmark.y * frame_height]
                        elif isinstance(landmark, (list, tuple)) and len(landmark) >= 2:
                            kpts[i] = [landmark[0], landmark[1]]
                return kpts

            kpts0 = extract_keypoints_from_result(detection_result_0, frame_shape[1], frame_shape[0])
            kpts1 = extract_keypoints_from_result(detection_result_1, frame_shape[1], frame_shape[0])

            # Triangulate
            kpts3d = []
            for uv0, uv1 in zip(kpts0, kpts1):
                if uv0[0]==-1 or uv1[0]==-1:
                    kpts3d.append(np.array([-1,-1,-1]))
                else:
                    kpts3d.append(triangulate_fisheye(uv0, uv1, K_left, D_left, K_right, D_right, R, T))
            kpts3d = np.array(kpts3d)
            # kpts3d = np.array(kpts3d) * 5.0  # 将3D点放大5倍
            # kpts3d = np.array(kpts3d) - 30.0

            append_keypoints_frame(frame_id, kpts3d)

            # Update plot every N frames
            if frame_id % plot_interval == 0:
                update_3d_plot(kpts3d)

            # Draw keypoints and skeleton
            vis0 = draw_keypoints(frame0.copy(), kpts0, color=(0,255,0))
            vis1 = draw_keypoints(frame1.copy(), kpts1, color=(0,0,255))
            combined_frame = np.hstack((vis0, vis1))
            out_video.write(combined_frame.astype(np.uint8))

            # Optional live view
            cv.imshow("Left Camera", vis0)
            cv.imshow("Right Camera", vis1)
            frame_id += 1
            if cv.waitKey(1) & 0xFF == 27:  # ESC
                break

            time.sleep(0.04)

        cap.release()
        out_video.release()
        cv.destroyAllWindows()
        plt.ioff()
        plt.close()

# =========================
# Run
# =========================
if __name__ == "__main__":
    video_path = "build/test/output.mp4"
    if len(sys.argv)>1:
        video_path = sys.argv[1]
    run(video_path)
