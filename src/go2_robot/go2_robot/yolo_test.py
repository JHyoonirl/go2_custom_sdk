from ultralytics import YOLO
from pathlib import Path
import torch
import pyrealsense2 as rs
import numpy as np
import cv2

current_file_path = Path(__file__)
current_dir = current_file_path.parent
model_path = current_dir.parent / "yolo_models" / "yolov8s.pt"

# RealSense 파이프라인을 생성합니다.
pipeline = rs.pipeline()

# 설정을 생성하고 컬러 스트림만 활성화합니다.
config = rs.config()
# 해상도: 1280x720, 포맷: BGR8 (OpenCV와 호환), 프레임: 30fps
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

print("웹캠을 시작합니다...")
# 설정대로 스트리밍을 시작합니다.
pipeline.start(config)
print(torch.cuda.is_available())

if __name__ == "__main__":
    try:
        model = YOLO(model_path)  # yolov8n.pt, yolov8s.pt, yolov8m.pt, yolov8l.pt, yolov8x.pt
        
        while True:
            # 다음 프레임이 올 때까지 기다립니다.
            frames = pipeline.wait_for_frames()
            # 컬러 프레임만 가져옵니다.
            color_frame = frames.get_color_frame()
            
            # 프레임이 없는 경우 다음 루프로 넘어갑니다.
            if not color_frame:
                continue
                
            # RealSense 이미지 데이터를 OpenCV에서 사용할 수 있는 Numpy 배열로 변환합니다.
            color_image = np.asanyarray(color_frame.get_data())
            
            # ===============================================================
            # 1. YOLO 모델로 객체 탐지 수행 (추론)
            results = model(color_image, verbose=False) # verbose=False로 설정하여 터미널 출력을 깔끔하게 합니다.

            # 2. 결과에서 바운딩 박스와 라벨이 그려진 이미지 가져오기
            annotated_frame = results[0].plot()
            
            # 3. 원본 이미지가 아닌, 탐지 결과가 그려진 이미지를 화면에 보여주기
            cv2.imshow('YOLOv8 RealSense', annotated_frame)
            # ===============================================================
            
            # 'q' 키를 1밀리초 동안 기다리고, 눌렸다면 루프를 탈출합니다.
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # 모든 작업이 끝나면 스트리밍을 중지합니다.
        print("웹캠을 종료합니다...")
        pipeline.stop()
        # 모든 OpenCV 창을 닫습니다.
        cv2.destroyAllWindows()
    