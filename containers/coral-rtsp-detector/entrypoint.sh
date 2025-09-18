#!/usr/bin/env bash
set -euo pipefail

cat > /app/detect_and_rtsp.py << 'PY'
import os, time, threading, cv2, numpy as np
from PIL import Image
import tflite_runtime.interpreter as tflite

MODEL = os.getenv("MODEL_PATH")
LABELS = os.getenv("LABELS_PATH")
DEV = os.getenv("CAMERA_DEV","/dev/video0")
FPS = int(os.getenv("FPS","30"))
W = int(os.getenv("WIDTH","1280"))
H = int(os.getenv("HEIGHT","720"))

labels = {}
with open(LABELS, 'r') as f:
    for line in f:
        idx, name = line.strip().split(maxsplit=1)
        labels[int(idx)] = name

# Try to use EdgeTPU delegate; fallback to CPU
delegates=[]
try:
    delegates=[tflite.load_delegate('libedgetpu.so.1')]
except Exception:
    pass

interpreter = tflite.Interpreter(model_path=MODEL, experimental_delegates=delegates)
interpreter.allocate_tensors()
inp = interpreter.get_input_details()[0]
out = interpreter.get_output_details()

cap = cv2.VideoCapture(DEV)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
cap.set(cv2.CAP_PROP_FPS, FPS)

def infer_frame(frame):
    img = cv2.resize(frame, (300,300))
    x = np.expand_dims(img, 0)
    interpreter.set_tensor(inp['index'], x)
    interpreter.invoke()
    boxes = interpreter.get_tensor(out[0]['index'])[0]
    classes = interpreter.get_tensor(out[1]['index'])[0].astype(int)
    scores = interpreter.get_tensor(out[2]['index'])[0]
    count = int(interpreter.get_tensor(out[3]['index'])[0])
    return boxes, classes, scores, count

# Simple overlay + stdout metrics; RTSP served by gst pipeline launched below
while True:
    ok, frame = cap.read()
    if not ok:
        time.sleep(0.01)
        continue
    boxes, classes, scores, cnt = infer_frame(frame)
    for i in range(cnt):
        if scores[i] < 0.5: continue
        y1, x1, y2, x2 = boxes[i]
        (x1, y1, x2, y2) = (int(x1*W), int(y1*H), int(x2*W), int(y2*H))
        cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
        name = labels.get(classes[i], "obj")
        cv2.putText(frame, f"{name}:{scores[i]:.2f}", (x1, max(0,y1-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
    cv2.imshow("annotated", frame)  # no-op in headless; kept for local test
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
PY

# Launch GStreamer RTSP in background (nv enc not required; CPU-safe)
python3 /app/detect_and_rtsp.py &

# Minimal RTSP using gst-rtsp-server via gst-launch equivalent (simplified)
# For production swap with a proper rtsp-server process or mediamtx
exec gst-launch-1.0 \
  v4l2src device=${CAMERA_DEV} ! video/x-raw,framerate=${FPS}/1,width=${WIDTH},height=${HEIGHT} \
  ! x264enc tune=zerolatency bitrate=3000 speed-preset=ultrafast \
  ! rtph264pay config-interval=1 pt=96 \
  ! udpsink host=127.0.0.1 port=5000
