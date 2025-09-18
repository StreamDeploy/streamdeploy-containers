#!/usr/bin/env bash
set -euo pipefail

cat > /app/server.py << 'PY'
import os, io
from fastapi import FastAPI, UploadFile, File
from fastapi.responses import JSONResponse
from PIL import Image
import numpy as np, onnxruntime as ort

app = FastAPI()
model_path = os.getenv("MODEL_PATH","/models/model.onnx")
providers = ["CUDAExecutionProvider","CPUExecutionProvider"]
session = ort.InferenceSession(model_path, providers=providers)

@app.get("/health")
def health():
    return {"ok": True, "model": os.path.basename(model_path), "providers": session.get_providers()}

@app.post("/infer")
async def infer(file: UploadFile = File(...)):
    img_bytes = await file.read()
    img = Image.open(io.BytesIO(img_bytes)).convert("RGB").resize((640,640))
    x = np.asarray(img).astype(np.float32)/255.0
    x = np.transpose(x, (2,0,1))[None, ...]  # NCHW
    outputs = session.run(None, {session.get_inputs()[0].name: x})
    return JSONResponse({"outputs": [o.tolist() for o in outputs]})
PY

exec python3 -m uvicorn server:app --host "${SERVICE_HOST}" --port "${SERVICE_PORT}"
