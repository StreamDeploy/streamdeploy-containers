#!/usr/bin/env bash
set -euo pipefail
cat > /app/server.py << 'PY'
import os, io
from fastapi import FastAPI, UploadFile, File
from fastapi.responses import JSONResponse
from PIL import Image
import numpy as np
from openvino.runtime import Core

core = Core()
model_xml = os.getenv("MODEL_XML","/models/model.xml")
device = os.getenv("OV_DEVICE","AUTO")
model = core.read_model(model=model_xml)
compiled = core.compile_model(model=model, device_name=device)
input_name = compiled.input(0).get_any_name()

app = FastAPI()
@app.get("/health")
def health():
    return {"ok": True, "device": device, "model": os.path.basename(model_xml)}

@app.post("/infer")
async def infer(file: UploadFile = File(...)):
    img = Image.open(io.BytesIO(await file.read())).convert("RGB").resize((640,640))
    x = np.asarray(img).transpose(2,0,1)[None].astype(np.float32)/255.0
    res = compiled([x])
    return JSONResponse({"outputs":[r.tolist() for r in res]})
PY
exec python3 -m uvicorn server:app --host "${SERVICE_HOST}" --port "${SERVICE_PORT}"
