import cv2
from fastapi import FastAPI, Response
from starlette.responses import StreamingResponse

app = FastAPI()

def generate_frames():
    camera = cv2.VideoCapture(0)  # เปิดกล้อง USB
    if not camera.isOpened():
        raise RuntimeError("ไม่สามารถเปิดกล้องได้")
    
    while True:
        success, frame = camera.read()
        if not success:
            break
        
        
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.get("/video_feed")
async def video_feed():
    return StreamingResponse(generate_frames(), media_type="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)