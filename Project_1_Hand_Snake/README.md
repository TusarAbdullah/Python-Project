# Hand-Tracking Snake Game (Python + OpenCV)

A Snake Game controlled by your **index finger** using real-time **hand tracking**.  
Eat the food (apple), grow longer, and avoid hitting your own body.

## Features
- Real-time hand tracking control (index finger tip)
- Smooth movement (reduced jitter)
- Transparent PNG food overlay
- Score counter
- Restart: **R**
- Exit: **Esc**

## Tech Stack
- Python
- OpenCV
- CVZone
- MediaPipe
- NumPy

## How to Run
```bash
pip install -r requirements.txt
python main.py
```

## Controls
- Move your index finger to control the snake
- Press R to restart
- Press Esc to exit

## Files
- main.py — main game code
- apple.png — food image (transparent PNG)

## Notes
- If the camera does not open, try changing this line in main.py:
  ```bash
  cap = cv2.VideoCapture(0)
  ```
  to 1 or 2.
- Food image should be RGBA PNG (transparent background) for overlayPNG.

