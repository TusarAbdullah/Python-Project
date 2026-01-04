import math
import random
import time
import cvzone
import cv2
import numpy as np
from cvzone.HandTrackingModule import HandDetector

cap = cv2.VideoCapture(0)
cap.set(3, 1280)
cap.set(4, 720)

detector = HandDetector(detectionCon=0.8, maxHands=1)


class SnakeGameClass:
    def __init__(self, pathFood):
        # Snake data
        self.points = []
        self.lengths = []
        self.currentLength = 0
        self.allowedLength = 150
        self.previousHead = (0, 0)

        # Game state
        self.score = 0
        self.gameOver = False

        # Hand smoothing
        self.smoothFactor = 0.22
        self.smoothX, self.smoothY = 0, 0

        # Collision cooldown after eating
        self.lastEatTime = 0
        self.collisionCooldown = 1.0  # একটু বেশি দিলাম

        # Movement clamp (reduce sudden jump -> false collision)
        self.maxStep = 45  # pixels per frame (try 35-60)

        # Food image (must be RGBA)
        self.imgFood = cv2.imread(pathFood, cv2.IMREAD_UNCHANGED)
        if self.imgFood is None:
            raise FileNotFoundError(f"Food image not found: {pathFood}")
        if len(self.imgFood.shape) != 3 or self.imgFood.shape[2] != 4:
            raise ValueError("Food image must be PNG with transparency (RGBA 4 channels).")

        # OPTIONAL: resize donut
        scale = 0.10
        self.imgFood = cv2.resize(self.imgFood, (0, 0), fx=scale, fy=scale)
        self.hFood, self.wFood = self.imgFood.shape[:2]
        self.foodPoint = (0, 0)
        self.randomFoodLocation()

    def reset(self):
        self.points = []
        self.lengths = []
        self.currentLength = 0
        self.allowedLength = 150
        self.previousHead = (0, 0)

        self.score = 0
        self.gameOver = False

        self.smoothX, self.smoothY = 0, 0
        self.lastEatTime = 0

        self.randomFoodLocation()

    def randomFoodLocation(self):
        x = random.randint(self.wFood // 2, 1280 - self.wFood // 2)
        y = random.randint(self.hFood // 2, 720 - self.hFood // 2)
        self.foodPoint = (x, y)

    def update(self, imgMain, currentHead):
        if self.gameOver:
            cvzone.putTextRect(imgMain, "Game Over", [300, 350],
                               scale=7, thickness=5, offset=20)
            cvzone.putTextRect(imgMain, f"Your Score: {self.score}", [300, 500],
                               scale=5, thickness=5, offset=20)
            cvzone.putTextRect(imgMain, "Press R to Restart", [320, 620],
                               scale=3, thickness=3, offset=10)
            return imgMain

        cx, cy = currentHead

        # -------- smoothing --------
        if self.smoothX == 0 and self.smoothY == 0:
            self.smoothX, self.smoothY = cx, cy

        self.smoothX = int(self.smoothX + (cx - self.smoothX) * self.smoothFactor)
        self.smoothY = int(self.smoothY + (cy - self.smoothY) * self.smoothFactor)
        cx, cy = self.smoothX, self.smoothY

        # -------- clamp big jumps (anti jitter) --------
        px, py = self.previousHead
        step = math.hypot(cx - px, cy - py)
        if step > self.maxStep and step > 0:
            ratio = self.maxStep / step
            cx = int(px + (cx - px) * ratio)
            cy = int(py + (cy - py) * ratio)

        # Update snake points
        self.points.append([cx, cy])

        distance = math.hypot(cx - px, cy - py)
        self.lengths.append(distance)
        self.currentLength += distance
        self.previousHead = (cx, cy)

        # Length reduction
        while self.currentLength > self.allowedLength and self.lengths:
            self.currentLength -= self.lengths[0]
            self.lengths.pop(0)
            self.points.pop(0)

        # Food eating check
        rx, ry = self.foodPoint
        if (rx - self.wFood // 2 < cx < rx + self.wFood // 2) and \
           (ry - self.hFood // 2 < cy < ry + self.hFood // 2):
            self.randomFoodLocation()
            self.allowedLength += 50
            self.score += 1
            self.lastEatTime = time.time()
            print("Score:", self.score)

        # -------- Draw snake body --------
        if len(self.points) > 1:
            for i in range(1, len(self.points)):
                ratio = i / len(self.points)
                green = int(80 + 120 * ratio)
                cv2.line(imgMain, self.points[i - 1], self.points[i], (0, green, 0), 18)

        # SAFETY 
        if not self.points:
            return imgMain

        # -------- Draw snake head (ellipse + eyes + tongue) --------
        hx, hy = self.points[-1]

        if len(self.points) > 2:
            x1, y1 = self.points[-2]
            dx, dy = hx - x1, hy - y1
        else:
            dx, dy = 1, 0

        angle = int(np.degrees(np.arctan2(dy, dx)))

        cv2.ellipse(imgMain, (hx, hy), (26, 18), angle, 0, 360, (0, 200, 0), cv2.FILLED)
        cv2.ellipse(imgMain, (hx, hy), (26, 18), angle, 0, 360, (0, 90, 0), 2)

        eye_offset = 10
        cv2.circle(imgMain, (hx - eye_offset, hy - 6), 3, (255, 255, 255), cv2.FILLED)
        cv2.circle(imgMain, (hx + eye_offset, hy - 6), 3, (255, 255, 255), cv2.FILLED)
        cv2.circle(imgMain, (hx - eye_offset, hy - 6), 1, (0, 0, 0), cv2.FILLED)
        cv2.circle(imgMain, (hx + eye_offset, hy - 6), 1, (0, 0, 0), cv2.FILLED)

        # Tongue
        tongue_len = 18
        norm = math.hypot(dx, dy) + 1e-6
        tx = int(hx + (dx / norm) * tongue_len)
        ty = int(hy + (dy / norm) * tongue_len)
        cv2.line(imgMain, (hx, hy), (tx, ty), (0, 0, 255), 2)
        cv2.line(imgMain, (tx, ty), (tx - 4, ty - 4), (0, 0, 255), 2)
        cv2.line(imgMain, (tx, ty), (tx - 4, ty + 4), (0, 0, 255), 2)

        # Draw food
        imgMain = cvzone.overlayPNG(
            imgMain, self.imgFood,
            (rx - self.wFood // 2, ry - self.hFood // 2)
        )

        # Score
        cvzone.putTextRect(imgMain, f"Score: {self.score}", [50, 80],
                           scale=3, thickness=3, offset=10)

        # -------- Collision check (much less sensitive) --------
        ignore_last = 18           # আগে 12 ছিল, এখন বেশি
        min_points_for_collision = 35  # ছোট snake হলে collision check না

        if len(self.points) > min_points_for_collision and len(self.points) > ignore_last + 10:
            if time.time() - self.lastEatTime > self.collisionCooldown:
                pts = np.array(self.points[:-ignore_last], np.int32)
                pts = pts.reshape((-1, 1, 2))
                # (optional debug)
                # cv2.polylines(imgMain, [pts], False, (0, 255, 0), 2)

                minDist = cv2.pointPolygonTest(pts, (cx, cy), True)

                # threshold আরও forgiving
                if minDist >= -35:   # try -40 if still false hit
                    print("Hit")
                    self.gameOver = True

        return imgMain


game = SnakeGameClass("apple.png")

while True:
    success, img = cap.read()
    if not success:
        print("Camera not found / cannot read frame")
        break

    img = cv2.flip(img, 1)
    hands, img = detector.findHands(img, flipType=False)

    if hands:
        lmList = hands[0]["lmList"]
        pointIndex = lmList[8][0:2]
        img = game.update(img, pointIndex)

    cv2.imshow("Image", img)
    key = cv2.waitKey(1)

    if key == ord("r"):
        game.reset()
    elif key == 27:
        break

cap.release()
cv2.destroyAllWindows()






