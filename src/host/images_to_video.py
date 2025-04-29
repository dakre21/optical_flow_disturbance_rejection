import os
import cv2

if __name__ == "__main__":
    images = [
        img for img in os.listdir("build") if img.endswith((".png", ".jpg", ".jpeg"))
    ]

    first_image = cv2.imread(os.path.join("build", images[0]))
    height, width, _ = first_image.shape

    video = cv2.VideoWriter('oflow.avi', 0, 1, (width, height))

