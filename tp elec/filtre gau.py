import cv2
import numpy as np
from urllib.request import urlopen

# Charger l'image
req = urlopen("http://www.vgies.com/downloads/robocup.png")
arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
img = cv2.imdecode(arr, -1)

# 1️⃣ Floutage Gaussien
img_blur = cv2.GaussianBlur(img, (9, 9), 0)
cv2.imshow("Gaussian Blur 3x3", img_blur)

img_blur_large = cv2.GaussianBlur(img, (7,7), 0)
cv2.imshow("Gaussian Blur 7x7", img_blur_large)
cv2.waitKey(0)

# 2️⃣ Sobel automatique
sobelx = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5)
sobely = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=5)
sobelxy = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5)

cv2.imshow("Sobel X", sobelx)
cv2.waitKey(0)
cv2.imshow("Sobel Y", sobely)
cv2.waitKey(0)
cv2.imshow("Sobel XY", sobelxy)
cv2.waitKey(0)

# 3️⃣ Sobel manuel (gradient horizontal supérieur)
kernel_sup = np.array([[-1,-2,-1],
                       [0,0,0],
                       [1,2,1]])
imgSobelHSup = cv2.filter2D(src=img_blur, ddepth=-1, kernel=kernel_sup)
cv2.imshow("Sobel Horizontal Supérieur", imgSobelHSup)
cv2.waitKey(0)

# Gradient horizontal inférieur
kernel_inf = np.array([[1,2,1],
                       [0,0,0],
                       [-1,-2,-1]])
imgSobelHInf = cv2.filter2D(src=img_blur, ddepth=-1, kernel=kernel_inf)

# Gradient vertical gauche
kernel_gauche = np.array([[-1,0,1],
                          [-2,0,2],
                          [-1,0,1]])
imgSobelVG = cv2.filter2D(src=img_blur, ddepth=-1, kernel=kernel_gauche)

# Gradient vertical droit
kernel_droit = np.array([[1,0,-1],
                         [2,0,-2],
                         [1,0,-1]])
imgSobelVD = cv2.filter2D(src=img_blur, ddepth=-1, kernel=kernel_droit)

# Fusion des contours
imgContours = cv2.addWeighted(imgSobelHSup, 0.25, imgSobelHInf, 0.25, 0)
imgContours = cv2.addWeighted(imgContours, 1, imgSobelVG, 0.25, 0)
imgContours = cv2.addWeighted(imgContours, 1, imgSobelVD, 0.25, 0)
cv2.imshow("Contours fusionnés", imgContours)
cv2.waitKey(0)

# 4️⃣ Autres filtres

# Accentuation (sharpen)
kernel_sharpen = np.array([[0,-1,0],
                           [-1,5,-1],
                           [0,-1,0]])
img_sharpen = cv2.filter2D(src=img, ddepth=-1, kernel=kernel_sharpen)
cv2.imshow("Sharpen", img_sharpen)
cv2.waitKey(0)

# Lissage (blur moyen)
kernel_blur = np.ones((5,5),np.float32)/25
img_blur2 = cv2.filter2D(src=img, ddepth=-1, kernel=kernel_blur)
cv2.imshow("Blur", img_blur2)
cv2.waitKey(0)

# Outline
kernel_outline = np.array([[-1,-1,-1],
                           [-1,8,-1],
                           [-1,-1,-1]])
img_outline = cv2.filter2D(src=img, ddepth=-1, kernel=kernel_outline)
cv2.imshow("Outline", img_outline)
cv2.waitKey(0)

cv2.destroyAllWindows()
