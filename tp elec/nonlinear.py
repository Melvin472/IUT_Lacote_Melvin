import cv2
import numpy as np
from urllib.request import urlopen

# Charger l'image
req = urlopen("http://www.vgies.com/downloads/robocup.png")
arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
img = cv2.imdecode(arr, -1)

# 1️⃣ Floutage Gaussien
img_blur = cv2.GaussianBlur(img, (9, 9), 0)
cv2.imshow("Gaussian Blur 9x9", img_blur)

img_blur_large = cv2.GaussianBlur(img, (7, 7), 0)
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

# ------------------------------------------------------------
# 6.1 FILTRAGE NON LINAIRE : Filtre médian
# ------------------------------------------------------------
median3 = cv2.medianBlur(img, 3)
median5 = cv2.medianBlur(img, 5)
median7 = cv2.medianBlur(img, 7)

cv2.imshow("Median 3x3", median3)
cv2.waitKey(0)
cv2.imshow("Median 5x5", median5)
cv2.waitKey(0)
cv2.imshow("Median 7x7", median7)
cv2.waitKey(0)

# ------------------------------------------------------------
# 6.2 FILTRAGE BILATERAL
# ------------------------------------------------------------
bilat_default = cv2.bilateralFilter(img, d=9, sigmaColor=75, sigmaSpace=75)
bilat_soft = cv2.bilateralFilter(img, d=9, sigmaColor=50, sigmaSpace=50)
bilat_strong = cv2.bilateralFilter(img, d=9, sigmaColor=150, sigmaSpace=150)

cv2.imshow("Bilateral (75,75)", bilat_default)
cv2.waitKey(0)
cv2.imshow("Bilateral doux (50,50)", bilat_soft)
cv2.waitKey(0)
cv2.imshow("Bilateral fort (150,150)", bilat_strong)
cv2.waitKey(0)

# ------------------------------------------------------------
# 6.3 DETECTION DE CONTOURS : Canny
# ------------------------------------------------------------

# On applique un petit flou avant (conseillé)
img_gauss_for_canny = cv2.GaussianBlur(img, (5,5), 0)

canny_default = cv2.Canny(img_gauss_for_canny, 100, 200)
cv2.imshow("Canny 100-200", canny_default)
cv2.waitKey(0)

# Variation seuils
canny_low = cv2.Canny(img_gauss_for_canny, 50, 100)
canny_high = cv2.Canny(img_gauss_for_canny, 150, 250)

cv2.imshow("Canny 50-100 (plus sensible)", canny_low)
cv2.waitKey(0)

cv2.imshow("Canny 150-250 (moins sensible)", canny_high)
cv2.waitKey(0)

# ------------------------------------------------------------
# Sobel manuel
# ------------------------------------------------------------

# Gradient horizontal supérieur
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

# ------------------------------------------------------------
# Autres filtres
# ------------------------------------------------------------

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

# 7️⃣ Opérations morpho-mathématiques : Érosion et Dilatation
kernel = np.ones((5,5), np.uint8)

img_erosion = cv2.erode(img, kernel, iterations=1)
img_dilation = cv2.dilate(img, kernel, iterations=1)

cv2.imshow("Input", img)
cv2.imshow("Erosion", img_erosion)
cv2.imshow("Dilation", img_dilation)
cv2.waitKey(0)
