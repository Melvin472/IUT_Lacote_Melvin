import cv2
import numpy as np
from matplotlib import pyplot as plt
from urllib.request import urlopen

req = urlopen("http://www.vgies.com/downloads/robocup.png")
arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
img = cv2.imdecode(arr, -1)

imageGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cv2.imshow('Grayscale', imageGray)

hist, bins = np.histogram(imageGray.flatten(), 256, [0,256])
cdf = hist.cumsum()
cdf_normalized = cdf * float(hist.max()) / cdf.max()
plt.plot(cdf_normalized, color='b')
plt.hist(imageGray.flatten(), 256, [0,256], color='r')
plt.xlim([0,256])
plt.legend(('cdf','histogram'), loc='upper left')
plt.show()

imgEqu = cv2.equalizeHist(imageGray)
cv2.imshow('Image egalisee', imgEqu)

histEq, binsEq = np.histogram(imgEqu.flatten(), 256, [0,256])
cdfEq = histEq.cumsum()
cdfEq_normalized = cdfEq * float(histEq.max()) / cdfEq.max()
plt.plot(cdfEq_normalized, color='b')
plt.hist(imgEqu.flatten(), 256, [0,256], color='r')
plt.xlim([0,256])
plt.legend(('cdfEq','histogramEq'), loc='upper left')
plt.show()

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
H, S, V = cv2.split(hsv)
Heq = cv2.equalizeHist(H)
Seq = cv2.equalizeHist(S)
Veq = cv2.equalizeHist(V)
hsv_eq = cv2.merge([Heq, Seq, Veq])
img_hsv_eq = cv2.cvtColor(hsv_eq, cv2.COLOR_HSV2BGR)
cv2.imshow('Image HSV egalisee', img_hsv_eq)

cv2.waitKey(0)
cv2.destroyAllWindows()
