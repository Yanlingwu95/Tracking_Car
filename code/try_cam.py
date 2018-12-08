import cv2
img = cv2.imread("a.jpg")
cv2.namedWindow("Image")
cv2.imshow("Image", img)
cv2.waitKey (0)
cv2.destroyAllWindows()  


