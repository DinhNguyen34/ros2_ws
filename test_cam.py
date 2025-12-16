import cv2

def main():
	cap = cv2.VideoCapture(0)

	if not cap.isOpened():
		print("Khong mo duoc camera 0")
		return

	while True:
		ret, frame = cap.read()
		if not ret:
			print("khong doc dc frame")
			break

		cv2.imshow("cam", frame)

		if cv2.waitKey(1) & 0xFF == 27:
			break

	cap.release()
	cv2.destroyAllWindows()

if __name__ == "__main__":
	main()
