import cv2



def take_picture():

	frames_ignored = 60
	 
	# Now we can initialize the camera capture object with the cv2.VideoCapture class.
	# All it needs is the index to a camera port.

	camera = cv2.VideoCapture(0)
	 
	# Captures a single image from the camera and returns it in PIL format
	def get_image():
	 # read is the easiest way to get a full image out of a VideoCapture object.
	 retval, im = camera.read()
	 return im
	 
	# Ramp the camera - these frames will be discarded and are only used to allow v4l2
	# to adjust light levels, if necessary

	for i in range(frames_ignored):
	 temp = get_image()

	camera_capture = get_image()
	# A nice feature of the imwrite method is that it will automatically choose the
	# correct format based on the file extension you provide. Convenient!
	return cv2.imwrite("ball_image.png", camera_capture)
	 
	# You'll want to release the camera, otherwise you won't be able to create a new
	# capture object until your script exits
	camera.release()





	

if __name__ == '__main__':
	take_picture()