import numpy as np
import cv2
import math


def __gstreamer_pipeline(
        camera_id,
        capture_width=3264,
        capture_height=2464,
        display_width=1920,
        display_height=1080,
        framerate=21,
        flip_method=0,
    ):
    return (
            "nvarguscamerasrc sensor-id=%d ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=True"
            % (
                    camera_id,
                    capture_width,
                    capture_height,
                    framerate,
                    flip_method,
                    display_width,
                    display_height,
            )
    )

def stereo_vision(spread, center_right, center_left, fov, image_width):
    fov = fov * 1.0
    image_width = image_width * 1.0      # converting fov and image_width to float
    dpp = fov / image_width              # calculating degrees per pixel (dpp)
    
    angle_left = (90 - fov/2) + center_left * dpp
    # print("Left angle: ", angle_left)
    angle_right = (90 - fov/2) + center_right * dpp
    # print("Right angle: ", angle_right)
    angle_center = 180 - angle_left - angle_right
    
    if angle_center <= 0:
        print("Object is too far!")
        return float('NaN')
    # print("Central angle: ", angle_center)
    
    angle_left = math.radians(angle_left)
    angle_right = math.radians(angle_right)
    angle_center = math.radians(angle_center)
    
    distance_left = spread * math.sin(angle_right) / math.sin(angle_center)
    distance_right = spread * math.sin(angle_left) / math.sin(angle_center)
    
    distance_average = (distance_left + distance_right) / 2
    
    return distance_average


# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

cv2.startWindowThread()

left_cam = cv2.VideoCapture(__gstreamer_pipeline(camera_id=1, flip_method=2), cv2.CAP_GSTREAMER)
right_cam = cv2.VideoCapture(__gstreamer_pipeline(camera_id=0, flip_method=2), cv2.CAP_GSTREAMER)

# the output will be written to output.avi
out = cv2.VideoWriter(
    'output.avi',
    cv2.VideoWriter_fourcc(*'MJPG'),
    15.,
    (640,480))

while(True):
    # Capture frame-by-frame
    left_ret, left_frame = left_cam.read()
    right_ret, right_frame = right_cam.read()
    right_frame = cv2.flip(right_frame, 1)
        # resizing for faster detection

    left_frame = cv2.resize(left_frame, (640, 480), cv2.INTER_AREA)
    right_frame = cv2.resize(right_frame, (640, 480), cv2.INTER_AREA)
    # using a greyscale picture, also for faster detection
    left_gray = cv2.cvtColor(left_frame, cv2.COLOR_RGB2GRAY)
    right_gray = cv2.cvtColor(right_frame, cv2.COLOR_RGB2GRAY)

    # detect people in the image
    # returns the bounding boxes for the detected objects
    left_boxes, left_weights = hog.detectMultiScale(left_frame, winStride=(8,8) )
    right_boxes, right_weights = hog.detectMultiScale(right_frame, winStride=(8,8) )

    if(len(left_boxes) > 0 and len(right_boxes) > 0):
        center_left = left_boxes[0][0] + (left_boxes[0][2] / 2)
        center_right = right_boxes[0][0] + (right_boxes[0][2] / 2)

        print("Center right:", center_right)
        # print("Center left:", center_left)

        distance = stereo_vision(spread = 0.8636, 
                                center_right = center_right,
                                center_left = center_left,
                                fov = 62.2,
                                image_width = len(left_frame[0]))
        cv2.putText(right_frame, "Distance: {:.3f}".format(distance), (right_boxes[0][0], right_boxes[0][1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
        print("Person found! Distance:", distance)

    output_boxes_right = np.array([[x, y, x + w, y + h] for (x, y, w, h) in right_boxes])

    for (xA, yA, xB, yB) in output_boxes_right:
        # display the detected boxes in the colour picture
        cv2.rectangle(right_frame, (xA, yA), (xB, yB),
                          (0, 255, 0), 2)
    
    output_boxes_left = np.array([[x, y, x + w, y + h] for (x, y, w, h ) in left_boxes])

    for (xA, yA, xB, yB) in output_boxes_left:
        cv2.rectangle(left_frame, (xA, yA), (xB, yB),
                            (0, 255, 0), 2)
    
    # Write the output video 
    out.write(right_frame.astype('uint8'))
    # Display the resulting frame
    cv2.imshow('Right',right_frame)
    cv2.imshow('Left', left_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
left_cam.release()
right_cam.release()
# and release the output
out.release()
# finally, close the window
cv2.destroyAllWindows()
cv2.waitKey(1)