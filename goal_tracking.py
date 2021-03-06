from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time

import redis



#workon cv

redis_host="localhost"
redis_port=6379
redis_password=""

# greenLower = (29, 86, 6) original
greenLower = (0, 45, 6)
# greenUpper = (64, 255, 255) original
greenUpper = (100, 255, 255)
pts = deque(maxlen=64)

vs=VideoStream(src=0).start()
time.sleep(2.0)

r=redis.StrictRedis(host=redis_host, port=redis_port, password=redis_password, decode_responses=True)
r.set("msg:hello", "Hello redis!!!!!")

#Based on the center of min enclosing circle, determine the shoot decision to send as a redis key to the controller
def set_redis(x,y,w,h,cx,cy):
	r.set("opencv2:rect", str([x,y,w,h]))
	r.set("opencv2:circle", str([cx,cy]))

	if (cx<0):
		r.set("opencv2:shoot_decision", "NO_GOAL")
	elif (cx>=0 and cx <=160):
		r.set("opencv2:shoot_decision", "LEFT")
	elif (cx>160 and cx<=350):
		r.set("opencv2:shoot_decision", "CENTER")
	else:
		r.set("opencv2:shoot_decision", "RIGHT")


while True:
	frame=vs.read()
	if frame is None:
		break

	# resize frame, blur, convert to HSV
	frame=imutils.resize(frame,width=600)
	blurred=cv2.GaussianBlur(frame,(11,11),0)
	hsv=cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)
	#Look for a certain range of colour (corresponding to the goal colour)
	mask=cv2.inRange(hsv,greenLower,greenUpper)
	#Erode and dilate to eliminate false positives
	mask=cv2.erode(mask,None,iterations=2)
	mask=cv2.dilate(mask,None,iterations=2)
	#returns the set of outlines (i.e., contours) that correspond to the detected green colour
	cnts=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts=imutils.grab_contours(cnts)
	center=None

	if len(cnts)>0:
		#obtain the contour with the largest area, and find the minimum enclosing circle
		c=max(cnts,key=cv2.contourArea)
		((x,y),radius)=cv2.minEnclosingCircle(c)
		#Obtain the centroid of the contour (not used for the final images in report)
		M=cv2.moments(c)
		center=(int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))

		if radius>10:
			#Draw the minimium enclosing circle 
			cv2.circle(frame,(int(x),int(y)),int(radius), (0,255,255),2)
			cv2.circle(frame,(int(x),int(y)),3, (0,0,255),2)
			#find the largest bounding box 
			#(rx,ry) = top left coordinate, and r2, rh =  width/height 
			rx,ry,rw,rh = cv2.boundingRect(c)
			# draw the biggest contour (c) in green
			cv2.rectangle(frame,(rx,ry),(rx+rw,ry+rh),(0,255,0),2)
			#Obtain the correct location of the bounding box, and get the corresponding state
			set_redis(rx,ry,rw,rh,int(x),int(y))
		else:
			set_redis(0,0,0,0,0,0)
	else:
		set_redis(-1,-1,-1,-1,-1,-1)

	pts.appendleft(center)

	cv2.imshow("Frame",frame)
	key=cv2.waitKey(1)&0xFF

	if key==ord("q"):
		break

"""
Add to CPP:
std::string GOAL_POSITION_KEY;
std::string GOAL_LEFT = "LEFT";
std::string GOAL_CENTER = "CENTER";
std::string GOAL_RIGHT = "RIGHT";
int main() {
	GOAL_POSITION_KEY = "opencv2:shoot_decision"
	std::string goal_position = redis_client.get(GOAL_POSITION_KEY);
	
	if (goal_position == GOAL_LEFT) {
		std::cout << "LEFT GOAL POSITION" << endl;
		state = LEFT_SWING;
	} else if (goal_position == GOAL_CENTER) {
		std::cout << "CENTER GOAL POSITION" << endl;
		state = CENTER_SWING;
	} else if (goal_position == GOAL_RIGHT) {
		std::cout << "RIGHT GOAL POSITION" << endl;
		state = RIGHT_SWING;
	} else {
		std::cout << "IMPOSSIBLE GOAL POSITION: " << goal_position << endl;
	}
}
"""
