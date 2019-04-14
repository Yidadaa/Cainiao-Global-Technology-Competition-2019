import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import os
import cv2
import numpy as np
import base64
import ntpath
import time

rospy.init_node('listener', anonymous=True)
image_publisher = rospy.Publisher('/cainiao_camera/image_raw', Image, queue_size=1)
bridge = CvBridge()

def publish_image(image_data, ts):
    image = bridge.cv2_to_imgmsg(image_data, encoding='bgr8')
    image_publisher.publish(image)
    return
    image = Image()
    ts = rospy.Time.from_sec(float(ts) / 1000)
    header = Header(stamp=ts)
    header.frame_id = 'map'
    image.height = image_data.shape[0]
    image.width = image_data.shape[1]
    image.encoding = 'bgr8'
    image.data = np.array(image_data).tostring()
    image.header = header
    image_publisher.publish(image)

def publish_video(video_path):
    base_string = ntpath.basename(video_path).replace('.mp4', '')
    base_ts = int(base64.decodestring(base_string + '=='))

    cvideo = cv2.VideoCapture(video_path)
    success, image = cvideo.read()

    (h, w) = image.shape[:2]
    print(h, w)
    center = (w / 2, h / 2)
    angle = -getRotationOf(video_path)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)

    while success:
        delay = 1000 / 30
        base_ts += delay
        print base_ts, delay
        publish_image(cv2.warpAffine(image, M, (h, w)), base_ts)
        success, image = cvideo.read()
        time.sleep(1.0 / 30)

def getRotationOf(video):
    sh = 'ffprobe -loglevel error -select_streams v:0 -show_entries stream_tags=rotate -of default=nw=1:nk=1 -i ' + video
    rotation = os.popen(sh).read().replace('\n', '')
    return int(rotation)

if __name__ == '__main__':
    root = './files/'
    files = os.listdir(root)
    video_path = ''
    for f in files:
        if f[-1] == '4':
            video_path = root + f
    publish_video(video_path)