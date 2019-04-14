import thread, os
import rospy

from imu_topic import publish_imu
from img_topic import publish_video

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    root = './files/'
    files = os.listdir(root)
    imu_path = ''
    video_path = ''
    for f in files:
        if f[-1] == 'a':
            imu_path = root + f
        if f[-1] == '4':
            video_path = root + f
    print imu_path
    print video_path
    thread.start_new_thread(publish_video, (video_path,))
    thread.start_new_thread(publish_imu, (imu_path,))

    while 1:
        pass