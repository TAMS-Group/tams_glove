import cv2
import rospy
import numpy
import cv_bridge
import rosbag


class BagImage:
    pass


class BagImageSet:
    pass


class ImageBag:

    def __init__(self, path):
        self.path = path

    def __iter__(self):
        bridge = cv_bridge.CvBridge()
        with rosbag.Bag(self.path, "r") as bag:
            last_time = rospy.Time(0)

            image_messages = {}
            camera_info_messages = {}

            bag_start_time = bag.get_start_time()
            bag_end_time = bag.get_end_time()

            self.start_time = bag_start_time
            self.end_time = bag_end_time

            for topic, message, mtime in bag:

                if mtime != last_time:

                    bag_images = []
                    for name in (image_messages.keys() & camera_info_messages.keys()):
                        image_time, image_message = image_messages[name]
                        info_time, info_message = camera_info_messages[name]
                        if image_time == info_time:
                            b = BagImage()
                            b.time = last_time
                            b.name = name
                            b.image = image_message
                            b.info = info_message
                            bag_images.append(b)
                    if len(bag_images) > 0:
                        bag_images = sorted(
                            bag_images, key=lambda img: img.name)
                        image_set = BagImageSet()
                        image_set.images = bag_images
                        image_set.time = last_time
                        yield image_set

                    last_time = mtime
                    image_messages = {}
                    camera_info_messages = {}

                if topic.endswith("/image_raw/compressed"):
                    image_messages[topic[:-
                                         len("/image_raw/compressed")]] = (mtime, message)

                if topic.endswith("/camera_info"):
                    camera_info_messages[topic[:-
                                               len("/camera_info")]] = (mtime, message)
