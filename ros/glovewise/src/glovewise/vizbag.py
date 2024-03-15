import rosbag
import rospy
import visualization_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import pyglovewise
import mittenwire


class FastMarkerArray:
    _type = visualization_msgs.msg.MarkerArray._type
    _md5sum = visualization_msgs.msg.MarkerArray._md5sum
    _full_text = visualization_msgs.msg.MarkerArray._full_text

    def __init__(self, data):
        self.data = bytes(data)

    def serialize(self, buff):
        buff.write(self.data)


class VizBag:

    def create(filename):
        self = VizBag()
        self.filename = filename
        self.messages = []
        return self

    def write(self):
        self.bag = rosbag.Bag(self.filename, "w")
        messages = sorted(self.messages, key=lambda x: x[2])
        for m in messages:
            self.bag.write(m[0], m[1], rospy.Time.from_sec(m[2]))
        self.bag.close()

    def begin_frame(self, time, topic="/tractor/visualization"):
        self.marker_array = visualization_msgs.msg.MarkerArray()
        self.messages.append((topic, self.marker_array, time))

    def visualize_points(self, name, size, color, points):
        marker = visualization_msgs.msg.Marker()
        marker.ns = name
        marker.type = visualization_msgs.msg.Marker.POINTS
        marker.points = [geometry_msgs.msg.Point(
            x=p[0], y=p[1], z=p[2]) for p in points]
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        self.marker_array.markers.append(marker)

    def visualize_colored_points(self, name, size, colors, points):
        marker = visualization_msgs.msg.Marker()
        marker.ns = name
        marker.type = visualization_msgs.msg.Marker.POINTS
        marker.points = [geometry_msgs.msg.Point(
            x=p[0], y=p[1], z=p[2]) for p in points]
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.colors = [std_msgs.msg.ColorRGBA(
            r=p[0], g=p[1], b=p[2], a=p[3]) for p in colors]
        self.marker_array.markers.append(marker)

    def visualize_lines(self, name, size, color, points):
        marker = visualization_msgs.msg.Marker()
        marker.ns = name
        marker.type = visualization_msgs.msg.Marker.LINE_LIST
        marker.points = [geometry_msgs.msg.Point(
            x=p[0], y=p[1], z=p[2]) for p in points]
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        self.marker_array.markers.append(marker)

    def visualize_colored_lines(self, name, size, colors, points):
        marker = visualization_msgs.msg.Marker()
        marker.ns = name
        marker.type = visualization_msgs.msg.Marker.LINE_LIST
        marker.points = [geometry_msgs.msg.Point(
            x=p[0], y=p[1], z=p[2]) for p in points]
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.colors = [std_msgs.msg.ColorRGBA(
            r=p[0], g=p[1], b=p[2], a=(p[3] if len(p) > 3 else 1)) for p in colors]
        self.marker_array.markers.append(marker)

    def visualize_mesh(self, name, color, points):
        marker = visualization_msgs.msg.Marker()
        marker.ns = name
        marker.type = visualization_msgs.msg.Marker.TRIANGLE_LIST
        marker.points = [geometry_msgs.msg.Point(
            x=p[0], y=p[1], z=p[2]) for p in points]
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        self.marker_array.markers.append(marker)

    def visualize_colored_mesh(self, name, colors, points):
        marker = visualization_msgs.msg.Marker()
        marker.ns = name
        marker.type = visualization_msgs.msg.Marker.TRIANGLE_LIST
        marker.points = [geometry_msgs.msg.Point(
            x=p[0], y=p[1], z=p[2]) for p in points]
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.colors = [std_msgs.msg.ColorRGBA(
            r=p[0], g=p[1], b=p[2], a=p[3]) for p in colors]
        self.marker_array.markers.append(marker)

    def visualize_mesh_fast(self, topic, time, name, color, points):
        message_data = pyglovewise.build_mesh_message(name, color, points)
        self.messages.append((topic, FastMarkerArray(message_data), time))

    def visualize_colored_mesh_fast(self, topic, time, name, colors, points):
        with mittenwire.Profiler("vizbag serialize colored mesh", 0):
            message_data = pyglovewise.build_colored_mesh_message(
                name, colors, points)
        with mittenwire.Profiler("vizbag pack colored mesh", 0):
            self.messages.append((topic, FastMarkerArray(message_data), time))

    def insert_message(self, topic, message, time):
        self.messages.append((topic, message, time))
