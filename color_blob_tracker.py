import roslib; roslib.load_manifest('color_blob_tracker')
import rospy
from geometry_msgs.msg import Twist
from cmvision.msg import Blobs, Blob
from create_node.msg import TurtlebotSensorState
from math import *


#global
turn = 0.0
blob_position = 0

def callback(data):
    global turn
    global blob_position

    image_width = 640
    image_center = image_width / 2
    max_turn = 0.5
    scale = 0.001
    turn = 0.0
    dead_band = 30 #set dead band in px from center
 
    blob_position = 0
    if(len(data.blobs)):
        for obj in data.blobs:
            blob_position = blob_position + obj.x

        blob_position = blob_position/len(data.blobs)
        rospy.loginfo("blob is at %s"%blob_position)

        # find the distance from the blob's center to the center of the image
        distance_from_center = blob_position - image_center

	# dead band of account for camera lag and prevent oscillations
	if abs(distance_from_center) <  dead_band:
		turn = 0
		return

        # calculate a turning velocity that's proportional to the distance from center
        # scale this value to adjust the responsiveness
        turn_raw = -scale * distance_from_center * max_turn

        # We need to keep the turn velocity between 0.5 and -0.5
        # Get the max value of the absolute value and max_turn velocity
        turn_magnitude = max(abs(turn_raw), max_turn)

        # Set the turn velocity to match the magnitude of the enveloped value
        # Set the direction to the direction of the raw value
        turn = copysign(turn_magnitude, turn_raw)
  
def run():
    global blob_position
    # publish twist messages to turtlebot
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)

    #subscribe to the blobs from cmvision
    rospy.Subscriber('/blobs', Blobs, callback)
    rospy.init_node('color_blob_tracker')

    global turn
    twist = Twist()

    while not rospy.is_shutdown():

        # turn to track blob
        if ( turn != 0.0 ):
            str = "Turning %s"%turn
            rospy.loginfo(str)
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn
            turn = 0.0

            # stop otherwise
        else:
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

        # send the message and delay
        pub.publish(twist)
	blob_position = 0
        rospy.sleep(1.5)

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException: pass
