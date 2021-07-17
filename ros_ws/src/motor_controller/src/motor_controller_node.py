#!/usr/bin/env python
from math import pi, cos, sin

from functools import partial
import rospy
import tf
import socket
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from ros_opcua_srvs.srv import Subscribe,Write,Unsubscribe
from ros_opcua_msgs.msg import *

__author__ = "buhl@brommeweb.dk (Jacob Buh)"


# TODO need to find some better was of handling OSerror 11 or preventing it, any ideas?

class EncoderOdom:
    def __init__(self):
  
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.last_enc_time = rospy.Time.now()

    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

  
    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        quat = tf.transformations.quaternion_from_euler(0, 0, cur_theta)
        current_time = rospy.Time.now()

        br = tf.TransformBroadcaster()
        br.sendTransform((cur_x, cur_y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, cur_theta),
                         current_time,
                         "base_link",
                         "odom")

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)

        
class Node:
    def __init__(self):
        rospy.init_node("motor_controller_node")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to motor controller")
        
        self.unsubscribe = False
        self.twist = Twist()
        
        self.odomData = {}
        self.odomData['x'] = 0.0
        self.odomData['y'] = 0.0
        self.odomData['th'] = 0.0
        self.odomData['vx'] = 0.0
        self.odomData['vth'] = 0.0


        self.MAX_SPEED = float(rospy.get_param("~max_speed", "2.0"))
        self.MIN_SPEED = float(rospy.get_param("~min_speed", "-2.0"))
        
        self.TICKS_PER_METER = float(rospy.get_param("~tick_per_meter", "105860"))
        self.BASE_WIDTH = float(rospy.get_param("~base_width", "0.35"))
        
        self.VEL_TOPIC = '/'+rospy.get_param("~vel_topic", "cmd_vel")
        
        self.OPCUA_TOPIC_PREFIX = '/'+rospy.get_param("~opcua_namespace", "opcua") +'/'+rospy.get_param("~opcua_node_name", "opcua_client")+'/'
        
        self.encodm = EncoderOdom()
        self.last_set_speed_time = rospy.get_rostime()


        rospy.Subscriber(self.VEL_TOPIC, Twist, self.vel_callback)

        rospy.sleep(1)

        rospy.logdebug("max_speed %f", self.MAX_SPEED)
        rospy.logdebug("min_speed %f", self.MIN_SPEED)
        rospy.logdebug("ticks_per_meter %f", self.TICKS_PER_METER)
        rospy.logdebug("base_width %f", self.BASE_WIDTH)
        rospy.logdebug("OPC UA topic prefix %s", self.OPCUA_TOPIC_PREFIX)
        rospy.logdebug("vel topic %s", self.VEL_TOPIC)
        
        rospy.loginfo("Wait for service write")
        rospy.wait_for_service(self.OPCUA_TOPIC_PREFIX+'write')
        self.opcuaWrite = rospy.ServiceProxy(self.OPCUA_TOPIC_PREFIX+'write', Write)
        
        rospy.loginfo("Wait for service unsubscribe")
        rospy.wait_for_service(self.OPCUA_TOPIC_PREFIX+'unsubscribe')
        self.opcuaUnsubscribe = rospy.ServiceProxy(self.OPCUA_TOPIC_PREFIX+'unsubscribe', Unsubscribe)
        
        
        rospy.loginfo("Wait for service subscribe")
        rospy.wait_for_service(self.OPCUA_TOPIC_PREFIX+'subscribe')
        
        rospy.loginfo("call service")
        
        
        self.opcuaNodes = ['ns=6;s=::motorCtrl:ros.topics.odemetry.twist.twist.linear.x','vx'
                          ,'ns=6;s=::motorCtrl:ros.topics.odemetry.twist.twist.angular.z','vth'
                          ,'ns=6;s=::motorCtrl:ros.topics.odemetry.pose.pose.position.x','x'
                          ,'ns=6;s=::motorCtrl:ros.topics.odemetry.pose.pose.position.y','y'
                          ,'ns=6;s=::motorCtrl:ros.topics.odemetry.pose.pose.orientation.z','th'
                          ]
                          
        
        try:
            subscribe = rospy.ServiceProxy(self.OPCUA_TOPIC_PREFIX+'subscribe', Subscribe)
            
            
            i = 0;
            
            while i < len(self.opcuaNodes):
                address = Address()
                address.nodeId = self.opcuaNodes[i]
                address.qualifiedName = self.opcuaNodes[i].rsplit('.')[-1]
                resp1 = subscribe(address,self.opcuaNodes[i+1])
                
                if resp1:
                    rospy.loginfo("Subscribed to %s",self.opcuaNodes[i+1])
                    
                    rospy.Subscriber(self.OPCUA_TOPIC_PREFIX+self.opcuaNodes[i+1], TypeValue, partial(self.opcua_callback,var=self.opcuaNodes[i+1]))
                    
                    
                else:
                    rospy.logerr("Failed subscribe to %s, error: %s",self.opcuaNodes[i+1],resp1.error_message)
                
                
                i = i + 2
            
            self.unsubscribe = True
        
        
        
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def send_vel_to_motor(self,twist):
        value = TypeValue()
        address = Address()

        address.nodeId = 'ns=6;s=::motorCtrl:ros.topics.twist.linear.x'
        address.qualifiedName = address.nodeId.rsplit('.')[-1]		
        value.type = 'float64'
        value.double_d = twist.linear.x
        resp1 = self.opcuaWrite(address,value)
        if resp1.success:
            rospy.logdebug("Write to linear x: %f",value.double_d)
        else:
            rospy.logerr("Failed write to linear x, error: %s",resp1.error_message)


        address.nodeId = 'ns=6;s=::motorCtrl:ros.topics.twist.angular.z'
        address.qualifiedName = address.nodeId.rsplit('.')[-1]		
        value.double_d = twist.angular.z
        resp1 = self.opcuaWrite(address,value)
        if resp1.success:
            rospy.logdebug("Write to angular z: %f",value.double_d)
        else:
            rospy.logerr("Failed write to angular z, error: %s",resp1.error_message)

    def run(self):
        rospy.loginfo("Starting motor drive")
        r_time = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.encodm.publish_odom(self.odomData['x'],self.odomData['y'], self.odomData['th'], self.odomData['vx'], self.odomData['vth'])
            self.send_vel_to_motor(self.twist)
            self.twist = Twist();
            r_time.sleep()
            
    def vel_callback(self, twist):
		self.last_set_speed_time = rospy.get_rostime()
		rospy.logdebug('vel_callback')
		self.twist = twist;		
		
    def opcua_callback(self, node,var=''):
		self.odomData[var] = node.double_d
		rospy.logdebug('opcua_callback for %s=%f',var,node.double_d)
        #twist.linear.x
        
    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
		rospy.loginfo("Shutting down")
		
		i = 0;
		if self.unsubscribe:
			while i < len(self.opcuaNodes):
				address = Address()
				address.nodeId = self.opcuaNodes[i]
				address.qualifiedName = self.opcuaNodes[i].rsplit('.')[-1]
				resp1 = self.opcuaUnsubscribe(address)
				
				if resp1.success:
					rospy.loginfo("Unsubscribed to %s",self.opcuaNodes[i+1])
				else:
					rospy.logerr("Failed unsubscribe to %s, error: %s",self.opcuaNodes[i+1],resp1.error_message)
					
					
				i = i + 2
		
        

if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")

