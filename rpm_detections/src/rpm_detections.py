#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import pymysql
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID

class ObstacleMonitor:
    def __init__(self):
        rospy.init_node('obstacle_monitor')

        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

        self.min_distance = 0.4  # 장애물 감지 임계값 (미터)
        self.stop_cmd = Twist()  # 정지 명령
        self.current_goal = None
        self.obstacle_detected = False
        self.human_detected = False

        self.goal_sub = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_callback)

        # Database connection details
        self.servername = "13.124.83.151"
        self.username = "rpm"
        self.password = "11223344"
        self.dbname = "rpm"
        self.class_name = None

        # Initialize database connection
        self.db_connection = None
        self.connect_to_database()

        # Timer for periodic data read
        self.timer = rospy.Timer(rospy.Duration(0.5), self.read_database_data)  # 0.5초마다 실행 (조정 가능)

    def connect_to_database(self):
        try:
            self.db_connection = pymysql.connect(
                host=self.servername,
                user=self.username,
                password=self.password,
                database=self.dbname,
                connect_timeout=10,  # 연결 타임아웃 설정 (초 단위)
                autocommit=True      # 자동 커밋 활성화
            )
            rospy.loginfo("데이터베이스 연결 성공")
        except pymysql.MySQLError as e:
            rospy.logerr("데이터베이스 연결 오류: %s" % e)

    def read_database_data(self, event=None):
        if self.db_connection and self.db_connection.open:
            try:
                with self.db_connection.cursor() as cursor:
                    query = "SELECT class_name FROM detections WHERE detections_no = 1"
                    cursor.execute(query)
                    result = cursor.fetchone()

                    if result:
                        new_class_name = result[0]
                        # rospy.loginfo("데이터베이스 데이터: %s" % new_class_name)

                        if new_class_name.startswith("person") and not self.human_detected:
                            self.cancel_goal()
                            self.cmd_vel_pub.publish(self.stop_cmd)
                            # rospy.loginfo("인간 감지! 로봇이 정지됩니다.")
                            print("인간 감지! 로봇이 정지됩니다.")
                            self.human_detected = True
                            self.obstacle_detected = True  # 인간 감지는 장애물 감지로 간주
                        elif not new_class_name.startswith("person") and self.human_detected:
                            if not self.obstacle_detected:
                                self.publish_goal(self.current_goal)
                            # rospy.loginfo("인간 없음. 로봇이 움직입니다.")
                            self.human_detected = False

                    else:
                        rospy.logwarn("데이터베이스에서 반환된 데이터가 없습니다.")
                        self.class_name = None

            except pymysql.MySQLError as e:
                rospy.logerr("데이터베이스 쿼리 오류: %s" % e)
                self.class_name = None

        else:
            rospy.logwarn("데이터베이스 연결이 닫혔습니다. 재연결을 시도합니다.")
            self.connect_to_database()
            self.class_name = None

    def goal_callback(self, goal):
        self.current_goal = goal

    def lidar_callback(self, data):
        min_distance = min(data.ranges)

        if min_distance < self.min_distance:
            if not self.obstacle_detected:
                self.cancel_goal()
                self.cmd_vel_pub.publish(self.stop_cmd)
                # rospy.loginfo("장애물 감지! 로봇이 정지됩니다.")
                print("장애물 감지! 로봇이 정지됩니다.")
                self.obstacle_detected = True
        else:
            if self.obstacle_detected and not self.human_detected:
                self.publish_goal(self.current_goal)
                # rospy.loginfo("장애물 없음. 로봇이 움직입니다.")
                print("장애물 없음. 로봇이 움직입니다.")
                self.obstacle_detected = False

    def cancel_goal(self):
        cancel_msg = GoalID()
        self.cancel_pub.publish(cancel_msg)

    def publish_goal(self, goal):
        if goal:
            self.goal_pub.publish(goal)

if __name__ == '__main__':
    try:
        ObstacleMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
