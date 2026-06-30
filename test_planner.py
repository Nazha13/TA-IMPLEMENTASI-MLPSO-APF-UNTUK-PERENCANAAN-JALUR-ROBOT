#!/usr/bin/env python
import rospy
import argparse
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseStamped

def create_pose(x, y, frame_id="map"):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose

def test_planner(args):
    # Menggunakan anonymous=True agar kita bisa run beberapa tester sekaligus jika perlu
    rospy.init_node('global_planner_tester', anonymous=True)
    
    rospy.loginfo("Menunggu service /move_base/make_plan...")
    rospy.wait_for_service('/move_base/make_plan')
    make_plan_service = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
    
    start_pose = create_pose(args.start_x, args.start_y)
    goal_pose = create_pose(args.goal_x, args.goal_y)

    req = GetPlanRequest()
    req.start = start_pose
    req.goal = goal_pose
    req.tolerance = 0.5 

    rospy.loginfo("=====================================================")
    rospy.loginfo("Memulai Pengujian: %s", args.planner_name)
    rospy.loginfo("Rute      : Start(%.2f, %.2f) -> Goal(%.2f, %.2f)", args.start_x, args.start_y, args.goal_x, args.goal_y)
    rospy.loginfo("Iterasi   : %d kali", args.runs)
    rospy.loginfo("=====================================================")

    for i in range(args.runs):
        try:
            resp = make_plan_service(req)
            if len(resp.plan.poses) > 0:
                rospy.loginfo("[Run %02d/%02d] %s: Sukses (Path length: %d)", i+1, args.runs, args.planner_name, len(resp.plan.poses))
            else:
                rospy.logwarn("[Run %02d/%02d] %s: Gagal menemukan path.", i+1, args.runs, args.planner_name)
                
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            
        rospy.sleep(0.5) # Jeda untuk memberikan waktu node C++ menulis CSV

if __name__ == '__main__':
    # Setup Argument Parser
    parser = argparse.ArgumentParser(description='General Global Planner Benchmarking Tool')
    parser.add_argument('--start_x', type=float, default=0.0, help='Koordinat X Start')
    parser.add_argument('--start_y', type=float, default=0.0, help='Koordinat Y Start')
    parser.add_argument('--goal_x', type=float, default=10.0, help='Koordinat X Goal')
    parser.add_argument('--goal_y', type=float, default=5.0, help='Koordinat Y Goal')
    parser.add_argument('--runs', type=int, default=40, help='Jumlah iterasi pengujian (default: 40)')
    parser.add_argument('--planner_name', type=str, default='Global Planner', help='Nama algoritma untuk visualisasi log')

    # rospy.myargv() memfilter argumen bawaan ROS agar tidak error dengan argparse
    args = parser.parse_args(rospy.myargv()[1:])

    try:
        test_planner(args)
    except rospy.ROSInterruptException:
        pass