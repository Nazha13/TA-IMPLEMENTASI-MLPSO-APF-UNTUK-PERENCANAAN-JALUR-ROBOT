// Deklarasi pustaka yang digunakan
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <base_local_planner/goal_functions.h>
#include <vector>
#include <cmath>

#include <pluginlib/class_list_macros.h>

namespace APF {

class APFPlanner : public nav_core::BaseLocalPlanner {
    private:
        bool initialized_ = false;
        std::vector<geometry_msgs::PoseStamped> global_plan_;
        tf2_ros::Buffer* tf_;
        costmap_2d::Costmap2DROS* costmap_ros_;

        // Parameter APF
        double k_att_ = 2.0;
        double normal_k_rep_ = 1.0;
        double normal_d0_ = 0.8;
        double avoid_k_rep_ = 0.004;
        double avoid_d0_ = 0.6;
        
        // Parameter kalkulasi virtual point
        double avoid_k1_ = 4.0;
        double avoid_k2_ = 2.0;
        double avoid_k3_ = 5.0;
        double avoid_k4_ = 1.5;

        // Parameter tambahan untuk kontrol
        double lookahead_dist_ = 0.6;
        double scan_radius_ = 1.0; 
        double max_vel_ = 0.2;
        double min_vel_ = 0.02;
        double decel_rad_ = 0.5;
        double k_theta_ = 2.0;
        double max_angular_vel_ = 0.4;
        double min_angular_vel_ = -0.4;
        double xy_goal_tolerance_ = 0.25;
	    double robot_radius_ = 0.3;
	    double vp_radius_ = 0.3;

        // Variabel kondisi
        double prev_ftotal_x_ = 0.0;
        double prev_ftotal_y_ = 0.0;
        
        enum ApfState {NORMAL, AVOIDANCE};
        ApfState current_state_ = NORMAL;
        bool is_complete_ = false;

        // Variabel deteksi stuck
        int avoidance_counter_ = 0;
        int avoidance_threshold_ = 1000; // time to avoid limit
        
        ros::Time last_avoidance_time_ = ros::Time(0);
        ros::Time last_pose_time_ = ros::Time(0);
        double last_pose_x_ = 0.0;
        double last_pose_y_ = 0.0;
        double stuck_time_threshold_ = 10.0; // seconds
        double stuck_distance_threshold_ = 0.05; // meters 

        // Fungsi untuk melihat apakah garis antara dua titik aman dari rintangan berdasarkan costmap
        bool pathCost(double start_x, double start_y, double end_x, double end_y, costmap_2d::Costmap2D* costmap){
            double dist = std::hypot(end_x - start_x, end_y - start_y);
            int steps = std::max(1, (int)std::ceil(dist / (costmap->getResolution())));

            for(int i = 0; i <= steps; ++i){
                double t = (double)i / steps;
                double sample_x = start_x + t * (end_x - start_x);
                double sample_y = start_y + t * (end_y - start_y);

                unsigned int mx, my;
                if(costmap->worldToMap(sample_x, sample_y, mx, my)){
                    if(costmap->getCost(mx, my) >= 200){ 
                        return false;
                    }
                }
                else {
                    return false;
                }
            }
            return true;
        }

    public:
        APFPlanner() {}
        APFPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
            initialize(name, tf, costmap_ros);
        }

        void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
            if(!initialized_){
                ros::NodeHandle private_nh("~/" + name);
                tf_ = tf;
                costmap_ros_ = costmap_ros;
                initialized_ = true;
                ROS_INFO("APF Planner Plugin initialized.");
            }
        }

        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
            global_plan_ = plan;
            is_complete_ = false;
            return true;
        }

        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
            if(global_plan_.empty()) return false;

            // Reset timestamp untuk mencegah masalah sinkronisasi transformasi 
            for (auto& pose : global_plan_) {
                pose.header.stamp = ros::Time(0);
            }

            // Membaca posisi robot 
            geometry_msgs::PoseStamped global_pose;
            costmap_ros_->getRobotPose(global_pose);
            global_pose.header.stamp = ros::Time(0);

            double robot_x = global_pose.pose.position.x;
            double robot_y = global_pose.pose.position.y;
            double robot_yaw = tf2::getYaw(global_pose.pose.orientation);

            ros::Time now = ros::Time::now();

            if(last_pose_time_ == ros::Time(0)){
                last_pose_time_ = now;
                last_pose_x_ = robot_x;
                last_pose_y_ = robot_y;
            }

            // Deteksi apakah robot bergerak atau tidak
            if((now - last_pose_time_).toSec() > stuck_time_threshold_){
                double dist_moved = std::hypot(robot_x - last_pose_x_, robot_y - last_pose_y_);
                
                if(dist_moved < stuck_distance_threshold_){
                    ROS_WARN("APF: Robot seems to be stuck either from local minima or flickering states.");
                    current_state_ = NORMAL;
                    last_pose_time_ = ros::Time(0);
                    avoidance_counter_ = 0;
                    return false;
                } else {
                    last_pose_time_ = now;
                    last_pose_x_ = robot_x;
                    last_pose_y_ = robot_y;
                }
            }

            // Transformasi titik tujuan global ke frame lokal
            geometry_msgs::PoseStamped map_goal = global_plan_.back();
            geometry_msgs::PoseStamped local_goal;

            try {
                tf_->transform(map_goal, local_goal, costmap_ros_->getGlobalFrameID());
            } catch (tf2::TransformException &ex) {
                ROS_WARN_THROTTLE(1.0, "APF: TF Transform Error: %s", ex.what());
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;
                return true;
            }

            double goal_x = local_goal.pose.position.x;
            double goal_y = local_goal.pose.position.y;
            double dist_to_goal = std::hypot(goal_x - robot_x, goal_y - robot_y);

            if(dist_to_goal <= xy_goal_tolerance_){
                is_complete_ = true;
                ROS_INFO_ONCE("APF: Target Reached");
            }

            if (is_complete_) {
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;
                return true;
            }

            // Transformasi/ekstraksi global plan ke dalam frame lokal
            std::vector<geometry_msgs::PoseStamped> transformed_plan;
            if (!base_local_planner::transformGlobalPlan(
                    *tf_, global_plan_, global_pose, 
                    *(costmap_ros_->getCostmap()), 
                    costmap_ros_->getGlobalFrameID(), 
                    transformed_plan)) {
                
                ROS_WARN("APF Debugger: TF Transform failed. Halting robot for safety.");
                cmd_vel.linear.x = 0.0; cmd_vel.linear.y = 0.0; cmd_vel.angular.z = 0.0;
                return false; 
            }

            if(transformed_plan.empty()){
                ROS_WARN("APF Debugger: Transformed plan is empty. Halting robot for safety.");
                cmd_vel.linear.x = 0.0; cmd_vel.linear.y = 0.0; cmd_vel.angular.z = 0.0;
                return false; 
            }

            double target_x = robot_x;
            double target_y = robot_y;
            
            costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
            int size_x = costmap->getSizeInCellsX();
            int size_y = costmap->getSizeInCellsY();

            // Reset obs vars
            double min_dist_obs = 1e9;
            double closest_obs_x = 0.0;
            double closest_obs_y = 0.0;
            
            int radius_cells = std::ceil(scan_radius_ / costmap->getResolution());
            unsigned int robot_mx, robot_my;

            if (costmap->worldToMap(robot_x, robot_y, robot_mx, robot_my)) {
                int min_x = std::max(0, (int)robot_mx - radius_cells);
                int max_x = std::min((int)size_x - 1, (int)robot_mx + radius_cells);
                int min_y = std::max(0, (int)robot_my - radius_cells);
                int max_y = std::min((int)size_y - 1, (int)robot_my + radius_cells);

                for(int i = min_x; i <= max_x; ++i){
                    for(int j = min_y; j <= max_y; ++j){
                        if(costmap->getCost(i,j) >= 200){
                            double obs_x, obs_y;
                            costmap->mapToWorld(i, j, obs_x, obs_y);
                            double dist_to_obs = std::hypot(robot_x - obs_x, robot_y - obs_y);

                            // Memilih 1 titik terdekat untuk mewakili rintangan
                            if(dist_to_obs < scan_radius_){
                                if(dist_to_obs < min_dist_obs){
                                    min_dist_obs = dist_to_obs;
                                    closest_obs_x = obs_x;
                                    closest_obs_y = obs_y;
                                }
                            }
                        }
                    }
                }
            } else {
                ROS_WARN_THROTTLE(1.0, "APF: Robot is off the costmap!");
            }

            // Mencari titik waypoint terdekat pada jalur yang telah ditransformasikan
            int closest_index = 0;
            double min_dist_to_path = 1e9;
            for(int i = 0; i < transformed_plan.size(); ++i){
                double wp_x = transformed_plan[i].pose.position.x;
                double wp_y = transformed_plan[i].pose.position.y;
                double dist_to_wp = std::hypot(wp_x - robot_x, wp_y - robot_y);

                if(dist_to_wp < min_dist_to_path){
                    min_dist_to_path = dist_to_wp;
                    closest_index = i;
                }
            }

            // Logika pemilihan titik waypoint berdasarkan lookahead distance dan keamanan jalur
            int best_id = closest_index;

            if(dist_to_goal < decel_rad_){
                target_x = goal_x;
                target_y = goal_y;
            } else {
                int cell_R = std::ceil((robot_radius_) / costmap->getResolution());

                for(int i = closest_index; i < transformed_plan.size(); ++i){
                    double wp_x = transformed_plan[i].pose.position.x;
                    double wp_y = transformed_plan[i].pose.position.y;
                    double dist_to_wp = std::hypot(wp_x - robot_x, wp_y - robot_y);
                
                    unsigned int mx, my;
                    bool path_is_safe = true;
                    if (costmap->worldToMap(wp_x, wp_y, mx, my)) {
                        for(int dx = -cell_R; dx <= cell_R; ++dx){
                            for(int dy = -cell_R; dy <= cell_R; ++dy){
                                int check_x = mx + dx;
                                int check_y = my + dy;
                                if(check_x >= 0 && check_x < size_x && check_y >= 0 && check_y < size_y){
                                    if(costmap->getCost(check_x, check_y) >= 200){
                                        path_is_safe = false;
                                        break;
                                    }
                                }
                            }
                            if(!path_is_safe) break;
                        }
                    }

                    if(!path_is_safe) continue;

                    best_id = i;
                    if(dist_to_wp >= lookahead_dist_) break;
                }

                if(best_id >= transformed_plan.size()) best_id = transformed_plan.size() - 1;

                target_x = transformed_plan[best_id].pose.position.x;
                target_y = transformed_plan[best_id].pose.position.y;
            }

            // Logika AVOIDANCE (Mencari titik virtual alternatif jika terjebak)
            if(current_state_ == AVOIDANCE){
                avoidance_counter_++;

                if(avoidance_counter_ > avoidance_threshold_){
                    ROS_WARN("APF : Stuck in avoidance for too long. Let's see what PSO can do.");
                    current_state_ = NORMAL;
                    avoidance_counter_ = 0;
                    return false;
                }

                int num_points = 40;
                double R = vp_radius_;
                double best_score = -1e9;
                double best_vx = target_x;
                double best_vy = target_y;
                double global_target_x = target_x;
                double global_target_y = target_y;

                for(int i = 0; i < num_points; ++i){
                    double angle = (i * (2.0 * M_PI / num_points));
                    double vx = robot_x + R * std::cos(angle);
                    double vy = robot_y + R * std::sin(angle);

                    unsigned int mx, my;
                    bool is_valid = true;
                    if(costmap->worldToMap(vx, vy, mx, my)){
                        if(costmap->getCost(mx, my) >= 128) is_valid = false;
                    } else {
                        is_valid = false;
                    }

                    if(!is_valid) continue;

                    double v_yaw = std::atan2(vy - robot_y, vx - robot_x);
                    double cos_theta1 = std::cos(v_yaw - robot_yaw);
                    double cos_theta2 = std::cos(v_yaw - std::atan2(target_y - robot_y, target_x - robot_x));

                    double cost_penalty = 0.0;
                    if(costmap->worldToMap(vx, vy, mx, my)){
                        cost_penalty = (double)costmap->getCost(mx,my) / 128.0;
                    }

                    double l = std::hypot(vx - target_x, vy - target_y);
                    l /= (l + 1.0); // normalisasi ke 0 - 1 agar tidak mendominasi parameter lain yang rentang nilainya 0 - 1
                    double eval_score = (avoid_k1_ * cos_theta1) + (avoid_k2_ * cos_theta2) - (avoid_k3_ * cost_penalty) - (avoid_k4_ * l);

                    if(eval_score > best_score){
                        best_score = eval_score;
                        best_vx = vx;
                        best_vy = vy;
                    }
                }

                if(best_score == -1e9){
                    ROS_WARN("APF : No valid virtual point found! spinning in place");
                    cmd_vel.linear.x = 0.0; cmd_vel.linear.y = 0.0; cmd_vel.angular.z = 0.5;
                    return true;
                }

                target_x = best_vx;
                target_y = best_vy;

                // Cek apakah sudah cukup aman untuk kembali ke mode NORMAL
                if(pathCost(robot_x, robot_y, global_target_x, global_target_y, costmap) && min_dist_obs > 0.6){
                    ROS_INFO("APF : Safe enough, let's go back to NORMAL mode.");
                    current_state_ = NORMAL;
                    avoidance_counter_ = 0;
                    last_avoidance_time_ = ros::Time::now();
                }
            }

            // Set parameter gaya secara dinamis berdasarkan state aktif
            double active_k_rep = (current_state_ == NORMAL) ? normal_k_rep_ : avoid_k_rep_;
            double active_d0    = (current_state_ == NORMAL) ? normal_d0_ : avoid_d0_;

            // Kalkulasi Gaya Total
            double f_att = k_att_ * std::hypot(target_x - robot_x, target_y - robot_y);
            double f_att_x = f_att * (target_x - robot_x)/std::hypot(target_x - robot_x, target_y - robot_y);
            double f_att_y = f_att * (target_y - robot_y)/std::hypot(target_x - robot_x, target_y - robot_y);

            double f_rep_x = 0.0;
            double f_rep_y = 0.0;

            if(min_dist_obs <= active_d0){
                if(min_dist_obs < 0.15) min_dist_obs = 0.15; // prevent division by zero and explosive repulsive force
                double f_rep = active_k_rep * ((1.0 / min_dist_obs) - (1.0 / active_d0)) * std::pow((1.0 / min_dist_obs), 2);
                f_rep_x = f_rep * ((robot_x - closest_obs_x) / min_dist_obs);
                f_rep_y = f_rep * ((robot_y - closest_obs_y) / min_dist_obs);
            }

            double f_total_x = f_att_x + f_rep_x;
            double f_total_y = f_att_y + f_rep_y;

            // Deteksi Osilasi (Untuk mode NORMAL)
            if(current_state_ == NORMAL){
                double prev_mag = std::hypot(prev_ftotal_x_, prev_ftotal_y_);
                double curr_mag = std::hypot(f_total_x, f_total_y);

                if(curr_mag > 0.01 && prev_mag > 0.01){
                    double cos_theta = ((f_total_x * prev_ftotal_x_) + (f_total_y * prev_ftotal_y_)) / (curr_mag * prev_mag);
                    if(cos_theta < 0.0 && (ros::Time::now() - last_avoidance_time_).toSec() > 3.0){
                        ROS_WARN("APF : Oscillation detected. Switching to AVOIDANCE mode.");
                        current_state_ = AVOIDANCE;
                    }
                }
            }

            prev_ftotal_x_ = f_total_x; 
            prev_ftotal_y_ = f_total_y; 
            double target_yaw = std::atan2(f_total_y, f_total_x);

            // Kalkulasi kecepatan linear dan angular
            double current_max_vel = max_vel_;
            if(dist_to_goal < decel_rad_){
                double scale = dist_to_goal / decel_rad_;
                double vel = max_vel_ * scale;
                current_max_vel = std::max(min_vel_, vel);
            }

            double vel_x = f_total_x;
            double vel_y = f_total_y;

            double vel_mag = std::hypot(f_total_x, f_total_y);

            if(vel_mag > current_max_vel){
                double scale = current_max_vel / vel_mag;
                vel_x *= scale;
                vel_y *= scale;
            }

            double raw_yaw_diff = target_yaw - robot_yaw;
            double yaw_diff = std::atan2(std::sin(raw_yaw_diff), std::cos(raw_yaw_diff));
            double angular_vel = k_theta_ * yaw_diff;

            // Transformasi ke base link frame dari odom frame
            double local_vel_x = vel_x * std::cos(robot_yaw) + vel_y * std::sin(robot_yaw);
            double local_vel_y = -vel_x * std::sin(robot_yaw) + vel_y * std::cos(robot_yaw);

            double align_tolerance = 0.52; // ~30 derajat

            if(std::abs(yaw_diff) > align_tolerance){
                local_vel_x = 0.0;
                local_vel_y = 0.0;
            }

            if(angular_vel > max_angular_vel_) angular_vel = max_angular_vel_;
            if(angular_vel < min_angular_vel_) angular_vel = min_angular_vel_;

            cmd_vel.linear.x = local_vel_x;
            cmd_vel.linear.y = local_vel_y;
            cmd_vel.angular.z = angular_vel;

            ROS_INFO_THROTTLE(1.0, "Dist to goal: %.2f | State: %s", dist_to_goal, (current_state_ == NORMAL ? "NORMAL" : "AVOIDANCE"));

            return true;
        }

        bool isGoalReached(){
            return is_complete_;
        }
};

}

PLUGINLIB_EXPORT_CLASS(APF::APFPlanner, nav_core::BaseLocalPlanner);