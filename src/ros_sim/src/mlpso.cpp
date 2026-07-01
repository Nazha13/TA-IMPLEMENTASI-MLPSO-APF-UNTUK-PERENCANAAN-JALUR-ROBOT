// Deklarasi pustaka yang digunakan
#include <ros/ros.h>
#include <tuw_multi_robot_msgs/Graph.h> 
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <algorithm>
#include <pluginlib/class_list_macros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

// Deklarasi struct
struct Edge {
    int id;
    double weight;
    double penalty;
};

struct Node {
    int id;
    double x, y;
    std::vector<Edge> neighbours;
    int neighbours_count;
};

struct Particle {
    std::vector<double> pos_i;
    std::vector<double> velo;
    double score;
    double pBest_score;
    std::vector<double> pBestPos;
};

struct Global {
    std::vector<double> g_pos;
    double g_score;
};

struct Coordinate {
    double x, y;
};

struct PosNode {
    int id;
    double distance;
};

namespace mlpso {

class MLPSO : public nav_core::BaseGlobalPlanner {
    private:
        bool initialized_ = false;
        ros::Subscriber roadmap_sub_;
        ros::Publisher plan_pub_;
        ros::Publisher node_pub_;
        bool map_received = false;

        // Parameter untuk menggeser koordinat graf agar selaras dengan peta
        double offset_x = 13.313419; 
        double offset_y = 14.406021;

        // Parameter untuk fitness function
        constexpr static double W_DIST = 1.0;
        constexpr static double W_TURN = 0.5;
        constexpr static double W_EDGE = 100.0;
        constexpr static double COLLISION_PENALTY = 100000.0;

        // Parameter untuk PSO
        double c1 = 1.85; 
        double c2 = 2.0;  
        double w = 0.65;  
        
        int swarm_size = 65;
        int max_iter = 1000;
        int max_retries = 10;
        
        // Parameter untuk early stopping
        int patience = 220; 
        double min_improvement = 1e-4;

        double start_yaw = 0.0;

        std::vector<Node> nodes;
        Coordinate real_start;
        Coordinate real_goal;

        costmap_2d::Costmap2D* costmap_;

        // Fungsi untuk menghitung jarak Euclidean antara dua node
        double dist(Node* a, Node* b){
            return std::sqrt(std::pow(b->x - a->x, 2) + std::pow(b->y - a->y, 2));
        }

        // Fungsi untuk menghasilkan bilangan acak dengan tipe double pada rentang tertentu
        double randdbl(double min, double max){
            return min + (double)std::rand()/RAND_MAX * (max - min);
        }

        // Fungsi untuk menghitung cost dari edge antara dua node berdasarkan costmap
        double getEdgeCost(Node* a, Node* b){
            if(!costmap_) return 0.0;

            double res = costmap_->getResolution();
            if(res == 0.0) return 0.0;

            double distance = std::hypot(b->x - a->x, b->y - a->y);
            int steps = std::max(1, (int)(distance / res));
            double edge_cost = 0.0;

            for(int i = 0; i <= steps; ++i){
                double t = (double)i / steps;
                double sample_x = a->x + t * (b->x - a->x);
                double sample_y = a->y + t * (b->y - a->y);
                unsigned int map_x, map_y;

                if(costmap_->worldToMap(sample_x, sample_y, map_x, map_y)){
                    unsigned char cost = costmap_->getCost(map_x, map_y);
                    if(cost >= 200){
                        return COLLISION_PENALTY;
                    }
                    else if(cost > 0){
                        edge_cost += std::pow((double)cost / 200.0, 2);
                    }
                }
            }
            return edge_cost;
        }

        // Callback untuk menerima graf dari topik /segments dan menyimpannya ke dalam struktur data internal
        void roadmapCallback(const tuw_multi_robot_msgs::Graph::ConstPtr& msg) {
            nodes.clear();
            for(int i = 0; i < msg->vertices.size(); ++i){
                Node n;
                n.id = msg->vertices[i].id;
                n.x = msg->vertices[i].path[0].x - offset_x;
                n.y = msg->vertices[i].path[0].y - offset_y;
                n.neighbours_count = msg->vertices[i].successors.size() + msg->vertices[i].predecessors.size();
                nodes.push_back(n);
            }
            for(int i = 0; i < msg->vertices.size(); ++i){
                for(int j = 0; j < msg->vertices[i].successors.size(); ++j){
                    Edge e;
                    e.id = msg->vertices[i].successors[j];
                    e.weight = dist(&nodes[i], &nodes[e.id]);
                    nodes[i].neighbours.push_back(e);
                }
                for(int j = 0; j < msg->vertices[i].predecessors.size(); ++j){
                    Edge e;
                    e.id = msg->vertices[i].predecessors[j];
                    e.weight = dist(&nodes[i], &nodes[e.id]);
                    nodes[i].neighbours.push_back(e);
                }
            }
            map_received = true;
        }

        // Fungsi untuk menyambungkan titik ke k node terdekat dalam graf
        std::vector<int> topKNodes(std::vector<Node>& current_graph, Coordinate point, int k){
            std::vector<PosNode> posNodes;
            for(int i = 0; i < current_graph.size(); ++i){
                double distance = std::sqrt(std::pow(point.x - current_graph[i].x, 2) + std::pow(point.y - current_graph[i].y, 2));
                PosNode pn; pn.id = i; pn.distance = distance;
                posNodes.push_back(pn);
            }
            std::sort(posNodes.begin(), posNodes.end(), [](const PosNode& a, const PosNode& b) { return a.distance < b.distance; });
            std::vector<int> topKIds;
            for(int i = 0; i < std::min((int)posNodes.size(), k); ++i){
                topKIds.push_back(posNodes[i].id);
            }
            return topKIds;
        }

        // Fungsi untuk menyatukan titik awal dan tujuan ke dalam graf
        void mergeSGG(std::vector<Node>& current_graph){
            std::vector<int> start_candidates = topKNodes(current_graph, real_start, 5); 
            std::vector<int> goal_candidates = topKNodes(current_graph, real_goal, 5);

            Node startNode;
            startNode.id = current_graph.size();
            startNode.x = real_start.x; startNode.y = real_start.y;
            startNode.neighbours_count = 0;
            for(int i = 0; i < start_candidates.size(); ++i){
                Edge e; e.id = start_candidates[i];
                e.weight = std::sqrt(std::pow(current_graph[start_candidates[i]].x - real_start.x, 2) + std::pow(current_graph[start_candidates[i]].y - real_start.y, 2));
                startNode.neighbours.push_back(e);
                startNode.neighbours_count++;
            }
            current_graph.push_back(startNode);

            Node goalNode;
            goalNode.id = current_graph.size();
            goalNode.x = real_goal.x; goalNode.y = real_goal.y;
            goalNode.neighbours_count = 0;
            current_graph.push_back(goalNode);

            for(int i = 0; i < goal_candidates.size(); ++i){
                Node* n = &current_graph[goal_candidates[i]];
                Edge e; e.id = goalNode.id;
                e.weight = std::sqrt(std::pow(n->x - goalNode.x, 2) + std::pow(n->y - goalNode.y, 2));
                n->neighbours.push_back(e);
                n->neighbours_count++;
            }
        }

        // Fungsi untuk menghitung fitness dari partikel PSO
        double calculateFitness(Particle* p, std::vector<Node>& current_graph){
            double totalDistance = 0.0;
            double totalTurnPenalty = 0.0;
            double totalEdgePenalty = 0.0;

            int startNode_i = current_graph.size() - 2;
            int goalNode_i = current_graph.size() - 1;

            std::vector<bool> visited(current_graph.size(), false);
            int currentNode = startNode_i;
            visited[currentNode] = true;
            double currentYaw = start_yaw;
            int steps = 0;
            int max_steps = current_graph.size();

            while(currentNode != goalNode_i && steps < max_steps){
                Node* n = &current_graph[currentNode];
                int bestNode = currentNode;
                double highestValue = -1e9; 
                double tempDist= 0;
                double tempAngleDiff = 0;
                double tempYaw = currentYaw;
                double chosenEdgePenalty = 0.0; 
                bool moved = false;

                for(int i = 0; i < n->neighbours_count; ++i){
                    if(!visited[n->neighbours[i].id]){
                        double raw_priority = p->pos_i[n->neighbours[i].id];
                        double edge_danger = n->neighbours[i].penalty;
                        double modified_priority = raw_priority - (edge_danger * W_EDGE);

                        if(modified_priority > highestValue){
                            Node* neighbourNode = &current_graph[n->neighbours[i].id];
                            double angleToNeighbour = std::atan2(neighbourNode->y - n->y, neighbourNode->x - n->x);
                            double angleDiff = std::abs(std::atan2(std::sin(angleToNeighbour - currentYaw), std::cos(angleToNeighbour - currentYaw)));
                    
                            highestValue = modified_priority;
                            bestNode = n->neighbours[i].id;
                            tempDist = n->neighbours[i].weight;
                            tempAngleDiff = angleDiff;
                            tempYaw = angleToNeighbour;
                            chosenEdgePenalty = edge_danger; 
                            moved = true;
                        }
                    }
                }

                if(!moved) break;

                currentNode = bestNode;
                visited[currentNode] = true;

                totalDistance += tempDist;
                totalTurnPenalty += tempAngleDiff;
                totalEdgePenalty += chosenEdgePenalty; 
                currentYaw = tempYaw;
                steps++;
            }
            if(currentNode == goalNode_i) 
                return (totalDistance * W_DIST) + (totalTurnPenalty * W_TURN) + (totalEdgePenalty * W_EDGE);
            else 
                return (totalDistance * W_DIST) + (totalTurnPenalty * W_TURN) + COLLISION_PENALTY;
        }

        // Fungsi untuk mengembalikan rute akhir dari start ke goal berdasarkan gBest
        std::vector<int> calculateFinale(Global* g, std::vector<Node>& current_graph){
            std::vector<int> path;
            std::vector<bool> visited(current_graph.size(), false);
            int startNode_i = current_graph.size() - 2;
            int goalNode_i = current_graph.size() - 1;

            int currentNode = startNode_i;
            path.push_back(currentNode);
            visited[currentNode] = true; 
            int steps = 0;
            int max_steps = current_graph.size();

            while(currentNode != goalNode_i && steps < max_steps){
                Node* n = &current_graph[currentNode];
                int bestNode = currentNode;
                double highestValue = -1e9;
                bool moved = false;

                for(int i = 0; i < n->neighbours_count; ++i){
                    int neighbour_id = n->neighbours[i].id;
                    if(!visited[neighbour_id]){
                        double raw_priority = g->g_pos[neighbour_id];
                        double edge_danger = n->neighbours[i].penalty;
                        double modified_priority = raw_priority - (edge_danger * W_EDGE);

                        if(modified_priority > highestValue){
                            highestValue = modified_priority;
                            bestNode = neighbour_id;
                            moved = true;
                        }
                    }
                }
                if(!moved){
                    ROS_WARN("PSO failed to converge on a good solution.");
                    break;
                }
                currentNode = bestNode;
                path.push_back(currentNode);
                visited[currentNode] = true;
                steps++;
            }
            return path;
        }

        // Fungsi MLPSO utama 
        bool executePSO(std::vector<Node>& local_graph, int startNode_i, int goalNode_i, std::vector<int>& final_path) {
            std::vector<Particle> swarm(swarm_size);
            Global gBest;

            for(int i = 0; i < swarm_size; ++i){
                swarm[i].pos_i.resize(local_graph.size());
                swarm[i].velo.resize(local_graph.size());
                swarm[i].pBestPos.resize(local_graph.size());
            }
            gBest.g_pos.resize(local_graph.size());

            bool path_found = false;

            for (int retry = 0; retry < max_retries; ++retry) {
                gBest.g_score = 1e9;

                // Inisialisasi partikel
                for(int i = 0; i < swarm_size; ++i){
                    for(int j = 0; j < local_graph.size(); ++j){
                        swarm[i].pos_i[j] = randdbl(0.0, 1.0);
                        swarm[i].velo[j] = 0.0;
                        swarm[i].pBestPos[j] = swarm[i].pos_i[j];
                    }
                    swarm[i].pBest_score = calculateFitness(&swarm[i], local_graph);
                    swarm[i].score = swarm[i].pBest_score;
                    
                    if(swarm[i].score < gBest.g_score){
                        gBest.g_score = swarm[i].score;
                        gBest.g_pos = swarm[i].pos_i;
                    }
                }

                int unchanged_count = 0;
                double prev_gBest_score = gBest.g_score;

                // Loop iterasi PSO
                for(int i = 0; i < max_iter; ++i){

                    for(int j = 0; j < swarm_size; ++j){
                        for(int k = 0; k < local_graph.size(); ++k){
                            double r1 = randdbl(0, 1);
                            double r2 = randdbl(0, 1);
                            swarm[j].velo[k] = w * swarm[j].velo[k] + c1 * (swarm[j].pBestPos[k] - swarm[j].pos_i[k]) * r1 + c2 * (gBest.g_pos[k] - swarm[j].pos_i[k]) * r2;
                            swarm[j].pos_i[k] += swarm[j].velo[k];
                        }
                        
                        swarm[j].score = calculateFitness(&swarm[j], local_graph);

                        if(swarm[j].score < swarm[j].pBest_score){
                            swarm[j].pBest_score = swarm[j].score;
                            swarm[j].pBestPos = swarm[j].pos_i;
                        }

                        if(swarm[j].score < gBest.g_score){
                            gBest.g_score = swarm[j].score;
                            gBest.g_pos = swarm[j].pos_i;
                        }
                    }

                    // Early Stopping
                    if (std::abs(prev_gBest_score - gBest.g_score) < min_improvement) {
                        unchanged_count++;
                    } else {
                        unchanged_count = 0;
                        prev_gBest_score = gBest.g_score;
                    }

                    if (unchanged_count >= patience) {
                        ROS_INFO("PSO converged early at iteration %d (Score stable for %d iters)", i + 1, patience);
                        break; 
                    }
                }

                final_path = calculateFinale(&gBest, local_graph);

                if(!final_path.empty() && final_path.back() == goalNode_i){
                    path_found = true;
                    ROS_WARN("PSO converged on a solution in retry %d.", retry + 1);
                    break;
                } else {
                    ROS_WARN("PSO failed to converge on a solution in retry %d. Retrying...", retry + 1);
                }
            }

            if(!path_found){
                ROS_ERROR("PSO failed to find a valid path after %d retries.", max_retries);
                return false;
            }
            return true;
        }

        // Bezier smoothing function
        void generateSmoothPath(const std::vector<Coordinate>& path_p, std::vector<geometry_msgs::PoseStamped>& plan) {
            if (path_p.size() < 2) return;
            
            double t_step = 0.002;
            int steps = 1.0 / t_step;

            // Interpolasi linear dari titik awal ke titik tengah pertama
            Coordinate p0 = path_p[0];
            Coordinate p1 = path_p[1];
            double mid_x = (p0.x + p1.x) / 2.0;
            double mid_y = (p0.y + p1.y) / 2.0;

            for (int j = 0; j <= steps; ++j){
                double t = j * t_step;
                double u = 1 - t;
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = u * p0.x + t * mid_x;
                pose.pose.position.y = u * p0.y + t * mid_y;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0; 
                plan.push_back(pose);
            }

            // Interpolasi menggunakan kuadratik Bezier untuk keseluruhan jalur
            for (int i = 1; i < path_p.size() - 1; ++i){
                p0 = path_p[i - 1];
                p1 = path_p[i];
                Coordinate p2 = path_p[i + 1];

                double m1_x = (p0.x + p1.x) / 2.0;
                double m1_y = (p0.y + p1.y) / 2.0;
                double m2_x = (p1.x + p2.x) / 2.0;
                double m2_y = (p1.y + p2.y) / 2.0;

                for (int j = 0; j <= steps; ++j){
                    double t = j * t_step;
                    double u = 1 - t;
                    double tt = t * t;
                    double uu = u * u;
                    double uut = 2 * u * t;

                    geometry_msgs::PoseStamped pose;
                    pose.header.frame_id = "map";
                    pose.header.stamp = ros::Time::now();
                    pose.pose.position.x = uu * m1_x + uut * p1.x + tt * m2_x;
                    pose.pose.position.y = uu * m1_y + uut * p1.y + tt * m2_y;
                    pose.pose.position.z = 0.0;
                    pose.pose.orientation.w = 1.0; 
                    plan.push_back(pose);
                }
            }

            // Interpolasi linear dari titik tengah terakhir ke titik tujuan
            p0 = path_p[path_p.size() - 2];
            p1 = path_p.back();
            mid_x = (p0.x + p1.x) / 2.0;
            mid_y = (p0.y + p1.y) / 2.0;

            for (int j = 0; j <= steps; ++j){
                double t = j * t_step;
                double u = 1 - t;
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = u * mid_x + t * p1.x;
                pose.pose.position.y = u * mid_y + t * p1.y;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0; 
                plan.push_back(pose);
            }
        }

    public:
        MLPSO() {}
        MLPSO(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
            initialize(name, costmap_ros);
        }

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
            if(!initialized_){
                ros::NodeHandle private_nh("~/" + name);
                std::srand(std::time(0));

                costmap_ = costmap_ros->getCostmap();

                roadmap_sub_ = private_nh.subscribe("/segments", 10, &MLPSO::roadmapCallback, this);
                plan_pub_ = private_nh.advertise<nav_msgs::Path>("/pso_path", 10, true);
                node_pub_ = private_nh.advertise<nav_msgs::Path>("/pso_nodes", 10, true);

                initialized_ = true;
                ROS_INFO("MLPSO Plugin initialized.");
            }
        }

        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

            if(!initialized_ || !map_received) return false;

            plan.clear();
            real_start.x = start.pose.position.x;
            real_start.y = start.pose.position.y;
            start_yaw = tf::getYaw(start.pose.orientation);
            real_goal.x = goal.pose.position.x;
            real_goal.y = goal.pose.position.y;

            std::vector<Node> local_graph;
            local_graph = nodes;

            // Pemrosesan awal graf (penggabungan titik awal dan tujuan, serta perhitungan edge cost)
            mergeSGG(local_graph);
            for (size_t i = 0; i < local_graph.size(); ++i) {
                for (size_t j = 0; j < local_graph[i].neighbours_count; ++j) {
                    int neighbor_id = local_graph[i].neighbours[j].id;
                    local_graph[i].neighbours[j].penalty = getEdgeCost(&local_graph[i], &local_graph[neighbor_id]);
                }
            }

            int startNode_i = local_graph.size() - 2;
            int goalNode_i = local_graph.size() - 1;
            std::vector<int> final_path;

            // Optimasi dengan PSO
            bool success = executePSO(local_graph, startNode_i, goalNode_i, final_path);
            if (!success) return false;

            // Mempublikasikan node mentah 
            std::vector<Coordinate> path_p;
            nav_msgs::Path raw_node_path;
            raw_node_path.header.frame_id = "map";
            raw_node_path.header.stamp = ros::Time::now();

            for(int i = 0; i < final_path.size(); ++i){
                Coordinate temp;
                temp.x = local_graph[final_path[i]].x;
                temp.y = local_graph[final_path[i]].y;
                path_p.push_back(temp);

                geometry_msgs::PoseStamped pose;
                pose.header = raw_node_path.header;
                pose.pose.position.x = temp.x;
                pose.pose.position.y = temp.y;
                pose.pose.orientation.w = 1.0;
                raw_node_path.poses.push_back(pose);
            }
            node_pub_.publish(raw_node_path);

            // Generate Path Spline
            generateSmoothPath(path_p, plan);

            nav_msgs::Path final_pso_path;
            final_pso_path.header.frame_id = "map";
            final_pso_path.header.stamp = ros::Time::now();
            final_pso_path.poses = plan;
            plan_pub_.publish(final_pso_path);

            return true;
        }
};

}

PLUGINLIB_EXPORT_CLASS(mlpso::MLPSO, nav_core::BaseGlobalPlanner);