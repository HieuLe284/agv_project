#include "include/slam_robot.h"

using std::placeholders::_1;

// ════════════════════════════════════════════════════════════════════════════
//  Constructor
// ════════════════════════════════════════════════════════════════════════════
SlamRobot::SlamRobot() : Node("slam_robot") {
    // ── Callback groups ───────────────────────────────────────────────────
    callback_group_lidar_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // lidar
    callback_group_slam_  =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // slam
    callback_group_frontier_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // frontier

    // ── TF2 ──────────────────────────────────────────────────────────────
    tf_buffer_      = std::make_shared<tf2_ros::Buffer>(this->get_clock());      
    tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // ── Publishers ────────────────────────────────────────────────────────
    pub_cmd_    = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10); // moving

    pub_frontier_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/frontier_markers", rclcpp::QoS(10).transient_local());               // frontier

    pub_map_    = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).transient_local());                             // map

    pub_graph_nodes_ = create_publisher<geometry_msgs::msg::PoseArray>(
        "/slam_robot/graph_nodes", rclcpp::QoS(5).transient_local());          // PoseArray

    pub_graph_edges_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/slam_robot/graph_edges", rclcpp::QoS(5).transient_local());          // MarkeyArray

    pub_loop_closure_event_ = create_publisher<std_msgs::msg::String>(
        "/slam_robot/loop_closure_event", rclcpp::QoS(5).transient_local());   // Loop closure events

    pub_scan_visualization_ = create_publisher<sensor_msgs::msg::LaserScan>(
        "/slam_robot/scan_visualization", rclcpp::QoS(10).transient_local());  // LiDAR scan visualization

    // ── MapBuilder: 32m × 20m grid, resolution 5cm, origin (-17,-10) ─────
    map_builder_ = slam::MapBuilder(0.05, 640, 400, -17.0, -10.0);
    map_builder_.setPublisher(pub_map_);

    // ── Link MapBuilder to SlamGraph ──────────────────────────────────────
    slam_graph_.setMapBuilder(&map_builder_);                                   // Kết nối SlamGraph với MapBuilder
    slam_graph_.init();                                                         // Khởi tạo graph với 1 node anchor (0,0,0)

    // ── Subscriber ────────────────────────────────────────────────────────
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_lidar_;
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/agv_scan", rclcpp::SensorDataQoS(),
        std::bind(&SlamRobot::scanCallback, this, _1), sub_opt);                // LiDAR scan ( 10Hz )

    // ── Timers ────────────────────────────────────────────────────────────
    // 200ms = 5 Hz: get TF pose → slamTimerCallback → publish graph visualization
    timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&SlamRobot::slamTimerCallback, this),
        callback_group_slam_);

    // 200ms = 5 Hz: publish mapBuilderTimerCallback
    map_timer_ = create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&SlamRobot::mapBuilderTimerCallback, this));

    // 500ms = 2 Hz: frontier exploration timer
    frontier_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&SlamRobot::frontierTimerCallback, this),
        callback_group_frontier_);

    RCLCPP_INFO(get_logger(),
        "[SlamRobot] Graph-Based SLAM initialized (Grisetti et al., 2010).");
    RCLCPP_INFO(get_logger(),
        "[SlamRobot] Library: lib/SLAM_Graph_Based | Optimizer: Gauss-Newton");
    RCLCPP_INFO(get_logger(),
        "[SlamRobot] Frontier-Based Exploration initialized (Yamauchi, 1997).");
    RCLCPP_INFO(get_logger(),
        "[SlamRobot] Library: lib/frontier_based | Detector: Wave-Front BFS");
}

// ════════════════════════════════════════════════════════════════════════════
//  ---------------------------- SLAM GRAPH BASED -----------------------------
// ════════════════════════════════════════════════════════════════════════════
// ════════════════════════════════════════════════════════════════════════════
//  Helper: broadcast map→odom TF (identity — map = odom at origin)
// ════════════════════════════════════════════════════════════════════════════
void SlamRobot::broadcastMapOdomTF(rclcpp::Time now)
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = "map";
    tf.child_frame_id = "odom";
    tf.transform.translation.x = 0.0; 
    tf.transform.translation.y = 0.0;  
    tf.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);            
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
}

//  slamTimerCallback — 5 Hz: tính pose trong map frame ( SLAM Thread )
void SlamRobot::slamTimerCallback()
{
    // Lấy pose hiện tại của robot từ TF (odom → base_link)
    broadcastMapOdomTF(this->now());
    double ox, oy, otheta; // odom → base_link
    try{
        auto tf_odom = tf_buffer_->lookupTransform("odom", "base_link", rclcpp::Time());
        ox = tf_odom.transform.translation.x;
        oy = tf_odom.transform.translation.y;
        // Chuyển quaternion sang góc yaw (θ)
        tf2::Quaternion q(tf_odom.transform.rotation.x,
                          tf_odom.transform.rotation.y,
                          tf_odom.transform.rotation.z,
                          tf_odom.transform.rotation.w);
        // Truyền (x, y, θ) vào hàm graphSLAMcall()
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, otheta);
    }
    catch (tf2::TransformException& ex){
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[TF] Cannot get odom→base TF: %s", ex.what());
        return;
    }
    // Thực hiện Graph-Based SLAM
    graphSLAMcall(ox, oy, otheta);
}

// ── Sau optimize: cập nhật map→odom ──────────────────────────
// P_map_odom = P_map_base * inverse(P_odom_base)
void SlamRobot::updateMapOdom(double map_x, double map_y, double map_theta,
                                double odom_x, double odom_y, double odom_theta)
{
    // inverse(odom_base): ( -cos(θ)*x - sin(θ)*y, sin(θ)*x - cos(θ)*y, -θ )
    double inv_x = -cos(odom_theta) * odom_x - sin(odom_theta) * odom_y; // = -cos(θ)*ox - sin(θ)*oy
    double inv_y = sin(odom_theta) * odom_x - cos(odom_theta) * odom_y; // =  sin(θ)*ox - cos(θ)*oy
    // Tích: map_odom = map_base * inv(odom_base)
    map_odom_x = map_x + cos(map_theta) * inv_x - sin(map_theta) * inv_y;
    map_odom_y = map_y + sin(map_theta) * inv_x + cos(map_theta) * inv_y;
    map_odom_theta = normalizeAngle(map_theta - odom_theta);
}

// scanCallback - 10Hz : LiDAR Thread
void SlamRobot::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    broadcastMapOdomTF(this->now());

    // Bắt LiDAR scan để sử dụng vào hàm graphSLAMcall (odometry + loop closure)
    cached_scan_ranges_.assign(scan->ranges.begin(), scan->ranges.end());
    cached_scan_angle_min_       = scan->angle_min;
    cached_scan_angle_increment_ = scan->angle_increment;

    // Log khoảng cách 8 hướng của Lidar ( Front, Left, Right,...)
    double dists[8] = {99, 99, 99, 99, 99, 99, 99, 99};
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double r = scan->ranges[i];
        if (std::isnan(r) || r < 0.12) continue;
        double deg = (scan->angle_min + i * scan->angle_increment) * 180.0 / M_PI;
        while (deg <   0) deg += 360;
        while (deg >= 360) deg -= 360;
        int sector = static_cast<int>((deg + 22.5) / 45.0) % 8;
        if (r < dists[sector]) dists[sector] = r;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "[DIST] F:%.1f FL:%.1f L:%.1f BL:%.1f B:%.1f BR:%.1f R:%.1f FR:%.1f",
        dists[0], dists[1], dists[2], dists[3], dists[4], dists[5], dists[6], dists[7]);

    // Publish scan visualization
    pub_scan_visualization_->publish(*scan);

    // Gọi map_builder 
    static int scan_counter = 0;
    if (++scan_counter % 2 == 0) {
        auto occ = map_builder_.buildOccupancyGrid(this->now());
        if (!map_initialized_) { // nếu map chưa init 
            map_initialized_ = true; 
            RCLCPP_INFO(get_logger(), "[MapBuilder] Map ready: %dx%d @ %.3fm",
                occ.info.width, occ.info.height, occ.info.resolution);
        }
    }

    // Cập nhật occupanc grid từ LiDAR
    try {
        // map->odom may be corrected by Graph SLAM
        auto tf = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(scan->header.stamp));

        // Lấy tọa độ vị trí của robot trên bản đồ 
        double px = tf.transform.translation.x;
        double py = tf.transform.translation.y;
        tf2::Quaternion q(tf.transform.rotation.x,
                          tf.transform.rotation.y,
                          tf.transform.rotation.z,
                          tf.transform.rotation.w);
        double roll, pitch, yaw;

        // Lấy hướng quay của robot (yaw)
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // Lấy dữ liệu khoảng cách từ LiDAR
        std::vector<float> ranges_f(scan->ranges.begin(), scan->ranges.end());

        // Cập nhật occupancy grid
        map_builder_.updateFromRanges(ranges_f, scan->angle_min, scan->angle_increment,
                                      px, py, yaw);
    }
    catch (tf2::TransformException& ex){
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[TF] Cannot update map from scan: %s", ex.what());
    }
}

// ════════════════════════════════════ graphSLAMcall ════════════════════════════════════
//  
//  Thực hiện đầy đủ quy trình Graph-Based SLAM gồm 4 bước:
//  Step 1 — Front-End (Odometry):
//    δx = R_{i-1}^T · (t_i − t_{i-1}),  δθ = normalize(θ_i − θ_{i-1})
//    Gọi addOdometryNode():
//       - Thêm node pose mới x_i vào đồ thị.
//       - Thêm cạnh odometry (i−1, i, z_odom, Ω_odom).
//
//  Step 2 — Front-End (Loop Closure):
//    Proximity filter: ‖t_j − t_i‖ < d_thresh && |θ_j − θ_i| < θ_thresh
//    Scan correlation: C = Σ r_i·r_j / (‖r_i‖·‖r_j‖) > corr_thresh
//    → Gọi addLoopClosures(): Nếu hai scan được xác định là khớp,
//    thêm cạnh đóng vòng lặp (i, j, z_scan, Ω_loop) vào đồ thị.
//
//  Step 3 — Back-End (Gauss-Newton Optimization):
//    x* = argmin Σ_{<i,j>∈E} e_ij^T · Ω_ij · e_ij
//    1. Xây dựng ma trận Hessian H.
//    2. Xây dựng vector gradient b.
//    3. Thiết lập hệ:
//          H · Δξ = −b
//    4. Giải hệ bằng phương pháp khử Gauss (Gaussian Elimination).
//    5. Cập nhật toàn bộ pose trong đồ thị:
//          x ← x ⊞ Δξ
//
//  Step 4 — Map Rebuild (if optimized):
//    clearMap() → updateMapFromNode(i)  với mọi node i trong pose graph (Log-Odds ray-casting)
//    Bản đồ Occupancy Grid được xây dựng lại bằng:
//      - Biểu diễn Log-Odds
//      - Bresenham Ray-Casting
//      - Dữ liệu LiDAR đã lưu trong từng node
// ════════════════════════════════════════════════════════════════════════════
// ════════════════════════════════════════════════════════════════════════════
//  scanCallback — Cache LiDAR data, update map, publish scan viz
// ════════════════════════════════════════════════════════════════════════════
void SlamRobot::graphSLAMcall(double x, double y, double theta) {
    // ── Step 1: Add odometry node ─────────────────────────────────────────
    int new_idx = slam_graph_.addOdometryNode(
        x, y, theta,
        cached_scan_ranges_,
        cached_scan_angle_min_,
        cached_scan_angle_increment_);

    if (new_idx < 0) return;  // Nếu robot chưa di chuyển thì return luôn, không thêm node mới

    // ── Step 2: Detect loop closures ──────────────────────────────────────
    int matched_id = slam_graph_.addLoopClosures(new_idx);
    if (matched_id >= 0) {
        std_msgs::msg::String msg;
        msg.data = "Loop closure: Node[" + std::to_string(matched_id) +
                   "] <-> Node[" + std::to_string(new_idx) +
                   "] | Total: " + std::to_string(slam_graph_.loop_closure_count_);
        pub_loop_closure_event_->publish(msg);
        RCLCPP_INFO(get_logger(), "[SLAM] %s", msg.data.c_str());
    }

    // ── Step 3: Gauss-Newton back-end optimization ────────────────────────
    bool optimized = slam_graph_.optimizeIfNeeded();

    // ── Step 4: Rebuild map from all optimized poses ──────────────────────
    // Xây dựng lại toàn bộ bản đồ từ pose graph sau khi SLAM tối ưu hóa.
    if (optimized) {
        slam_graph_.clearMap(); // Xóa occupancy grid hiện tại
        const int N = slam_graph_.pose_graph.numNodes();
        for (int i = 1; i < N; ++i) {
            const auto& n = slam_graph_.pose_graph.nodes[i];
            std::vector<float> rf(n.scan_ranges.begin(), n.scan_ranges.end());
            
            // Chiếu lại scan này lên bản đồ bằng pose đã được tối ưu hóa bởi Graph SLAM.
            map_builder_.updateFromRanges(rf, n.scan_angle_min, n.scan_angle_increment,
                                          n.x, n.y, n.theta);
        }
        RCLCPP_WARN(get_logger(), "[SLAM] Map rebuilt from %d optimized nodes", N);
    }

    // ── Publish graph visualization (PoseArray + MarkerArray) ─────────────
    const int N = slam_graph_.pose_graph.numNodes();

    // /slam_robot/graph_nodes — PoseArray
    geometry_msgs::msg::PoseArray node_msg;
    node_msg.header.frame_id = "map";
    node_msg.header.stamp    = this->now();
    
    // Chuyển các node trong Pose Graph thành PoseArray.
    for (int i = 0; i < N; ++i) {
        const auto& gn = slam_graph_.pose_graph.nodes[i];
        geometry_msgs::msg::Pose p;
        
        // Gán vị trí node.
        p.position.x = gn.x;
        p.position.y = gn.y;
        p.position.z = 0.05;
        
        // Chuyển góc quay 2D (theta) thành quaternion ROS
        tf2::Quaternion q; 
        q.setRPY(0, 0, gn.theta);

        // Gán quaternion vào pose
        p.orientation.x = q.x(); 
        p.orientation.y = q.y();
        p.orientation.z = q.z(); 
        p.orientation.w = q.w();

        // Thêm pose này vào mảng poses
        node_msg.poses.push_back(p);
    }
    pub_graph_nodes_->publish(node_msg);

    // /slam_robot/graph_edges — MarkerArray
    visualization_msgs::msg::MarkerArray edge_msg;

    visualization_msgs::msg::Marker del_all;
    del_all.header.frame_id = "map";
    del_all.header.stamp    = node_msg.header.stamp;
    del_all.action = visualization_msgs::msg::Marker::DELETEALL;
    edge_msg.markers.push_back(del_all);


    // Hàm lambda để khởi tạo Marker dạng đường thẳng (LINE_LIST)
    visualization_msgs::msg::Marker odom_lines, loop_lines; // Tạo 2 marker
    auto initLine = [&](visualization_msgs::msg::Marker& m,
                        const std::string& ns, int id,
                        float r, float g, float b, float a, float w) // Hàm lambda initLine
    {
        m.header.frame_id = "map"; // Frame tham chiếu
        m.header.stamp    = node_msg.header.stamp; // Timestamp
        m.ns = ns;  
        m.id = id;
        m.type   = visualization_msgs::msg::Marker::LINE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = w;
        m.color.r = r;  
        m.color.g = g;  
        m.color.b = b;  
        m.color.a = a;
    };
    initLine(odom_lines, "odom_edges", 0, 0.3f, 0.3f, 1.0f, 0.6f, 0.02f);
    initLine(loop_lines, "loop_edges", 1, 1.0f, 0.2f, 0.2f, 0.9f, 0.04f);

    // Vẽ edges của Pose Graph
    for (const auto& e : slam_graph_.pose_graph.edges) {
        if (e.from >= N || e.to >= N) continue;
        geometry_msgs::msg::Point p1, p2; // Tạo 2 điểm đầu, điểm cuối

        // Điểm đầu
        p1.x = slam_graph_.pose_graph.nodes[e.from].x;
        p1.y = slam_graph_.pose_graph.nodes[e.from].y; 
        p1.z = 0.02;

        // Điểm cuối 
        p2.x = slam_graph_.pose_graph.nodes[e.to].x;
        p2.y = slam_graph_.pose_graph.nodes[e.to].y; 
        p2.z = 0.02;
        if (e.is_loop) { // Nếu là loop closure
            loop_lines.points.push_back(p1);
            loop_lines.points.push_back(p2);
        } else { // Nếu là odometry
            odom_lines.points.push_back(p1);
            odom_lines.points.push_back(p2);
        }
    }

    //Đưa marker vào MarkerArray
    edge_msg.markers.push_back(odom_lines);
    edge_msg.markers.push_back(loop_lines);
    pub_graph_edges_->publish(edge_msg);
}

// ════════════════════════════════════════════════════════════════════════════
//  mapBuilderTimerCallback — 5 Hz: publish OccupancyGrid
// ════════════════════════════════════════════════════════════════════════════
void SlamRobot::mapBuilderTimerCallback() {
    if (!map_initialized_) return;
    map_builder_.publishMap(this->now());
}

// ════════════════════════════════════════════════════════════════════════════
//  ----------------------------- FRONTIER BASED ------------------------------
// ════════════════════════════════════════════════════════════════════════════
//  Reference: Yamauchi — "A Frontier-Based Approach for Autonomous
//             Exploration" (Proc. IEEE Int. Conf. Robotics & Automation, 1997)
//
//  Nguyên lý hoạt động:
//    1. Robot duy trì occupancy grid M (FREE / OCCUPIED / UNKNOWN)
//    2. Frontier cell = FREE cell có ít nhất 1 neighbor UNKNOWN
//    3. Frontier region = cluster các frontier cell liền kề (BFS)
//    4. Frontier goal = centroid của frontier region tốt nhất
//       (cost function: cân bằng giữa khoảng cách & kích thước)
//    5. Robot di chuyển đến frontier goal → lặp lại
//    6. Khi không còn frontier → exploration hoàn thành
// ════════════════════════════════════════════════════════════════════════════

// frontierTimerCallback — 2 Hz: chạy frontier exploration
void SlamRobot::frontierTimerCallback()
{
    if (!map_initialized_) return;      // Chưa có map → chưa làm gì
    if (!exploration_mode_.load()) return; // Chưa bật exploration

    // ── Lấy pose robot hiện tại (map → base_link) ────────────────────────
    double rx, ry, rtheta;
    try {
        auto tf = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time());
        rx = tf.transform.translation.x;
        ry = tf.transform.translation.y;
        tf2::Quaternion q(tf.transform.rotation.x,
                          tf.transform.rotation.y,
                          tf.transform.rotation.z,
                          tf.transform.rotation.w);
        double roll, pitch;
        tf2::Matrix3x3(q).getRPY(roll, pitch, rtheta);
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[Frontier] Cannot get map→base TF: %s", ex.what());
        return;
    }

    // ── Cập nhật OccupancyGrid mới nhất vào frontier map ─────────────────
    auto occ_grid = map_builder_.buildOccupancyGrid(this->now());
    frontier_explorer_.update(occ_grid);

    // ── Gọi compute để tìm / cập nhật frontier goal ──────────────────────
    Pose2D cur(rx, ry, rtheta);
    slam_exploration(cur);

    // ── Publish frontier markers lên RViz ────────────────────────────────
    const FrontierRegion* best = nullptr;
    double bgx = cached_goal_x_;
    double bgy = cached_goal_y_;
    if (cached_has_goal_) {
        // Tìm best region từ cached_regions_ (region có centroid gần goal nhất)
        double min_dist = 1e9;
        for (const auto& r : cached_regions_) {
            double d = std::hypot(r.centroid_x - bgx, r.centroid_y - bgy);
            if (d < min_dist) {
                min_dist = d;
                best = &r;
            }
        }
    }
    publishFrontierMarkers(cached_regions_, best, cached_robot_x_, cached_robot_y_,
                           cached_goal_x_, cached_goal_y_);
}

// ---- Frontier Exploration ----
void SlamRobot::slam_exploration(Pose2D& cur) {
    if (!exploration_mode_.load()) return;

    // Gọi frontier_explorer_ để tìm frontier goal dựa trên occupancy grid + pose robot
    frontier_explorer_.compute(cur.x, cur.y, cur.theta);

    // Lưu kết quả vào cache để publish markers
    cached_regions_ = frontier_explorer_.getLastRegions();
    cached_robot_x_ = cur.x;
    cached_robot_y_ = cur.y;
    cached_goal_x_  = frontier_explorer_.getGoalX();
    cached_goal_y_  = frontier_explorer_.getGoalY();
    cached_has_goal_ = frontier_explorer_.hasGoal();

    // Log thông tin frontier
    if (cached_has_goal_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[Frontier] Goal: (%.2f, %.2f) | Regions: %zu | Mode: %s",
            cached_goal_x_, cached_goal_y_,
            cached_regions_.size(),
            exploration_mode_.load() ? "EXPLORING" : "IDLE");
    } else {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "[Frontier] No goal available. Regions: %zu", cached_regions_.size());
    }

    // Kiểm tra exploration đã hoàn thành chưa
    if (frontier_explorer_.isDone()) {
        RCLCPP_INFO(get_logger(), "[Frontier] === Exploration DONE ===");
        exploration_mode_.store(false);
    }
}

// ---- Frontier Markers ----
void SlamRobot::publishFrontierMarkers(
    const std::vector<FrontierRegion>& regions, const FrontierRegion* best,
    double robot_x, double robot_y, double goal_x, double goal_y)
{
  visualization_msgs::msg::MarkerArray arr;
  auto now = this->now();

  // Xóa tất cả markers cũ
  visualization_msgs::msg::Marker del_all;
  del_all.header.frame_id = "map"; del_all.header.stamp = now;
  del_all.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(del_all);

  int id = 0;

  // Vẽ từng frontier region dưới dạng SPHERE + TEXT
  for (size_t i = 0; i < regions.size(); ++i) {
    const auto& r = regions[i];
    bool is_best = (best && std::abs(r.centroid_x - best->centroid_x) < 0.01
                        && std::abs(r.centroid_y - best->centroid_y) < 0.01);

    // SPHERE: biểu diễn frontier region centroid
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = "map"; sphere.header.stamp = now;
    sphere.ns = "frontier_nodes"; sphere.id = id++;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position.x = r.centroid_x;
    sphere.pose.position.y = r.centroid_y;
    sphere.pose.position.z = 0.05;
    sphere.pose.orientation.w = 1.0;
    if (is_best) {
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.35;
      sphere.color.r = 1.0f; sphere.color.g = 0.85f;
      sphere.color.b = 0.0f; sphere.color.a = 1.0f;  // Vàng = best
    } else {
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.20;
      sphere.color.r = 0.0f; sphere.color.g = 0.85f;
      sphere.color.b = 1.0f; sphere.color.a = 0.85f;  // Xanh dương
    }
    arr.markers.push_back(sphere);

    // TEXT_VIEW_FACING: label hiển thị tên + kích thước frontier
    visualization_msgs::msg::Marker text;
    text.header.frame_id = "map"; text.header.stamp = now;
    text.ns = "frontier_labels"; text.id = id++;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = r.centroid_x;
    text.pose.position.y = r.centroid_y;
    text.pose.position.z = 0.30;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.18;
    text.color.r = text.color.g = text.color.b = text.color.a = 1.0f;
    text.text = "F" + std::to_string(i) + "(" + std::to_string(r.size()) + "c)";
    if (is_best) text.text = "[BEST] " + text.text;
    arr.markers.push_back(text);
  }

  // ARROW: chỉ hướng từ robot → goal nếu có goal
  if (cached_has_goal_) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "map"; arrow.header.stamp = now;
    arrow.ns = "frontier_path"; arrow.id = id++;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.06; arrow.scale.y = arrow.scale.z = 0.12;
    arrow.color.r = 0.1f; arrow.color.g = 1.0f;
    arrow.color.b = 0.3f; arrow.color.a = 0.9f;  // Xanh lá
    geometry_msgs::msg::Point s, e;
    s.x = robot_x; s.y = robot_y; s.z = 0.05;
    e.x = goal_x;   e.y = goal_y;   e.z = 0.05;
    arrow.points.push_back(s); arrow.points.push_back(e);
    arr.markers.push_back(arrow);
  }

  pub_frontier_markers_->publish(arr);
}

// ════════════════════════════════════════════════════════════════════════════
//  ----------------------------------- A* ------------------------------------
// ════════════════════════════════════════════════════════════════════════════



// ════════════════════════════════════════════════════════════════════════════
//  ----------------------------------- DWA -----------------------------------
// ════════════════════════════════════════════════════════════════════════════



// ════════════════════════════════════════════════════════════════════════════
//  main
// ════════════════════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);                           // Khởi tạo ROS2
    auto node = std::make_shared<SlamRobot>();          // Gọi hàm Constructor
    rclcpp::executors::MultiThreadedExecutor executor;  // Executor đa luồng
    executor.add_node(node);                            // Thêm node vào executor
    executor.spin();                                    // Bắt đầu vòng lặp xử lý callback
    rclcpp::shutdown();                                 // /rosout
    return 0;
}
