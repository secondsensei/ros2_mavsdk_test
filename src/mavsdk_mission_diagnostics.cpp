/**
 * @file mavsdk_mission_diagnostics.cpp
 * @brief MAVSDK Mission Integration Diagnostics for ROS2-PX4 system
 * @author Tanumaya Bhowmik
 *
 * This node demonstrates MAVSDK mission upload and execution while maintaining
 * ROS2 integration patterns from px4_gazebo_test package.
 *
 * Mission Profile:
 * 1. Takeoff to 10m
 * 2. Fly to waypoint 1: (26.755171568042137, -80.24434557941278)
 * 3. Loiter for 10 seconds
 * 4. Fly to waypoint 2: (26.755281141397138, -80.24422286838895)
 * 5. Land at waypoint 2
 *
 * MAVSDK handles mission upload/execution, ROS2 monitors system status
 */

#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>

// PX4 ROS2 messages for monitoring
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

// MAVSDK includes
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <thread>

using namespace std::chrono_literals;
using namespace mavsdk;

enum class MissionPhase
{
    INITIALIZING,
    CONNECTING_MAVSDK,
    WAITING_SYSTEM_READY,
    UPLOADING_MISSION,
    ARMING,
    STARTING_MISSION,
    MISSION_ACTIVE,
    MISSION_PAUSED,
    MISSION_COMPLETED,
    RETURNING_HOME,
    LANDED,
    ERROR_STATE
};

std::string mission_phase_to_string(MissionPhase phase)
{
    switch (phase)
    {
    case MissionPhase::INITIALIZING:
        return "INITIALIZING";
    case MissionPhase::CONNECTING_MAVSDK:
        return "CONNECTING_MAVSDK";
    case MissionPhase::WAITING_SYSTEM_READY:
        return "WAITING_SYSTEM_READY";
    case MissionPhase::UPLOADING_MISSION:
        return "UPLOADING_MISSION";
    case MissionPhase::ARMING:
        return "ARMING";
    case MissionPhase::STARTING_MISSION:
        return "STARTING_MISSION";
    case MissionPhase::MISSION_ACTIVE:
        return "MISSION_ACTIVE";
    case MissionPhase::MISSION_PAUSED:
        return "MISSION_PAUSED";
    case MissionPhase::MISSION_COMPLETED:
        return "MISSION_COMPLETED";
    case MissionPhase::RETURNING_HOME:
        return "RETURNING_HOME";
    case MissionPhase::LANDED:
        return "LANDED";
    case MissionPhase::ERROR_STATE:
        return "ERROR_STATE";
    default:
        return "UNKNOWN";
    }
}

class MAVSDKMissionDiagnostics : public rclcpp::Node
{
public:
    MAVSDKMissionDiagnostics() : Node("mavsdk_mission_diagnostics"),
                                 current_phase_(MissionPhase::INITIALIZING),
                                 mission_started_(false),
                                 system_ready_(false)
    {
        RCLCPP_INFO(this->get_logger(), "=== MAVSDK Mission Diagnostics Node ===");
        RCLCPP_INFO(this->get_logger(), "Integrating MAVSDK mission capabilities with ROS2 monitoring");

        // Initialize mission waypoints (easily configurable for web interface)
        initialize_mission_waypoints();

        // Configure ROS2 QoS for PX4 compatibility (following px4_gazebo_test patterns)
        setup_ros2_monitoring();

        // Initialize MAVSDK in separate thread to avoid blocking ROS2
        mavsdk_thread_ = std::thread(&MAVSDKMissionDiagnostics::initialize_mavsdk, this);

        // Create mission monitoring timer
        mission_timer_ = this->create_wall_timer(1000ms, [this]()
                                                 { this->monitor_mission_progress(); });

        // Create status reporting timer (following px4_gazebo_test pattern)
        status_timer_ = this->create_wall_timer(5000ms, [this]()
                                                { this->print_status_report(); });

        start_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Node initialized. Starting MAVSDK connection...");
    }

    ~MAVSDKMissionDiagnostics()
    {
        if (mavsdk_thread_.joinable())
        {
            mavsdk_thread_.join();
        }
    }

private:
    void initialize_mission_waypoints()
    {
        // Mission waypoints (easily configurable for future web interface)
        waypoint1_lat_ = 26.755171568042137;
        waypoint1_lon_ = -80.24434557941278;
        waypoint2_lat_ = 26.755281141397138;
        waypoint2_lon_ = -80.24422286838895;
        mission_altitude_ = 10.0f;
        loiter_time_seconds_ = 10;
        cruise_speed_ = 5.0f;
        approach_speed_ = 3.0f;

        RCLCPP_INFO(this->get_logger(), "Mission waypoints initialized:");
        RCLCPP_INFO(this->get_logger(), "  Waypoint 1: %.6f, %.6f", waypoint1_lat_, waypoint1_lon_);
        RCLCPP_INFO(this->get_logger(), "  Waypoint 2: %.6f, %.6f", waypoint2_lat_, waypoint2_lon_);
        RCLCPP_INFO(this->get_logger(), "  Altitude: %.1fm, Loiter: %ds", mission_altitude_, loiter_time_seconds_);
    }
    void setup_ros2_monitoring()
    {
        // Use PX4-compatible QoS settings (from px4_gazebo_test)
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        RCLCPP_INFO(this->get_logger(), "Setting up ROS2 monitoring with PX4-compatible QoS...");

        // Monitor vehicle status (critical for mission monitoring)
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", qos,
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg)
            {
                this->vehicle_status_callback(msg);
            });

        // Monitor global position (for mission waypoint tracking)
        global_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            "/fmu/out/vehicle_global_position", qos,
            [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
            {
                this->global_position_callback(msg);
            });

        // Monitor local position
        local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position_v1", qos,
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
            {
                this->local_position_callback(msg);
            });

        // Monitor vehicle command acknowledgments for mission feedback
        vehicle_command_ack_sub_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
            "/fmu/out/vehicle_command_ack", qos,
            [this](const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
            {
                this->vehicle_command_ack_callback(msg);
            });

        // Monitor land detection for mission completion confirmation
        vehicle_land_detected_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
            "/fmu/out/vehicle_land_detected", qos,
            [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
            {
                this->vehicle_land_detected_callback(msg);
            });

        // Note: We're using MAVSDK for mission management, so mission progress
        // comes from MAVSDK callbacks rather than PX4 MissionResult messages

        RCLCPP_INFO(this->get_logger(), "✓ ROS2 monitoring subscribers created");
    }

    void initialize_mavsdk()
    {
        current_phase_ = MissionPhase::CONNECTING_MAVSDK;
        RCLCPP_INFO(this->get_logger(), "Initializing MAVSDK connection...");

        try
        {
            // Initialize MAVSDK exactly like the official example
            mavsdk_ = std::make_unique<Mavsdk>(Mavsdk::Configuration{ComponentType::GroundStation});

            // Connect using the same pattern as fly_mission.cpp
            std::string connection_url = "udpin://0.0.0.0:14540"; // Official example format
            ConnectionResult connection_result = mavsdk_->add_any_connection(connection_url);

            if (connection_result != ConnectionResult::Success)
            {
                RCLCPP_ERROR(this->get_logger(), "Connection failed: %s",
                             connection_result_to_string(connection_result).c_str());
                current_phase_ = MissionPhase::ERROR_STATE;
                return;
            }

            RCLCPP_INFO(this->get_logger(), "MAVSDK connection established");

            // Wait for system discovery exactly like official example
            current_phase_ = MissionPhase::WAITING_SYSTEM_READY;
            auto system = mavsdk_->first_autopilot(3.0); // Same 3 second timeout as official
            if (!system)
            {
                RCLCPP_ERROR(this->get_logger(), "Timed out waiting for system");
                current_phase_ = MissionPhase::ERROR_STATE;
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Autopilot system discovered");

            // Initialize plugins exactly like official example
            action_ = std::make_unique<Action>(system.value());
            mission_ = std::make_unique<Mission>(system.value());
            telemetry_ = std::make_unique<Telemetry>(system.value());

            // Simple health check exactly like official example
            RCLCPP_INFO(this->get_logger(), "Waiting for system to be ready");
            while (!telemetry_->health_all_ok())
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Waiting for system to be ready");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            system_ready_ = true;
            RCLCPP_INFO(this->get_logger(), "System ready");

            // Create and upload mission
            create_and_upload_mission();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "MAVSDK initialization failed: %s", e.what());
            current_phase_ = MissionPhase::ERROR_STATE;
        }
    }

    void create_and_upload_mission()
    {
        current_phase_ = MissionPhase::UPLOADING_MISSION;
        RCLCPP_INFO(this->get_logger(), "Creating and uploading mission");

        try
        {
            std::vector<Mission::MissionItem> mission_items;

            // Mission Plan (using configurable waypoints):
            // 1. Takeoff to mission_altitude_ (handled automatically by PX4)
            // 2. Fly to waypoint 1 and loiter for loiter_time_seconds_
            // 3. Fly to waypoint 2
            // 4. Land at waypoint 2

            // Waypoint 1: Fly to first location and loiter
            mission_items.push_back(make_mission_item(
                waypoint1_lat_,    // configurable latitude
                waypoint1_lon_,    // configurable longitude
                mission_altitude_, // configurable altitude
                cruise_speed_,     // configurable cruise speed
                false,             // not fly-through (hover/loiter)
                0.0f,              // gimbal pitch
                0.0f,              // gimbal yaw
                Mission::MissionItem::CameraAction::None));

            // Waypoint 2: Fly to second location
            mission_items.push_back(make_mission_item(
                waypoint2_lat_,    // configurable latitude
                waypoint2_lon_,    // configurable longitude
                mission_altitude_, // configurable altitude
                approach_speed_,   // configurable approach speed
                false,             // hover at destination
                0.0f,
                0.0f,
                Mission::MissionItem::CameraAction::None));

            // Upload mission exactly like official example
            RCLCPP_INFO(this->get_logger(), "Uploading mission...");
            Mission::MissionPlan mission_plan{};
            mission_plan.mission_items = mission_items;
            const Mission::Result upload_result = mission_->upload_mission(mission_plan);

            if (upload_result != Mission::Result::Success)
            {
                RCLCPP_ERROR(this->get_logger(), "Mission upload failed: %s, exiting",
                             mission_result_to_string(upload_result).c_str());
                current_phase_ = MissionPhase::ERROR_STATE;
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Mission uploaded successfully");
            RCLCPP_INFO(this->get_logger(), "Mission plan: Takeoff -> WP1(%.6f, %.6f) -> Loiter %ds -> WP2(%.6f, %.6f) -> Land",
                        waypoint1_lat_, waypoint1_lon_, loiter_time_seconds_,
                        waypoint2_lat_, waypoint2_lon_);

            // Start mission execution
            start_mission_execution();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Mission creation failed: %s", e.what());
            current_phase_ = MissionPhase::ERROR_STATE;
        }
    }

    void start_mission_execution()
    {
        current_phase_ = MissionPhase::ARMING;
        RCLCPP_INFO(this->get_logger(), "Arming...");

        try
        {
            // Arm exactly like official example
            const Action::Result arm_result = action_->arm();
            if (arm_result != Action::Result::Success)
            {
                RCLCPP_ERROR(this->get_logger(), "Arming failed: %s",
                             action_result_to_string(arm_result).c_str());
                current_phase_ = MissionPhase::ERROR_STATE;
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Armed");

            // Subscribe to mission progress to handle loiter timing
            mission_->subscribe_mission_progress([this](Mission::MissionProgress mission_progress)
                                                 {
                RCLCPP_INFO(this->get_logger(), "Mission status update: %d / %d",
                           mission_progress.current, mission_progress.total);
                
                // Handle loiter at waypoint 1 (when current == 1, we're at first waypoint)
                static bool loiter_started = false;
                static std::chrono::steady_clock::time_point loiter_start_time;
                
                if (mission_progress.current == 1 && !loiter_started) {
                    RCLCPP_INFO(this->get_logger(), "Reached waypoint 1, starting %d-second loiter...", loiter_time_seconds_);
                    loiter_started = true;
                    loiter_start_time = std::chrono::steady_clock::now();
                }
                
                if (loiter_started && mission_progress.current == 1) {
                    auto loiter_duration = std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::steady_clock::now() - loiter_start_time);
                    
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "Loitering... %ld/%d seconds", loiter_duration.count(), loiter_time_seconds_);
                    
                    if (loiter_duration.count() >= loiter_time_seconds_) {
                        RCLCPP_INFO(this->get_logger(), "Loiter complete, continuing to waypoint 2");
                        loiter_started = false;
                        // Mission will automatically continue to next waypoint
                    }
                }
                
                // Check if mission is complete
                if (mission_progress.current == mission_progress.total) {
                    current_phase_ = MissionPhase::MISSION_COMPLETED;
                    RCLCPP_INFO(this->get_logger(), "Mission completed, initiating landing...");
                    
                    // Land at current position (waypoint 2)
                    std::thread([this]() {
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        const Action::Result land_result = action_->land();
                        if (land_result == Action::Result::Success) {
                            RCLCPP_INFO(this->get_logger(), "Landing initiated");
                            current_phase_ = MissionPhase::LANDED;
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Landing command failed: %s", 
                                       action_result_to_string(land_result).c_str());
                        }
                    }).detach();
                } });

            current_phase_ = MissionPhase::STARTING_MISSION;
            RCLCPP_INFO(this->get_logger(), "Starting mission...");

            // Start mission exactly like official example
            Mission::Result start_mission_result = mission_->start_mission();
            if (start_mission_result != Mission::Result::Success)
            {
                RCLCPP_ERROR(this->get_logger(), "Starting mission failed: %s",
                             mission_result_to_string(start_mission_result).c_str());
                current_phase_ = MissionPhase::ERROR_STATE;
                return;
            }

            mission_started_ = true;
            current_phase_ = MissionPhase::MISSION_ACTIVE;
            RCLCPP_INFO(this->get_logger(), "Mission started");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Mission execution failed: %s", e.what());
            current_phase_ = MissionPhase::ERROR_STATE;
        }
    }

    Mission::MissionItem make_mission_item(
        double latitude_deg,
        double longitude_deg,
        float relative_altitude_m,
        float speed_m_s,
        bool is_fly_through,
        float gimbal_pitch_deg,
        float gimbal_yaw_deg,
        Mission::MissionItem::CameraAction camera_action)
    {
        Mission::MissionItem new_item{};
        new_item.latitude_deg = latitude_deg;
        new_item.longitude_deg = longitude_deg;
        new_item.relative_altitude_m = relative_altitude_m;
        new_item.speed_m_s = speed_m_s;
        new_item.is_fly_through = is_fly_through;
        new_item.gimbal_pitch_deg = gimbal_pitch_deg;
        new_item.gimbal_yaw_deg = gimbal_yaw_deg;
        new_item.camera_action = camera_action;
        return new_item;
    }

    // ROS2 Callback functions (following px4_gazebo_test patterns)
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        latest_vehicle_status_ = msg;

        // Log significant state changes
        static uint8_t last_nav_state = 255;
        static uint8_t last_arming_state = 255;

        if (msg->nav_state != last_nav_state)
        {
            RCLCPP_INFO(this->get_logger(), "Navigation state: %s",
                        get_nav_state_string(msg->nav_state).c_str());
            last_nav_state = msg->nav_state;
        }

        if (msg->arming_state != last_arming_state)
        {
            RCLCPP_INFO(this->get_logger(), "Arming state: %s",
                        get_arm_state_string(msg->arming_state).c_str());
            last_arming_state = msg->arming_state;
        }
    }

    void global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
    {
        latest_global_position_ = msg;
    }

    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        latest_local_position_ = msg;
    }

    void vehicle_command_ack_callback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
    {
        latest_vehicle_command_ack_ = msg;

        // Log command acknowledgments (useful for mission commands)
        // Use the actual field names from VehicleCommandAck message
        if (msg->result == 0) // Assuming 0 = ACCEPTED based on MAVLink standards
        {
            RCLCPP_DEBUG(this->get_logger(), "✅ Vehicle command accepted (cmd=%u)", msg->command);
        }
        else if (msg->result == 4) // Assuming 4 = DENIED based on MAVLink standards
        {
            RCLCPP_WARN(this->get_logger(), "❌ Vehicle command denied (cmd=%u)", msg->command);
        }
        else if (msg->result == 3) // Assuming 3 = FAILED based on MAVLink standards
        {
            RCLCPP_WARN(this->get_logger(), "❌ Vehicle command failed (cmd=%u)", msg->command);
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "📋 Vehicle command result=%u (cmd=%u)", msg->result, msg->command);
        }
    }

    void vehicle_land_detected_callback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
    {
        latest_vehicle_land_detected_ = msg;

        // Log landing detection for mission completion
        static bool last_landed_state = false;
        bool currently_landed = msg->landed;

        if (currently_landed != last_landed_state)
        {
            if (currently_landed)
            {
                RCLCPP_INFO(this->get_logger(), "🛬 Vehicle landed detected");
                if (current_phase_ == MissionPhase::MISSION_COMPLETED)
                {
                    current_phase_ = MissionPhase::LANDED;
                    RCLCPP_INFO(this->get_logger(), "✅ Mission completed successfully - vehicle landed");
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "🚁 Vehicle airborne detected");
            }
            last_landed_state = currently_landed;
        }
    }

    void mission_progress_callback(Mission::MissionProgress progress)
    {
        RCLCPP_INFO(this->get_logger(), "📍 Mission progress: %d / %d waypoints",
                    progress.current, progress.total);

        // Handle loiter logic at waypoint 1 (index 1, second item)
        static bool loiter_started = false;
        static std::chrono::steady_clock::time_point loiter_start_time;

        if (progress.current == 2 && !loiter_started)
        {
            RCLCPP_INFO(this->get_logger(), "⏳ Starting 10-second loiter at waypoint 1...");
            loiter_started = true;
            loiter_start_time = std::chrono::steady_clock::now();
        }

        if (loiter_started && progress.current == 2)
        {
            auto loiter_duration = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - loiter_start_time);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "🔄 Loitering... %ld/10 seconds", loiter_duration.count());

            if (loiter_duration.count() >= 10)
            {
                RCLCPP_INFO(this->get_logger(), "✅ Loiter complete, continuing mission...");
                loiter_started = false;
            }
        }

        // Check if mission is complete
        if (progress.current == progress.total)
        {
            current_phase_ = MissionPhase::MISSION_COMPLETED;
            RCLCPP_INFO(this->get_logger(), "🏁 Mission completed! Initiating landing...");

            // Land at current position (waypoint 2)
            std::thread([this]()
                        {
                std::this_thread::sleep_for(std::chrono::seconds(2));
                const Action::Result land_result = action_->land();
                if (land_result == Action::Result::Success) {
                    RCLCPP_INFO(this->get_logger(), "🛬 Landing initiated");
                    current_phase_ = MissionPhase::LANDED;
                } else {
                    RCLCPP_WARN(this->get_logger(), "⚠️ Landing command failed: %s", 
                               action_result_to_string(land_result).c_str());
                } })
                .detach();
        }
    }

    void monitor_mission_progress()
    {
        if (!mission_ || !system_ready_)
            return;

        // Check mission status exactly like official example
        if (mission_started_ && current_phase_ == MissionPhase::MISSION_ACTIVE)
        {
            try
            {
                auto mission_finished = mission_->is_mission_finished();
                if (mission_finished.first == Mission::Result::Success && mission_finished.second)
                {
                    if (current_phase_ != MissionPhase::MISSION_COMPLETED)
                    {
                        current_phase_ = MissionPhase::MISSION_COMPLETED;
                        RCLCPP_INFO(this->get_logger(), "Mission finished, commanding RTL...");

                        // RTL exactly like official example
                        std::thread([this]()
                                    {
                            std::this_thread::sleep_for(std::chrono::seconds(1));
                            const Action::Result rtl_result = action_->return_to_launch();
                            if (rtl_result != Action::Result::Success) {
                                RCLCPP_WARN(this->get_logger(), "Failed to command RTL: %s", 
                                           action_result_to_string(rtl_result).c_str());
                            } else {
                                RCLCPP_INFO(this->get_logger(), "Commanded RTL");
                                current_phase_ = MissionPhase::RETURNING_HOME;
                            } })
                            .detach();
                    }
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Mission status check failed: %s", e.what());
            }
        }
    }

    void print_status_report()
    {
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_time_);

        RCLCPP_INFO(this->get_logger(), "\n=== MAVSDK Mission Diagnostics Status ===");
        RCLCPP_INFO(this->get_logger(), "Runtime: %ld seconds", duration.count());
        RCLCPP_INFO(this->get_logger(), "Mission Phase: %s", mission_phase_to_string(current_phase_).c_str());
        RCLCPP_INFO(this->get_logger(), "System Ready: %s", system_ready_ ? "✅ YES" : "❌ NO");
        RCLCPP_INFO(this->get_logger(), "Mission Started: %s", mission_started_ ? "✅ YES" : "❌ NO");

        // Print vehicle status if available
        if (latest_vehicle_status_)
        {
            RCLCPP_INFO(this->get_logger(), "Vehicle State: %s | %s",
                        get_nav_state_string(latest_vehicle_status_->nav_state).c_str(),
                        get_arm_state_string(latest_vehicle_status_->arming_state).c_str());
        }

        // Print position if available
        if (latest_global_position_)
        {
            RCLCPP_INFO(this->get_logger(), "Position: lat=%.6f°, lon=%.6f°, alt=%.1fm",
                        latest_global_position_->lat, latest_global_position_->lon,
                        latest_global_position_->alt);
        }

        // Connection status summary
        bool mavsdk_ok = (mavsdk_ && system_ready_);
        bool ros2_ok = (latest_vehicle_status_ != nullptr);

        RCLCPP_INFO(this->get_logger(), "Connections: MAVSDK=%s | ROS2=%s",
                    mavsdk_ok ? "✅" : "❌", ros2_ok ? "✅" : "❌");

        if (current_phase_ == MissionPhase::ERROR_STATE)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ SYSTEM IN ERROR STATE - Check logs above");
        }

        RCLCPP_INFO(this->get_logger(), "============================================\n");
    }

    // Helper functions (from px4_gazebo_test patterns)
    std::string get_nav_state_string(uint8_t nav_state)
    {
        switch (nav_state)
        {
        case 0:
            return "MANUAL";
        case 1:
            return "ALTCTL";
        case 2:
            return "POSCTL";
        case 3:
            return "AUTO_MISSION";
        case 4:
            return "AUTO_LOITER";
        case 5:
            return "AUTO_RTL";
        case 14:
            return "OFFBOARD";
        case 17:
            return "AUTO_TAKEOFF";
        case 18:
            return "AUTO_LAND";
        default:
            return "UNKNOWN(" + std::to_string(nav_state) + ")";
        }
    }

    std::string get_arm_state_string(uint8_t arm_state)
    {
        switch (arm_state)
        {
        case 1:
            return "DISARMED";
        case 2:
            return "ARMED";
        default:
            return "UNKNOWN(" + std::to_string(arm_state) + ")";
        }
    }

    std::string connection_result_to_string(ConnectionResult result)
    {
        switch (result)
        {
        case ConnectionResult::Success:
            return "SUCCESS";
        case ConnectionResult::Timeout:
            return "TIMEOUT";
        case ConnectionResult::SocketError:
            return "SOCKET_ERROR";
        case ConnectionResult::BindError:
            return "BIND_ERROR";
        case ConnectionResult::SocketConnectionError:
            return "SOCKET_CONNECTION_ERROR";
        case ConnectionResult::ConnectionError:
            return "CONNECTION_ERROR";
        case ConnectionResult::NotImplemented:
            return "NOT_IMPLEMENTED";
        case ConnectionResult::SystemNotConnected:
            return "SYSTEM_NOT_CONNECTED";
        case ConnectionResult::SystemBusy:
            return "SYSTEM_BUSY";
        case ConnectionResult::CommandDenied:
            return "COMMAND_DENIED";
        case ConnectionResult::DestinationIpUnknown:
            return "DESTINATION_IP_UNKNOWN";
        case ConnectionResult::ConnectionsExhausted:
            return "CONNECTIONS_EXHAUSTED";
        case ConnectionResult::ConnectionUrlInvalid:
            return "CONNECTION_URL_INVALID";
        case ConnectionResult::BaudrateUnknown:
            return "BAUDRATE_UNKNOWN";
        default:
            return "UNKNOWN";
        }
    }

    std::string mission_result_to_string(Mission::Result result)
    {
        switch (result)
        {
        case Mission::Result::Success:
            return "SUCCESS";
        case Mission::Result::Error:
            return "ERROR";
        case Mission::Result::TooManyMissionItems:
            return "TOO_MANY_MISSION_ITEMS";
        case Mission::Result::Busy:
            return "BUSY";
        case Mission::Result::Timeout:
            return "TIMEOUT";
        case Mission::Result::InvalidArgument:
            return "INVALID_ARGUMENT";
        case Mission::Result::Unsupported:
            return "UNSUPPORTED";
        case Mission::Result::NoMissionAvailable:
            return "NO_MISSION_AVAILABLE";
        case Mission::Result::TransferCancelled:
            return "TRANSFER_CANCELLED";
        default:
            return "UNKNOWN";
        }
    }

    std::string action_result_to_string(Action::Result result)
    {
        switch (result)
        {
        case Action::Result::Success:
            return "SUCCESS";
        case Action::Result::NoSystem:
            return "NO_SYSTEM";
        case Action::Result::ConnectionError:
            return "CONNECTION_ERROR";
        case Action::Result::Busy:
            return "BUSY";
        case Action::Result::CommandDenied:
            return "COMMAND_DENIED";
        case Action::Result::CommandDeniedLandedStateUnknown:
            return "COMMAND_DENIED_LANDED_STATE_UNKNOWN";
        case Action::Result::CommandDeniedNotLanded:
            return "COMMAND_DENIED_NOT_LANDED";
        case Action::Result::Timeout:
            return "TIMEOUT";
        case Action::Result::VtolTransitionSupportUnknown:
            return "VTOL_TRANSITION_SUPPORT_UNKNOWN";
        case Action::Result::NoVtolTransitionSupport:
            return "NO_VTOL_TRANSITION_SUPPORT";
        case Action::Result::ParameterError:
            return "PARAMETER_ERROR";
        case Action::Result::Unsupported:
            return "UNSUPPORTED";
        case Action::Result::Failed:
            return "FAILED";
        default:
            return "UNKNOWN";
        }
    }

    // ROS2 components
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_detected_sub_;

    rclcpp::TimerBase::SharedPtr mission_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // MAVSDK components
    std::unique_ptr<Mavsdk> mavsdk_;
    std::unique_ptr<Action> action_;
    std::unique_ptr<Mission> mission_;
    std::unique_ptr<Telemetry> telemetry_;
    std::thread mavsdk_thread_;

    // State tracking
    std::atomic<MissionPhase> current_phase_;
    std::atomic<bool> mission_started_;
    std::atomic<bool> system_ready_;
    std::chrono::steady_clock::time_point start_time_;

    // Latest ROS2 messages (following px4_gazebo_test patterns)
    px4_msgs::msg::VehicleStatus::SharedPtr latest_vehicle_status_;
    px4_msgs::msg::VehicleGlobalPosition::SharedPtr latest_global_position_;
    px4_msgs::msg::VehicleLocalPosition::SharedPtr latest_local_position_;
    px4_msgs::msg::VehicleCommandAck::SharedPtr latest_vehicle_command_ack_;
    px4_msgs::msg::VehicleLandDetected::SharedPtr latest_vehicle_land_detected_;

    // Mission configuration (easily updatable for web interface)
    double waypoint1_lat_;
    double waypoint1_lon_;
    double waypoint2_lat_;
    double waypoint2_lon_;
    float mission_altitude_;
    int loiter_time_seconds_;
    float cruise_speed_;
    float approach_speed_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MAVSDKMissionDiagnostics>();

    RCLCPP_INFO(node->get_logger(), "MAVSDK Mission Diagnostics Node started.");
    RCLCPP_INFO(node->get_logger(), "Mission: Takeoff → Waypoint 1 → Loiter 10s → Waypoint 2 → Land");
    RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to stop.");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}