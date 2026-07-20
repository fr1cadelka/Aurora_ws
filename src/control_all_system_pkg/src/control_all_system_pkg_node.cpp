#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/gimbal_manager_set_pitchyaw.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/gimbal_manager_pitchyaw.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <algorithm>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <sstream>

using namespace std::chrono_literals;

int getch_nonblock() {
    static struct termios oldt, newt;
    static bool initialized = false;

    if (!initialized) {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        initialized = true;
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    int ch = getchar();
    fcntl(STDIN_FILENO, F_SETFL, flags);
    if (ch == EOF) return -1;
    return ch;
}

class UnifiedController : public rclcpp::Node {
public:
    UnifiedController() : Node("unified_controller"), running_(true) {
        // Параметры тележки
        this->declare_parameter<int>("manual_speed_step", 50);
        this->declare_parameter<int>("cruise_speed_pwm", 1500);
        this->declare_parameter<int>("cruise_speed_pwm_backward", 1100);
        this->declare_parameter<int>("steering_left_pwm", 1350);
        this->declare_parameter<int>("steering_right_pwm", 1650);
        this->declare_parameter<int>("neutral_pwm", 1500);
        this->declare_parameter<std::string>("telega_cmd_topic", "/telega_commands");
        this->declare_parameter<std::string>("status_topic", "/telega_status");

        this->declare_parameter<int>("tcp_timeout_ms", 800);
        this->declare_parameter<int>("keyboard_timeout_ms", 200);

        manual_speed_step_ = this->get_parameter("manual_speed_step").as_int();
        cruise_speed_pwm_ = this->get_parameter("cruise_speed_pwm").as_int();
        cruise_speed_pwm_backward_ = this->get_parameter("cruise_speed_pwm_backward").as_int();
        steering_left_pwm_ = this->get_parameter("steering_left_pwm").as_int();
        steering_right_pwm_ = this->get_parameter("steering_right_pwm").as_int();
        neutral_pwm_ = this->get_parameter("neutral_pwm").as_int();
        telega_cmd_topic_ = this->get_parameter("telega_cmd_topic").as_string();
        status_topic_ = this->get_parameter("status_topic").as_string();

        tcp_timeout_ms_ = this->get_parameter("tcp_timeout_ms").as_int();
        keyboard_timeout_ms_ = this->get_parameter("keyboard_timeout_ms").as_int();

        // Параметры камеры
        this->declare_parameter<std::string>("gimbal_topic", "/gimbal/commands");
        this->declare_parameter<std::string>("angle_publish_topic", "/gimbal/angles");
        this->declare_parameter<std::string>("gimbal_service", "/mavros/gimbal_control/manager/pitchyaw");
        this->declare_parameter<int>("gimbal_device_id", 1);
        this->declare_parameter<int>("flags", 0);
        this->declare_parameter<double>("step_angle", 2.0);
        this->declare_parameter<double>("max_pitch", 25.0);
        this->declare_parameter<double>("min_pitch", -90.0);
        this->declare_parameter<double>("max_yaw", 180.0);
        this->declare_parameter<double>("min_yaw", -180.0);
        this->declare_parameter<double>("zoom_speed", 0.2);
        this->declare_parameter<std::string>("camera_ip", "192.168.144.25");
        this->declare_parameter<int>("camera_port", 14550);
        this->declare_parameter<int>("vdisp_rgb", 3);
        this->declare_parameter<int>("vdisp_wide_thermal", 5);
        this->declare_parameter<std::string>("camera_cmd_topic", "/camera_control");

        gimbal_topic_ = this->get_parameter("gimbal_topic").as_string();
        angle_publish_topic_ = this->get_parameter("angle_publish_topic").as_string();
        gimbal_service_ = this->get_parameter("gimbal_service").as_string();
        gimbal_device_id_ = this->get_parameter("gimbal_device_id").as_int();
        flags_ = this->get_parameter("flags").as_int();
        step_ = this->get_parameter("step_angle").as_double();
        max_pitch_ = this->get_parameter("max_pitch").as_double();
        min_pitch_ = this->get_parameter("min_pitch").as_double();
        max_yaw_ = this->get_parameter("max_yaw").as_double();
        min_yaw_ = this->get_parameter("min_yaw").as_double();
        zoom_speed_ = this->get_parameter("zoom_speed").as_double();
        camera_ip_ = this->get_parameter("camera_ip").as_string();
        camera_port_ = this->get_parameter("camera_port").as_int();
        vdisp_rgb_ = this->get_parameter("vdisp_rgb").as_int();
        vdisp_wide_thermal_ = this->get_parameter("vdisp_wide_thermal").as_int();
        camera_cmd_topic_ = this->get_parameter("camera_cmd_topic").as_string();

        // Инициализация тележки
        is_connected_ = false;
        is_armed_ = false;
        linear_throttle_ = neutral_pwm_;
        angular_steering_ = neutral_pwm_;
        cruise_control_active_ = false;
        cruise_control_backward_ = false;
        last_throttle_press_time_ = this->now();
        last_steering_press_time_ = this->now();

        is_tcp_mode_ = false;
        last_tcp_command_time_ = this->now();

        // Подписчики
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            std::bind(&UnifiedController::stateCallback, this, std::placeholders::_1));

        telega_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            telega_cmd_topic_, 10,
            std::bind(&UnifiedController::telegaCmdCallback, this, std::placeholders::_1));

        camera_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            camera_cmd_topic_, 10,
            std::bind(&UnifiedController::cameraCmdCallback, this, std::placeholders::_1));

        // Публикаторы
        gimbal_pub_ = this->create_publisher<mavros_msgs::msg::GimbalManagerSetPitchyaw>(gimbal_topic_, 10);
        angle_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(angle_publish_topic_, 10);
        stream_pub_ = this->create_publisher<std_msgs::msg::Int32>("/set_rtsp_stream", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>(status_topic_, 10);

        // Клиенты сервисов
        command_client_ = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");
        gimbal_client_ = this->create_client<mavros_msgs::srv::GimbalManagerPitchyaw>(gimbal_service_);

        // Запускаем поток для чтения клавиш
        keyboard_thread_ = std::thread(&UnifiedController::keyboardReader, this);

        // Таймеры
        telega_timer_ = this->create_wall_timer(50ms, std::bind(&UnifiedController::telegaControlLoop, this));
        process_queue_timer_ = this->create_wall_timer(10ms, std::bind(&UnifiedController::processKeyQueue, this));
        status_timer_ = this->create_wall_timer(100ms, std::bind(&UnifiedController::publishStatus, this));

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "ОБЪЕДИНЕННЫЙ КОНТРОЛЛЕР ЗАПУЩЕН");
        RCLCPP_INFO(this->get_logger(), "🚜 Тележка: %s", telega_cmd_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "📷 Камера: %s", camera_cmd_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "📊 Статус: %s", status_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "⚙️ TCP таймаут: %d мс, клавиатура: %d мс", tcp_timeout_ms_, keyboard_timeout_ms_);
        RCLCPP_INFO(this->get_logger(), "========================================");
        printHelp();
    }

    ~UnifiedController() {
        running_ = false;
        queue_cv_.notify_all();
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
    }

private:
    int getCruiseSpeed() {
        return neutral_pwm_ + manual_speed_step_;
    }

    int getCruiseSpeedBackward() {
        return neutral_pwm_ - manual_speed_step_;
    }

    void printHelp() {
        std::cout << "\n=== УПРАВЛЕНИЕ ===" << std::endl;
        std::cout << "🚜 Тележка: W/S - вперед/назад, A/D - влево/вправо" << std::endl;
        std::cout << "   Q - круиз вперед, E - круиз назад, R/F - скорость" << std::endl;
        std::cout << "   SPACE - стоп" << std::endl;
        std::cout << "📷 Камера: СТРЕЛКИ - наклон/поворот" << std::endl;
        std::cout << "   Z/X/C - зум +/-/стоп, 1/2 - видеопоток" << std::endl;
        std::cout << "⚙️ +/- - шаг угла, H - справка, ESC - выход\n" << std::endl;
    }

    void printTelegaStatus() {
        std::cout << "\r[Тележка] ШИМ: вперед=" << (neutral_pwm_ + manual_speed_step_)
                  << " назад=" << (neutral_pwm_ - manual_speed_step_)
                  << " | круиз F=" << getCruiseSpeed()
                  << " B=" << getCruiseSpeedBackward()
                  << " | руль: лево=" << steering_left_pwm_
                  << " право=" << steering_right_pwm_
                  << (is_tcp_mode_ ? " [TCP]" : " [KB]")
                  << "        " << std::flush;
    }

    void publishStatus() {
        std::stringstream ss;
        ss << "Тележка: ШИМ: вперед=" << (neutral_pwm_ + manual_speed_step_)
           << " назад=" << (neutral_pwm_ - manual_speed_step_)
           << " | круиз F=" << getCruiseSpeed()
           << " B=" << getCruiseSpeedBackward()
           << " | руль: лево=" << steering_left_pwm_
           << " право=" << steering_right_pwm_
           << (is_tcp_mode_ ? " [TCP]" : " [KB]");

        auto msg = std_msgs::msg::String();
        msg.data = ss.str();
        status_pub_->publish(msg);
    }

    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
        is_connected_ = msg->connected;
        if (is_armed_ != msg->armed) {
            is_armed_ = msg->armed;
            std::cout << "\n[СТАТУС] ArduPilot " << (is_armed_ ? "ARMED!" : "DISARMED!") << std::endl;
        }
    }

    void telegaCmdCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string cmd = msg->data;
        RCLCPP_INFO(this->get_logger(), "🚜 TCP: '%s'", cmd.c_str());

        is_tcp_mode_ = true;
        last_tcp_command_time_ = this->now();

        if ((cruise_control_active_ || cruise_control_backward_) && (cmd == "w" || cmd == "s")) {
            cruise_control_active_ = false;
            cruise_control_backward_ = false;
            std::cout << "\n[Круиз] Отключен через TCP." << std::endl;
        }

        if (cmd == "r") {
            manual_speed_step_ = std::min(450, manual_speed_step_ + 25);
            printTelegaStatus();
        }
        else if (cmd == "f") {
            manual_speed_step_ = std::max(25, manual_speed_step_ - 25);
            printTelegaStatus();
        }
        else if (cmd == "w") {
            if (cruise_control_active_ || cruise_control_backward_) {
                cruise_control_active_ = false;
                cruise_control_backward_ = false;
            }
            linear_throttle_ = neutral_pwm_ + manual_speed_step_;
            last_throttle_press_time_ = this->now();
        }
        else if (cmd == "s") {
            if (cruise_control_active_ || cruise_control_backward_) {
                cruise_control_active_ = false;
                cruise_control_backward_ = false;
            }
            linear_throttle_ = neutral_pwm_ - manual_speed_step_;
            last_throttle_press_time_ = this->now();
        }
        else if (cmd == "a") {
            angular_steering_ = steering_left_pwm_;
            last_steering_press_time_ = this->now();
        }
        else if (cmd == "d") {
            angular_steering_ = steering_right_pwm_;
            last_steering_press_time_ = this->now();
        }
        else if (cmd == "q") {
            cruise_control_active_ = !cruise_control_active_;
            cruise_control_backward_ = false;
            linear_throttle_ = neutral_pwm_;
            angular_steering_ = neutral_pwm_;
            std::cout << "\n[Круиз] " << (cruise_control_active_ ? "ВПЕРЕД ВКЛ" : "ВЫКЛ") << std::endl;
        }
        else if (cmd == "e") {
            cruise_control_backward_ = !cruise_control_backward_;
            cruise_control_active_ = false;
            linear_throttle_ = neutral_pwm_;
            angular_steering_ = neutral_pwm_;
            std::cout << "\n[Круиз] " << (cruise_control_backward_ ? "НАЗАД ВКЛ" : "ВЫКЛ") << std::endl;
        }
        else if (cmd == " " || cmd == "stop") {
            cruise_control_active_ = false;
            cruise_control_backward_ = false;
            linear_throttle_ = neutral_pwm_;
            angular_steering_ = neutral_pwm_;
            is_tcp_mode_ = false;
            std::cout << "\r[TCP] СТОП     " << std::flush;
        }
    }

    void cameraCmdCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string cmd = msg->data;
        RCLCPP_INFO(this->get_logger(), "📷 TCP: '%s'", cmd.c_str());

        float new_pitch = static_cast<float>(pitch_);
        float new_yaw = static_cast<float>(yaw_);
        bool gimbal_cmd = false;

        if (cmd == "UP") { new_pitch = std::min(static_cast<float>(pitch_ + step_), static_cast<float>(max_pitch_)); gimbal_cmd = true; }
        else if (cmd == "DOWN") { new_pitch = std::max(static_cast<float>(pitch_ - step_), static_cast<float>(min_pitch_)); gimbal_cmd = true; }
        else if (cmd == "LEFT") { new_yaw = std::min(static_cast<float>(yaw_ + step_), static_cast<float>(max_yaw_)); gimbal_cmd = true; }
        else if (cmd == "RIGHT") { new_yaw = std::max(static_cast<float>(yaw_ - step_), static_cast<float>(min_yaw_)); gimbal_cmd = true; }
        else if (cmd == "STOP" || cmd == "SPACE") { new_pitch = 0.0f; new_yaw = 0.0f; gimbal_cmd = true; }
        else if (cmd == "Z" || cmd == "PLUS") { sendZoomContinuous(zoom_speed_); }
        else if (cmd == "X" || cmd == "MINUS") { sendZoomContinuous(-zoom_speed_); }
        else if (cmd == "C") { sendZoomStop(); }

        if (gimbal_cmd) {
            pitch_ = new_pitch;
            yaw_ = new_yaw;
            sendGimbalCommand(new_pitch, new_yaw);
        }
    }

    void keyboardReader() {
        while (running_) {
            int key = getch_nonblock();
            if (key != -1) {
                if (key == 27) {
                    int next = getch_nonblock();
                    if (next == 91) {
                        int arrow = getch_nonblock();
                        if (arrow != -1) {
                            int arrow_code = -(arrow + 100);
                            std::lock_guard<std::mutex> lock(queue_mutex_);
                            key_queue_.push(arrow_code);
                            queue_cv_.notify_one();
                        }
                    } else {
                        std::lock_guard<std::mutex> lock(queue_mutex_);
                        key_queue_.push(27);
                        queue_cv_.notify_one();
                    }
                } else {
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    key_queue_.push(key);
                    queue_cv_.notify_one();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    void processKeyQueue() {
        std::queue<int> local_queue;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (key_queue_.empty()) return;
            std::swap(key_queue_, local_queue);
        }

        while (!local_queue.empty()) {
            int key = local_queue.front();
            local_queue.pop();
            processKey(key);
        }
    }

    void processKey(int key) {
        if (is_tcp_mode_ && key != 27 && key != 'h' && key != 'H') {
            return;
        }

        if (is_tcp_mode_) {
            is_tcp_mode_ = false;
            std::cout << "\n[Управление] Переключено на клавиатуру" << std::flush;
        }

        if (key < 0) {
            int arrow = -(key + 100);
            float new_pitch = static_cast<float>(pitch_);
            float new_yaw = static_cast<float>(yaw_);

            switch (arrow) {
            case 65:
                new_pitch = std::min(static_cast<float>(pitch_ + step_), static_cast<float>(max_pitch_));
                break;
            case 66:
                new_pitch = std::max(static_cast<float>(pitch_ - step_), static_cast<float>(min_pitch_));
                break;
            case 67:
                new_yaw = std::max(static_cast<float>(yaw_ - step_), static_cast<float>(min_yaw_));
                break;
            case 68:
                new_yaw = std::min(static_cast<float>(yaw_ + step_), static_cast<float>(max_yaw_));
                break;
            default:
                return;
            }

            pitch_ = new_pitch;
            yaw_ = new_yaw;
            sendGimbalCommand(new_pitch, new_yaw);
            return;
        }

        switch (key) {
        case 'w': case 'W':
            if (cruise_control_active_ || cruise_control_backward_) {
                cruise_control_active_ = false;
                cruise_control_backward_ = false;
                std::cout << "\n[Круиз] Отключен." << std::endl;
            }
            linear_throttle_ = neutral_pwm_ + manual_speed_step_;
            last_throttle_press_time_ = this->now();
            break;

        case 's': case 'S':
            if (cruise_control_active_ || cruise_control_backward_) {
                cruise_control_active_ = false;
                cruise_control_backward_ = false;
                std::cout << "\n[Круиз] Отключен." << std::endl;
            }
            linear_throttle_ = neutral_pwm_ - manual_speed_step_;
            last_throttle_press_time_ = this->now();
            break;

        case 'a': case 'A':
            angular_steering_ = steering_left_pwm_;
            last_steering_press_time_ = this->now();
            break;

        case 'd': case 'D':
            angular_steering_ = steering_right_pwm_;
            last_steering_press_time_ = this->now();
            break;

        case 'q': case 'Q':
            cruise_control_active_ = !cruise_control_active_;
            cruise_control_backward_ = false;
            linear_throttle_ = neutral_pwm_;
            angular_steering_ = neutral_pwm_;
            std::cout << "\n🚜 Круиз ВПЕРЕД: " << (cruise_control_active_ ? "ВКЛ" : "ВЫКЛ") << std::endl;
            break;

        case 'e': case 'E':
            cruise_control_backward_ = !cruise_control_backward_;
            cruise_control_active_ = false;
            linear_throttle_ = neutral_pwm_;
            angular_steering_ = neutral_pwm_;
            std::cout << "\n🚜 Круиз НАЗАД: " << (cruise_control_backward_ ? "ВКЛ" : "ВЫКЛ") << std::endl;
            break;

        case 'r': case 'R':
            manual_speed_step_ = std::min(450, manual_speed_step_ + 25);
            printTelegaStatus();
            break;

        case 'f': case 'F':
            manual_speed_step_ = std::max(25, manual_speed_step_ - 25);
            printTelegaStatus();
            break;

        case ' ':
            cruise_control_active_ = false;
            cruise_control_backward_ = false;
            linear_throttle_ = neutral_pwm_;
            angular_steering_ = neutral_pwm_;
            pitch_ = 0.0f;
            yaw_ = 0.0f;
            sendGimbalCommand(0.0f, 0.0f);
            is_tcp_mode_ = false;
            std::cout << "\r🛑 ПОЛНЫЙ СТОП     " << std::flush;
            break;

        case 'z': case 'Z':
            sendZoomContinuous(zoom_speed_);
            break;

        case 'x': case 'X':
            sendZoomContinuous(-zoom_speed_);
            break;

        case 'c': case 'C':
            sendZoomStop();
            break;

        case '1':
            setImageMode(vdisp_rgb_, "RGB ЗУМ");
            break;

        case '2':
            setImageMode(vdisp_wide_thermal_, "WIDE/THERMAL");
            break;

        case '+': case '=':
            if (step_ < 20.0) step_ += 0.5;
            std::cout << "\r📐 Шаг угла: " << step_ << "°     " << std::flush;
            break;

        case '-': case '_':
            if (step_ > 0.5) step_ -= 0.5;
            std::cout << "\r📐 Шаг угла: " << step_ << "°     " << std::flush;
            break;

        case 'h': case 'H':
            printHelp();
            break;

        case 27:
            RCLCPP_INFO(this->get_logger(), "⏹️ Выход...");
            running_ = false;
            rclcpp::shutdown();
            break;
        }
    }

    void telegaControlLoop() {
        if (!is_connected_) return;

        auto current_time = this->now();

        int timeout_ms = is_tcp_mode_ ? tcp_timeout_ms_ : keyboard_timeout_ms_;

        auto elapsed_steering = current_time - last_steering_press_time_;
        if (elapsed_steering.nanoseconds() > timeout_ms * 1000000LL) {
            angular_steering_ = neutral_pwm_;
        }

        if (!cruise_control_active_ && !cruise_control_backward_) {
            auto elapsed_throttle = current_time - last_throttle_press_time_;
            if (elapsed_throttle.nanoseconds() > timeout_ms * 1000000LL) {
                linear_throttle_ = neutral_pwm_;
            }
        }

        if (is_tcp_mode_) {
            auto elapsed_tcp = current_time - last_tcp_command_time_;
            if (elapsed_tcp.nanoseconds() > tcp_timeout_ms_ * 1000000LL * 2) {
                is_tcp_mode_ = false;
                std::cout << "\n[Управление] TCP режим деактивирован (таймаут)" << std::flush;
            }
        }

        int pwm_motor1 = neutral_pwm_;
        int pwm_motor2 = neutral_pwm_;

        if (is_armed_) {
            int current_linear;
            if (cruise_control_active_) {
                current_linear = getCruiseSpeed();
            } else if (cruise_control_backward_) {
                current_linear = getCruiseSpeedBackward();
            } else {
                current_linear = linear_throttle_;
            }

            int steering_offset = angular_steering_ - neutral_pwm_;

            pwm_motor1 = std::clamp(current_linear + steering_offset, 1000, 2000);
            pwm_motor2 = std::clamp(current_linear - steering_offset, 1000, 2000);
        } else {
            linear_throttle_ = neutral_pwm_;
            angular_steering_ = neutral_pwm_;
            cruise_control_active_ = false;
            cruise_control_backward_ = false;
            is_tcp_mode_ = false;
        }

        sendServoCommand(1, pwm_motor1);
        sendServoCommand(2, pwm_motor2);
    }

    void sendServoCommand(int channel, int pwm_value) {
        if (!command_client_->service_is_ready()) return;

        auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        request->command = 183;
        request->param1 = static_cast<float>(channel);
        request->param2 = static_cast<float>(pwm_value);
        command_client_->async_send_request(request);
    }

    void sendGimbalCommand(float pitch, float yaw) {
        float min_p = static_cast<float>(min_pitch_);
        float max_p = static_cast<float>(max_pitch_);
        float min_y = static_cast<float>(min_yaw_);
        float max_y = static_cast<float>(max_yaw_);

        pitch = std::clamp(pitch, min_p, max_p);
        yaw = std::clamp(yaw, min_y, max_y);

        auto request = std::make_shared<mavros_msgs::srv::GimbalManagerPitchyaw::Request>();
        request->pitch = pitch;
        request->yaw = yaw;
        request->pitch_rate = 0.0f;
        request->yaw_rate = 0.0f;
        request->flags = flags_;
        request->gimbal_device_id = gimbal_device_id_;

        if (gimbal_client_->service_is_ready()) {
            gimbal_client_->async_send_request(request);
        }

        auto msg = mavros_msgs::msg::GimbalManagerSetPitchyaw();
        msg.pitch = pitch;
        msg.yaw = yaw;
        msg.pitch_rate = 0.0f;
        msg.yaw_rate = 0.0f;
        msg.flags = flags_;
        msg.gimbal_device_id = gimbal_device_id_;
        gimbal_pub_->publish(msg);

        auto angle_msg = std_msgs::msg::Float32MultiArray();
        angle_msg.data.push_back(pitch);
        angle_msg.data.push_back(yaw);
        angle_pub_->publish(angle_msg);
    }

    void sendZoomContinuous(double speed) {
        if (!command_client_->service_is_ready()) return;

        auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        req->command = 531;
        req->param1 = 1.0f;
        req->param2 = std::clamp(static_cast<float>(speed), -1.0f, 1.0f);
        command_client_->async_send_request(req);
    }

    void sendZoomStop() {
        if (!command_client_->service_is_ready()) return;

        auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
        req->command = 531;
        req->param1 = 1.0f;
        req->param2 = 0.0f;
        command_client_->async_send_request(req);
    }

    void sendSiyiCommand(uint8_t cmd_id, uint8_t data) {
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) return;

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(camera_port_);
        inet_pton(AF_INET, camera_ip_.c_str(), &addr.sin_addr);

        uint8_t packet[4] = {0x55, 0x01, cmd_id, data};
        sendto(sock, packet, sizeof(packet), 0, (struct sockaddr*)&addr, sizeof(addr));
        close(sock);
    }

    void setImageMode(int vdisp_mode, const std::string& mode_name) {
        RCLCPP_INFO(this->get_logger(), "📺 Режим: %s", mode_name.c_str());
        sendSiyiCommand(0x11, static_cast<uint8_t>(vdisp_mode));

        auto msg = std_msgs::msg::Int32();
        msg.data = (vdisp_mode == vdisp_rgb_) ? 1 : 0;
        stream_pub_->publish(msg);
    }

    // Переменные тележки
    bool is_connected_, is_armed_;
    int linear_throttle_, angular_steering_;
    int manual_speed_step_, cruise_speed_pwm_, cruise_speed_pwm_backward_;
    int steering_left_pwm_, steering_right_pwm_, neutral_pwm_;
    bool cruise_control_active_;
    bool cruise_control_backward_;
    rclcpp::Time last_throttle_press_time_, last_steering_press_time_;
    std::string telega_cmd_topic_;
    std::string status_topic_;

    bool is_tcp_mode_;
    rclcpp::Time last_tcp_command_time_;
    int tcp_timeout_ms_;
    int keyboard_timeout_ms_;

    // Переменные камеры
    double pitch_ = 0.0, yaw_ = 0.0;
    double step_, max_pitch_, min_pitch_, max_yaw_, min_yaw_, zoom_speed_;
    int gimbal_device_id_, flags_;
    std::string gimbal_topic_, angle_publish_topic_, gimbal_service_;
    std::string camera_ip_;
    int camera_port_, vdisp_rgb_, vdisp_wide_thermal_;
    std::string camera_cmd_topic_;

    // Подписчики
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr telega_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_cmd_sub_;

    // Публикаторы
    rclcpp::Publisher<mavros_msgs::msg::GimbalManagerSetPitchyaw>::SharedPtr gimbal_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr stream_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    // Клиенты сервисов
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client_;
    rclcpp::Client<mavros_msgs::srv::GimbalManagerPitchyaw>::SharedPtr gimbal_client_;

    // Таймеры
    rclcpp::TimerBase::SharedPtr telega_timer_;
    rclcpp::TimerBase::SharedPtr process_queue_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // Очередь событий
    std::queue<int> key_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread keyboard_thread_;
    std::atomic<bool> running_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UnifiedController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}