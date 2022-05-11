#include <stdio.h>
#include <ncurses.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <thread>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

#include "geometry_msgs/msg/twist.hpp"

#define INPUT_TIMER_DURATION_MS 1

class Teleop : public rclcpp::Node
{
    public:
        explicit Teleop() : Node("Teleop")
        {
            initscr();
            noecho();
            timeout(0);

            createParameters();

            linearSpeedFactor = defaultLinearSpeedFactor;
            angularSpeedFactor = defaultAngularSpeedFactor;

            drawScreen();

            twistPublisher = this->create_publisher<geometry_msgs::msg::Twist>(topicName, 10);

            outputTimer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(outputRateMs),
                    std::bind(&Teleop::outputTimerCallback, this));
            inputTimer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(INPUT_TIMER_DURATION_MS),
                    std::bind(&Teleop::inputTimerCallback, this));
        }

        ~Teleop()
        {
            endwin();
        }
    
    private:
        std::map<char, std::vector<float>> moveBindings
        {
            {'i', {1, 0, 0}},
            {'o', {1, 0, -1}},
            {'j', {0, 0, 1}},
            {'l', {0, 0, -1}},
            {'u', {1, 0, 1}},
            {',', {-1, 0, 0}},
            {'.', {-1, 0, 1}},
            {'m', {-1, 0, -1}},
            {'O', {1, -1, 0}},
            {'I', {1, 0, 0}},
            {'J', {0, 1, 0}},
            {'L', {0, -1, 0}},
            {'U', {1, 1, 0}},
            {'<', {-1, 0, 0}},
            {'>', {-1, -1, 0}},
            {'M', {-1, 1, 0}},
            {'t', {0, 0, 0}},
            {'b', {0, 0, 0}}
            //k or K will cause a stop since they are not defined
        };
        std::map<char, std::vector<float>> speedBindings
        {
            {'q', {1.1, 1.1}},
            {'z', {0.9, 0.9}},
            {'w', {1.1, 1}},
            {'x', {0.9, 1}},
            {'e', {1, 1.1}},
            {'c', {1, 0.9}}
        };
        std::map<char, std::vector<float>> speedAbsBindings
        {
            {'a', {1, 1}},
            {'s', {1, 0}},
            {'d', {0, 1}}
        };

        void createParameters()
        {
            this->declare_parameter<double>("linear_speed", 1);
            this->declare_parameter<double>("angular_speed", 1);
            this->declare_parameter<int64_t>("output_rate_ms", 50);
            this->declare_parameter<int64_t>("input_expiration_ms", 2000);
            this->declare_parameter<std::string>("topic", "/cmd_vel");

            this->get_parameter("linear_speed", defaultLinearSpeedFactor);
            this->get_parameter("angular_speed", defaultAngularSpeedFactor);
            this->get_parameter("output_rate_ms", outputRateMs);
            this->get_parameter("input_expiration_ms", inputExpirationMs);
            this->get_parameter("topic", topicName);
        }

        void drawScreen()
        {
            mvprintw(0, 0,  "       Safer Teleop");
            mvprintw(1, 0,  "-------------------------");
            mvprintw(2, 0,  "Moving in a straight line");
            mvprintw(3, 0,  "U           I           O");
            mvprintw(4, 0,  "J           K (Stop)    L");
            mvprintw(5, 0,  "M           <           >");

            mvprintw(7, 0,  "Moving and turning");
            mvprintw(8, 0,  "u           i           o");
            mvprintw(9, 0,  "j           k (Stop)    l");
            mvprintw(10, 0, "m           ,           .");

            mvprintw(12, 0, "Speed   | Faster | Slower | Reset");
            mvprintw(13, 0, "---------------------------------");
            mvprintw(14, 0, "Overall |   q    |   z    |   a");
            mvprintw(15, 0, " Linear |   w    |   x    |   s");
            mvprintw(16, 0, "Angular |   e    |   c    |   d");

            updateSpeedDisplay();

        }

        void sendTwistMessage()
        {
            geometry_msgs::msg::Twist message;

            message.linear.x = speeds[0] * linearSpeedFactor;
            message.linear.y = speeds[1] * linearSpeedFactor;
            message.linear.z = 0;
            message.angular.x = 0;
            message.angular.y = 0;
            message.angular.z = speeds[2] * angularSpeedFactor;

            twistPublisher->publish(message);
        }

        void outputTimerCallback()
        {
            sendTwistMessage();
        }

        void updateSpeedDisplay()
        {
            move(19, 0);
            clrtoeol();
            mvprintw(19, 0, "Linear: %f", linearSpeedFactor);
            move(20, 0);
            clrtoeol();
            mvprintw(20, 0, "Angular: %f", angularSpeedFactor);
            move(21, 0);
        }

        void setStop()
        {
            speeds[0] = speeds[1] = speeds[2] = 0;
        }

        void inputTimerCallback()
        {
            int c = getch();
            if(c == ERR)
            {
                timerCount++;
                if(timerCount >= inputExpirationMs / INPUT_TIMER_DURATION_MS)
                {
                    setStop();
                    sendTwistMessage();
                    timerCount = 0;
                }
                return;
            }
            else
            {
                if(moveBindings.count(c) == 1)
                {
                    speeds[0] = moveBindings[c][0];
                    speeds[1] = moveBindings[c][1];
                    speeds[2] = moveBindings[c][2];
                }
                else if(speedBindings.count(c) == 1)
                {
                    linearSpeedFactor *= speedBindings[c][0];
                    angularSpeedFactor *= speedBindings[c][1];

                    updateSpeedDisplay();
                }
                else if(speedAbsBindings.count(c) == 1)
                {
                    if(speedAbsBindings[c][0])
                        linearSpeedFactor = defaultLinearSpeedFactor;
                    if(speedAbsBindings[c][1])
                        angularSpeedFactor = defaultAngularSpeedFactor;

                    updateSpeedDisplay();
                }
                else
                {
                    setStop();
                }

                sendTwistMessage();
            }

            timerCount = 0;
        }

        rclcpp::TimerBase::SharedPtr outputTimer;
        rclcpp::TimerBase::SharedPtr inputTimer;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPublisher;

        double linearSpeedFactor;
        double angularSpeedFactor;
        double defaultLinearSpeedFactor;
        double defaultAngularSpeedFactor;
        double speeds[3];
        int timerCount;
        int outputRateMs;
        int inputExpirationMs;
        std::string topicName;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exe;

    std::shared_ptr<Teleop> lc_node =
        std::make_shared<Teleop>();

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();
  rclcpp::shutdown();
    return 0;
}