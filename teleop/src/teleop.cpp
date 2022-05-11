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

class Teleop : public rclcpp::Node
{
    public:
        explicit Teleop() : Node("Teleop")
        {
            initscr();
            timeout(0);

            twistPublisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

            outputTimer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(50), std::bind(&Teleop::outputTimerCallback, this));
            inputTimer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(5), std::bind(&Teleop::inputTimerCallback, this));
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
                if(timerCount == 400)
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

        float linearSpeedFactor = 1;
        float angularSpeedFactor = 1;
        float speeds[3];
        int timerCount;
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