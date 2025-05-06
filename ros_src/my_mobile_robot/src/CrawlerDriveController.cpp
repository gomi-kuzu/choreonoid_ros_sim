#include <cnoid/SimpleController>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <thread>
#include <mutex>

class CrawlerDriveController : public cnoid::SimpleController
{
    public:
        virtual bool configure(cnoid::SimpleControllerConfig* config) override;
        virtual bool initialize(cnoid::SimpleControllerIO* io) override;
        virtual bool control() override;
        virtual void unconfigure() override;    private:
    private:
        cnoid::Link* wheels[2];
        rclcpp::Node::SharedPtr node;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
        geometry_msgs::msg::Twist command;
        rclcpp::executors::StaticSingleThreadedExecutor::UniquePtr executor;
        std::thread executorThread;
        std::mutex commandMutex;};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CrawlerDriveController)

bool CrawlerDriveController::configure(cnoid::SimpleControllerConfig* config)
{
    node = std::make_shared<rclcpp::Node>(config->controllerName());
    subscription = node->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 1,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg){
        std::lock_guard<std::mutex> lock(commandMutex);
        command = *msg;
    });
    executor = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    executor->add_node(node);
    executorThread = std::thread([this](){executor->spin();});
    return true;
}


bool CrawlerDriveController::initialize(cnoid::SimpleControllerIO* io)
{
    auto body = io->body();
    wheels[0] = body->link("CRAWLER_TRACK_L");
    wheels[1] = body->link("CRAWLER_TRACK_R");
    for(int i=0; i < 2; ++i){
        auto wheel = wheels[i];
        wheel->setActuationMode(JointVelocity);
        io->enableOutput(wheel, JointVelocity);
    }
    return true;
}

bool CrawlerDriveController::control()
{
    constexpr double wheelRadius = 0.076;
    constexpr double halfAxleWidth = 0.145;
    constexpr double kd = 0.3;
    double dq_target[2];

{
        std::lock_guard<std::mutex> lock(commandMutex);
        double dq_x = command.linear.x / wheelRadius;
        double dq_yaw = command.angular.z * halfAxleWidth / wheelRadius;
        dq_target[0] = dq_x - dq_yaw;
        dq_target[1] = dq_x + dq_yaw;
}

    for(int i=0; i < 2; ++i){
        wheels[i]->dq_target() = kd * (dq_target[i] - wheels[i]->dq_target()); //速度のP制御
    }
    return true;
}

void CrawlerDriveController::unconfigure()
{
    if(executor){
        executor->cancel();
        executorThread.join();
        executor->remove_node(node);
        executor.reset();
    }
}