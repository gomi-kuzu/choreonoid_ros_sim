#include <cnoid/SimpleController>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <thread>
#include <mutex>

class AizuSpiderDriveController : public cnoid::SimpleController
{
    public:
        virtual bool configure(cnoid::SimpleControllerConfig* config) override;
        virtual bool initialize(cnoid::SimpleControllerIO* io) override;
        virtual bool control() override;
        virtual void unconfigure() override;    private:
    private:
        cnoid::Link* wheels[6];
        rclcpp::Node::SharedPtr node;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
        geometry_msgs::msg::Twist command;
        rclcpp::executors::StaticSingleThreadedExecutor::UniquePtr executor;
        std::thread executorThread;
        std::mutex commandMutex;};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(AizuSpiderDriveController)

bool AizuSpiderDriveController::configure(cnoid::SimpleControllerConfig* config)
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


bool AizuSpiderDriveController::initialize(cnoid::SimpleControllerIO* io)
{
    auto body = io->body();
    wheels[0] = body->link("L_TRACK");
    wheels[1] = body->link("R_TRACK");
    wheels[2] = body->link("FL_SUB_TRACK");
    wheels[3] = body->link("FR_SUB_TRACK");
    wheels[4] = body->link("BL_SUB_TRACK");
    wheels[5] = body->link("BR_SUB_TRACK");

    for(int i=0; i < 6; ++i){
        auto wheel = wheels[i];
        wheel->setActuationMode(JointVelocity);
        io->enableOutput(wheel, JointVelocity);
    }
    return true;
}

bool AizuSpiderDriveController::control()
{
    constexpr double wheelRadius = 0.176;
    constexpr double halfAxleWidth = 0.3;
    constexpr double kd = 0.3;
    double dq_target[6];

{
        std::lock_guard<std::mutex> lock(commandMutex);
        double dq_x = command.linear.x / wheelRadius;
        double dq_yaw = command.angular.z * halfAxleWidth / wheelRadius;
        double trgFF = command.linear.y;
        double trgBF = command.linear.z;
        dq_target[0] = dq_x - dq_yaw;
        dq_target[1] = dq_x + dq_yaw;
        dq_target[2] = trgFF * (dq_x - dq_yaw);
        dq_target[3] = trgFF * (dq_x + dq_yaw);
        dq_target[4] = trgBF * (dq_x - dq_yaw);
        dq_target[5] = trgBF * (dq_x + dq_yaw);

}

    for(int i=0; i < 6; ++i){
        wheels[i]->dq_target() = kd * (dq_target[i] - wheels[i]->dq_target()); //速度のP制御
    }
    return true;
}

void AizuSpiderDriveController::unconfigure()
{
    if(executor){
        executor->cancel();
        executorThread.join();
        executor->remove_node(node);
        executor.reset();
    }
}