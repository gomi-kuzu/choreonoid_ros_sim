#include <cnoid/SimpleController>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>
#include <thread>
#include <mutex>

class SubCrawlerDriveController : public cnoid::SimpleController
{
    public:
        virtual bool configure(cnoid::SimpleControllerConfig* config) override;
        virtual bool initialize(cnoid::SimpleControllerIO* io) override;
        virtual bool control() override;
        virtual void unconfigure() override;
    private:
        cnoid::Link* joints[2];
        rclcpp::Node::SharedPtr node;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription;
        geometry_msgs::msg::Vector3 command;
        std::unique_ptr<rclcpp::executors::StaticSingleThreadedExecutor> executor;
        std::thread executorThread;
        std::mutex commandMutex;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SubCrawlerDriveController)

bool SubCrawlerDriveController::configure(cnoid::SimpleControllerConfig* config)
{
    node = std::make_shared<rclcpp::Node>(config->controllerName());
    subscription = node->create_subscription<geometry_msgs::msg::Vector3>(
    "/cmd_sub_craw_joint_trq", 1,
    [this](const geometry_msgs::msg::Vector3::SharedPtr msg){
        std::lock_guard<std::mutex> lock(commandMutex);
        command = *msg;
    });
    executor = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    executor->add_node(node);
    executorThread = std::thread([this](){executor->spin();});
    return true;
}

bool SubCrawlerDriveController::initialize(cnoid::SimpleControllerIO* io)
{
    auto body = io->body();
    joints[0] = body->joint("SubCraw_L_joint");
    joints[1] = body->joint("SubCraw_R_joint");
    for(int i=0; i < 2; ++i){
        auto joint = joints[i];
        joint->setActuationMode(JointAngle);
        io->enableInput(joint, JointTorque);
        io->enableInput(joint, JointAngle);
        io->enableOutput(joint, JointTorque);
        io->enableOutput(joint, JointAngle);

    }
    return true;
}
bool SubCrawlerDriveController::control()
{
    constexpr double kd = 1;
    double q_target[2];

{
        std::lock_guard<std::mutex> lock(commandMutex);
        q_target[0] = command.z;
        q_target[1] = command.y;
}

    for(int i=0; i < 2; ++i){
        auto joint = joints[i];
        joint->q_target() = joint->q() + kd * q_target[i];
    }
    return true;
}

void SubCrawlerDriveController::unconfigure()
{
    if(executor){
        executor->cancel();
        executorThread.join();
        executor->remove_node(node);
        executor.reset();
    }
}