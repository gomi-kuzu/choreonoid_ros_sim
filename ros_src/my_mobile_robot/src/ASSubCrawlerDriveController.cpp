#include <cnoid/SimpleController>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>
#include <thread>
#include <mutex>

class ASSubCrawlerDriveController : public cnoid::SimpleController
{
    public:
        virtual bool configure(cnoid::SimpleControllerConfig* config) override;
        virtual bool initialize(cnoid::SimpleControllerIO* io) override;
        virtual bool control() override;
        virtual void unconfigure() override;
    private:
        cnoid::Link* joints[4];
        rclcpp::Node::SharedPtr node;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription;
        geometry_msgs::msg::Vector3 command;
        std::unique_ptr<rclcpp::executors::StaticSingleThreadedExecutor> executor;
        std::thread executorThread;
        std::mutex commandMutex;
        double dt;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ASSubCrawlerDriveController)

bool ASSubCrawlerDriveController::configure(cnoid::SimpleControllerConfig* config)
{
    node = std::make_shared<rclcpp::Node>(config->controllerName());
    subscription = node->create_subscription<geometry_msgs::msg::Vector3>(
    "/cmd_sub_craw_joint", 1,
    [this](const geometry_msgs::msg::Vector3::SharedPtr msg){
        std::lock_guard<std::mutex> lock(commandMutex);
        command = *msg;
    });
    executor = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    executor->add_node(node);
    executorThread = std::thread([this](){executor->spin();});
    return true;
}

bool ASSubCrawlerDriveController::initialize(cnoid::SimpleControllerIO* io)
{
    auto body = io->body();
    dt = io->timeStep();
    joints[0] = body->joint("FL_FLIPPER_JOINT");
    joints[1] = body->joint("FR_FLIPPER_JOINT");
    joints[2] = body->joint("BL_FLIPPER_JOINT");
    joints[3] = body->joint("BR_FLIPPER_JOINT");
    for(int i=0; i < 4; ++i){
        auto joint = joints[i];
        joint->setActuationMode(JointDisplacement);
        io->enableIO(joint);

        // io->enableInput(joint, JointTorque);
        // io->enableOutput(joint, JointTorque);
    }
    // for(int i=0; i < 2; ++i){
    //     auto joint = joints[i+2];
    //     joint->setActuationMode(JointTorque);
    //     io->enableInput(joint, JointTorque);
    //     io->enableOutput(joint, JointTorque);
    // }
    return true;
}
bool ASSubCrawlerDriveController::control()
{
    constexpr double kd = 1.5;
    double q_target[4];

{
        std::lock_guard<std::mutex> lock(commandMutex);
        q_target[0] = command.z;
        q_target[1] = command.y;
        q_target[2] = command.x;
        q_target[3] = command.x;
}

    for(int i=0; i < 4; ++i){
        auto joint = joints[i];
        joint->q_target() = joint->q() + kd * q_target[i] * dt;
    }
    // for(int i=0; i < 2; ++i){
    //     auto joint = joints[i+2];
    //     joint->q_target() = joint->q();
    //     // joint->u() = 0;
    // }
    return true;
}

void ASSubCrawlerDriveController::unconfigure()
{
    if(executor){
        executor->cancel();
        executorThread.join();
        executor->remove_node(node);
        executor.reset();
    }
}