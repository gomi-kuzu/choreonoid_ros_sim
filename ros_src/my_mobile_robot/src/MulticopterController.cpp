#include <cnoid/SimpleController>
#include <cnoid/RotorDevice>
#include <cnoid/EigenUtil>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <thread>
#include <mutex>

using namespace std;
using namespace cnoid;

namespace {

const string propname[] = { "PROP0", "PROP1", "PROP2", "PROP3" };
const std::string rotorname[] = { "RotorDevice0", "RotorDevice1", "RotorDevice2", "RotorDevice3" };
};

const double sign[4][4] = {
    { 1.0, -1.0, -1.0,  1.0 },
    { 1.0,  1.0, -1.0, -1.0 },
    { 1.0,  1.0,  1.0,  1.0 },
    { 1.0, -1.0,  1.0, -1.0 }
};
const double dir[] = { 1.0, -1.0, 1.0, -1.0 };

static const double KP[] = { 0.4, 0.4, 0.4, 1.0 };
static const double KD[] = { 0.02, 1.0, 1.0, 0.05 };
const double RATE[] = { -1.0, 0.1, -0.1, -0.5 };

// For the stable mode
const double KPX[] = { 0.4, 0.4 };
const double KDX[] = { 0.4, 0.4 };
const double RATEX[] = { -1.0, -1.0 };


class MulticopterController : public cnoid::SimpleController
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Vector4 getZRPY();
        Vector2 getXY();
        virtual bool configure(cnoid::SimpleControllerConfig* config) override;
        virtual bool initialize(cnoid::SimpleControllerIO* io) override;
        virtual bool control() override;
        virtual void unconfigure() override;    private:
    private:

        int targetMode;
        BodyPtr ioBody;
        ostream* os;
        Link* prop[4];
        Multicopter::RotorDevice* rotor[4];
        Link* cameraT;
        double timeStep;

        Vector4 zrpyref;
        Vector4 zrpyprev;
        Vector4 dzrpyref;
        Vector4 dzrpyprev;

        // For the stable mode
        bool isStableMode;
        bool prevModeButtonState;
        Vector2 xyref;
        Vector2 xyprev;
        Vector2 dxyref;
        Vector2 dxyprev;
        
        double qref;
        double qprev;
        bool rotorswitch;

        rclcpp::Node::SharedPtr node;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr upDownSubscription;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr forBackSubscription;
        geometry_msgs::msg::Vector3 upDownCommand;
        geometry_msgs::msg::Vector3 forBackCommand;
        rclcpp::executors::StaticSingleThreadedExecutor::UniquePtr executor;
        std::thread executorThread;
        std::mutex commandMutex;
    };

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(MulticopterController)

bool MulticopterController::configure(cnoid::SimpleControllerConfig* config)
{
    node = std::make_shared<rclcpp::Node>(config->controllerName());
    upDownSubscription = node->create_subscription<geometry_msgs::msg::Vector3>(
    "/cmd_up_down", 1,
    [this](const geometry_msgs::msg::Vector3::SharedPtr msg){
        std::lock_guard<std::mutex> lock(commandMutex);
        upDownCommand = *msg;
    });
    forBackSubscription = node->create_subscription<geometry_msgs::msg::Vector3>(
    "/cmd_for_back", 1,
    [this](const geometry_msgs::msg::Vector3::SharedPtr msg){
        std::lock_guard<std::mutex> lock(commandMutex);
        forBackCommand = *msg;
    });
    executor = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    executor->add_node(node);
    executorThread = std::thread([this](){executor->spin();});
    return true;
}


bool MulticopterController::initialize(cnoid::SimpleControllerIO* io)
{
    ioBody = io->body();
    os = &io->os();
    timeStep = io->timeStep();

    io->enableInput(ioBody->rootLink(), LINK_POSITION);

    cameraT = ioBody->link("CAMERA_T");
    cameraT->setActuationMode(Link::JointTorque);
    io->enableIO(cameraT);
    qref = qprev = cameraT->q();

    for(int i = 0; i < 4; i++) {
        prop[i] = ioBody->link(propname[i]);
        // prop[i]->setActuationMode(Link::JointVelocity);
        // io->enableIO(prop[i]);

        prop[i]->setActuationMode(Link::JointTorque);
        io->enableInput(prop[i], JOINT_VELOCITY);
        io->enableOutput(prop[i]);

        rotor[i] = ioBody->findDevice<Multicopter::RotorDevice>(rotorname[i]);
        io->enableInput(rotor[i]);
    }

    zrpyref = zrpyprev = getZRPY();
    dzrpyref = dzrpyprev = Vector4::Zero();

    isStableMode = false;
    prevModeButtonState = false;
    xyref = xyprev = getXY();
    dxyref = dxyprev = Vector2::Zero();


    rotorswitch = true;


    return true;
}

Vector4 MulticopterController::getZRPY()
{
    auto T = ioBody->rootLink()->position();
    double z = T.translation().z();
    Vector3 rpy = rpyFromRot(T.rotation());
    return Vector4(z, rpy[0], rpy[1], rpy[2]);
}   

Vector2 MulticopterController::getXY()
{
    auto p = ioBody->rootLink()->translation();
    return Vector2(p.x(), p.y());
}

bool MulticopterController::control()
{
   //control rotors
   Vector4 zrpy = getZRPY();
   double cc = cos(zrpy[1]) * cos(zrpy[2]);
   double gfcoef = 1.0 * 9.80665 / 4 / cc ;
   Vector4 force = Vector4::Zero();
   Vector4 torque = Vector4::Zero();

   bool modeButtonState = forBackCommand.x;
   if(modeButtonState){
       if(!prevModeButtonState){
           isStableMode = !isStableMode;
       }
   }
   prevModeButtonState = modeButtonState;

   if(rotorswitch) {
       Vector4 f;

       Vector4 dzrpy = (zrpy - zrpyprev) / timeStep;
       Vector4 ddzrpy = (dzrpy - dzrpyprev) / timeStep;

       // For the stable mode
       Vector2 xy = getXY();
       Vector2 dxy = (xy - xyprev) / timeStep;
       Vector2 ddxy = (dxy - dxyprev) / timeStep;
       Vector2 dxy_local = Eigen::Rotation2Dd(-zrpy[3]) * dxy;
       Vector2 ddxy_local = Eigen::Rotation2Dd(-zrpy[3]) * ddxy;

       for(int axis = 0; axis < 4; ++axis) {
           
           double pos {0.0};
           if(axis == 0){
                pos = -1*upDownCommand.z;
            }else if(axis == 1){
                pos = forBackCommand.z;
            }else if(axis == 2){
                pos = upDownCommand.y;
            }else if(axis == 3){
                pos = -1*forBackCommand.y;
           }
           if((axis == 0) || (axis == 3)) {
               if(fabs(pos) > 0.25) {
                   dzrpyref[axis] = RATE[axis] * pos;
               } else {
                   dzrpyref[axis] = 0.0;
               }
               f[axis] = KP[axis] * (dzrpyref[axis] - dzrpy[axis]) + KD[axis] * (0.0 - ddzrpy[axis]);
           } else {
               if(!isStableMode){
                   if(fabs(pos) > 0.25) {
                       zrpyref[axis] = RATE[axis] * pos;
                   } else {
                       zrpyref[axis] = 0.0;
                   }
               } else {
                   int axis_xy = axis - 1;
                   if(fabs(pos) > 0.25) {
                       dxyref[axis_xy] = RATEX[axis_xy] * pos;
                   } else {
                       dxyref[axis_xy] = 0.0;
                   }
                   zrpyref[axis] =
                       KPX[axis_xy] * (dxyref[axis_xy] - dxy_local[1 - axis_xy]) +
                       KDX[axis_xy] * (0.0 - ddxy_local[1 - axis_xy]);

                   if(axis == 1) {
                       zrpyref[axis] *= -1.0;
                   }
               }
               f[axis] = KP[axis] * (zrpyref[axis] - zrpy[axis]) + KD[axis] * (0.0 - dzrpy[axis]);
           }
       }
       zrpyprev = zrpy;
       dzrpyprev = dzrpy;

       // For the stable mode
       xyprev = xy;
       dxyprev = dxy;
       
       for(int i = 0; i < 4; ++i) {
           double fi = 0.0;
           fi += gfcoef;
           fi += sign[i][0] * f[0];
           fi += sign[i][1] * f[1];
           fi += sign[i][2] * f[2];
           fi += sign[i][3] * f[3];
           force[i] = fi;
           torque[i] = dir[i] * fi;
       }
   }

   for(int i = 0; i < 4; ++i) {
       double tau = torque[i];
       rotor[i]->setTorque(tau);
       if(tau != 0.0) {
           prop[i]->u() = tau * 0.001;
       } else {
           double dq = prop[i]->dq();
           prop[i]->u() = 0.0005 * (0.0 - dq);
       }
       rotor[i]->setValue(force[i]);
       rotor[i]->notifyStateChange();
   }




//for test
    //     constexpr double kd = 0.3;
//     constexpr double v_scale = 2000;
//     double dq_target[4];
// {
//         std::lock_guard<std::mutex> lock(commandMutex);
//         double dq_x = v_scale * upDownCommand.y;
//         dq_target[0] = dq_x;
//         dq_target[1] = dq_x;
//         dq_target[2] = dq_x;
//         dq_target[3] = dq_x;
// }

//     for(int i=0; i < 4; ++i){
//         prop[i]->dq_target() = kd * (dq_target[i] - prop[i]->dq_target()); //速度のP制御
//     }
    return true;
}

void MulticopterController::unconfigure()
{
    if(executor){
        executor->cancel();
        executorThread.join();
        executor->remove_node(node);
        executor.reset();
    }
}

