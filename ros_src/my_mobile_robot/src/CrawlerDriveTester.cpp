/*
https://choreonoid.org/ja/manuals/latest/simulation/pseudo-continuous-track.htmlを参考に
*/

#include <cnoid/SimpleController>

class CrawlerDriveTester : public cnoid::SimpleController
{
    public:
        virtual bool initialize(cnoid::SimpleControllerIO* io) override;
        virtual bool control() override;
    private:
        cnoid::Link* wheels[2];
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CrawlerDriveTester)

bool CrawlerDriveTester::initialize(cnoid::SimpleControllerIO* io)
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
bool CrawlerDriveTester::control()
{
    wheels[0]->dq_target() = 1.0;
    wheels[1]->dq_target() = 1.0;
    return true;
}