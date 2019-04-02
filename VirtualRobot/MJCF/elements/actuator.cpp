#include "actuator.h"

#include "../Document.h"


using namespace mjcf;


const std::string ActuatorGeneral::tag  = "general";
const std::string ActuatorMotor::tag    = "motor";
const std::string ActuatorPosition::tag = "position";
const std::string ActuatorVelocity::tag = "velocity";
const std::string ActuatorCylinder::tag = "cylinder";
const std::string ActuatorMuscle::tag   = "muscle";

const std::string ActuatorSection::tag  = "actuator";


Eigen::VectorXf ActuatorGeneral::dynprmDefault()
{
    Eigen::VectorXf v(10);
    v.setZero();
    v(0) = 1;
    return v;
}

Eigen::VectorXf ActuatorGeneral::gainprmDefault()
{
    Eigen::VectorXf v(10);
    v.setZero();
    v(0) = 1;
    return v;
}

Eigen::VectorXf ActuatorGeneral::biasprmDefault()
{
    Eigen::VectorXf v(10);
    v.setZero();
    return v;
}

Eigen::Vector2f ActuatorMuscle::timeconstDefault()
{
    return Eigen::Vector2f(0.01, 0.04);
}

Eigen::Vector2f ActuatorMuscle::rangeDefault()
{
    return Eigen::Vector2f(0.75, 1.05);
}


template <class ElementD>
ElementD addActuator(ActuatorSection& as, const std::string& joint)
{
    ElementD motor = as.addChild<ElementD>();
    motor.joint = joint;
    return motor;
}

ActuatorMotor ActuatorSection::addMotor(const std::string& joint)
{
    return addActuator<ActuatorMotor>(*this, joint);
}

ActuatorPosition ActuatorSection::addPosition(const std::string& joint, float kp)
{
    ActuatorPosition position = addActuator<ActuatorPosition>(*this, joint);
    if (kp >= 0)
    {
        position.kp = kp;
    }
    return position;
}

ActuatorVelocity ActuatorSection::addVelocity(const std::string& joint, float kv)
{
    ActuatorVelocity velocity = addActuator<ActuatorVelocity>(*this, joint);
    if (kv >= 0)
    {
        velocity.kv = kv;
    }
    return velocity;
}
