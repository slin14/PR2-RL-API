#include <iostream>
#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/XmlFactory.h>

int
main(int argc, char** argv)
{

	rl::mdl::XmlFactory factory;
	rl::mdl::Kinematic* kinematics = dynamic_cast<rl::mdl::Kinematic*>(factory.create("C:\\Users\\sophi\\rl-0.7.0\\build\\robot.rlmdl.xml"));
	rl::math::Vector q(7);
	q << -22.918, 57.298, 0, -117.456, 0, -5.7296, 0;
	q *= rl::math::DEG2RAD;
	kinematics->setPosition(q);
	kinematics->forwardPosition();
	rl::math::Transform t = kinematics->getOperationalPosition(0);
	rl::math::Vector3 position = t.translation();
	std::cout << "Joint configuration in degrees: " << q.transpose() * rl::math::RAD2DEG << std::endl;
	std::cout << "End-effector position: [m] " << position.transpose() << std::endl;

	std::shared_ptr<rl::mdl::Model> model(factory.create("model.xml"));
	rl::math::Vector qd = model->getVelocity();
	std::cout << "Velocity: " << qd.transpose() * rl::math::RAD2DEG << std::endl;

	rl::math::Vector qdd = model->getAcceleration();
	std::cout << "Acceleration: " << qdd.transpose() * rl::math::RAD2DEG << std::endl;

	rl::math::Vector tau = model->getTorque();
	std::cout << "Torque: " << tau.transpose() * rl::math::RAD2DEG << std::endl;

	/*rl::mdl::XmlFactory factory;
	rl::mdl::Kinematic* kinematics = dynamic_cast<rl::mdl::Kinematic*>(factory.create("C:\\Users\\sophi\\rl-0.7.0\\build\\robot.rlmdl.xml"));
	rl::math::Vector q(7);
	q << -22.918, 57.298, 0, -117.456, 0, -5.7296, 0;
	q *= rl::math::DEG2RAD;
	kinematics->setPosition(q);
	kinematics->forwardPosition();
	rl::math::Transform t = kinematics->getOperationalPosition(0);
	rl::math::Vector3 position = t.translation();
	rl::math::Vector3 orientation = t.rotation().eulerAngles(2, 1, 0).reverse();
	std::cout << "Joint configuration in degrees: " << q.transpose() * rl::math::RAD2DEG << std::endl;
	std::cout << "End-effector position: [m] " << position.transpose() << " orientation [deg] " << orientation.transpose() * rl::math::RAD2DEG << std::endl;
	*/
	return 0;
}