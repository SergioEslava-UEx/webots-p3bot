/*
 *    Copyright (C) 2025 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

#pragma region ROBOCOMP_METHODS

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}

        wheelsMatrix <<
                     1.0/ WHEEL_RADIUS,  -1.0/ WHEEL_RADIUS, SumLxLyOverRadius * ROTATION_INCREMENT_COEFFICIENT,
                    -1.0/ WHEEL_RADIUS,  -1.0/ WHEEL_RADIUS,  SumLxLyOverRadius * ROTATION_INCREMENT_COEFFICIENT,
                    1.0/ WHEEL_RADIUS,   1.0/ WHEEL_RADIUS,  SumLxLyOverRadius * ROTATION_INCREMENT_COEFFICIENT,
                    -1.0/ WHEEL_RADIUS,  1.0/ WHEEL_RADIUS, SumLxLyOverRadius * ROTATION_INCREMENT_COEFFICIENT;

    }
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}


void SpecificWorker::initialize()
{
    std::cout << "Initialize worker" << std::endl;
    this->setPeriod("Compute",TIME_STEP);

    robot = new webots::Supervisor();
    robotNode = robot->getSelf();

    // Base motors and sensors initialization.
    const char *motorNames[4] = {"wheel1", "wheel2", "wheel3", "wheel4"};
    for (int i = 0; i < 4; i++)
    {
        motors[i] = robot->getMotor(motorNames[i]);
        positionSensors[i] = motors[i]->getPositionSensor();
        positionSensors[i]->enable(this->getPeriod("Compute"));
        motors[i]->setPosition(INFINITY); // Speed Mode
        motors[i]->setVelocity(0);
    }

    // Right Kinova Arm motors and sensors initialization.
    std::string prefix = "Right_";
    for (size_t i = 0; i < kinovaMotorNames.size(); ++i) {

        kinovaArmRMotors.push_back(robot->getMotor(prefix + kinovaMotorNames[i]));
        kinovaArmRSensors.push_back(robot->getPositionSensor(prefix + kinovaSensorNames[i]));

        if (!kinovaArmRMotors[i] || !kinovaArmRSensors[i]) {
            std::cerr << "Error: No se pudo obtener el motor o sensor para el actuator " << i+1 << std::endl;
            continue;
        }

        kinovaArmRSensors[i]->enable(this->getPeriod("Compute"));
    }

    // Left Kinova Arm motors and sensors initialization.
    prefix = "Left_";
    for (size_t i = 0; i < kinovaMotorNames.size(); ++i) {

        kinovaArmLMotors.push_back(robot->getMotor(prefix + kinovaMotorNames[i]));
        kinovaArmLSensors.push_back(robot->getPositionSensor(prefix + kinovaSensorNames[i]));

        if (!kinovaArmLMotors[i] || !kinovaArmLSensors[i]) {
            std::cerr << "Error: No se pudo obtener el motor o sensor para el actuator " << i+1 << std::endl;
            continue;
        }

        kinovaArmLSensors[i]->enable(this->getPeriod("Compute"));
    }
}



void SpecificWorker::compute()
{
    double now = robot->getTime() * 1000;

    if(robot) receiving_robotSpeed(robot, now);
    robot->step(this->getPeriod("Compute"));
}



void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
}



//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}

#pragma endregion ROBOCOMP_METHODS


void SpecificWorker::receiving_robotSpeed(webots::Supervisor* _robot, double timestamp)
{
    const double* shadow_position = robotNode->getPosition();
    const double* shadow_orientation = robotNode->getOrientation();
    const double* shadow_velocity = robotNode->getVelocity();
    float orientation = atan2(shadow_orientation[1], shadow_orientation[0]) - M_PI_2;

    Eigen::Matrix2f rt_rotation_matrix;
    rt_rotation_matrix << cos(orientation), -sin(orientation),
            sin(orientation), cos(orientation);

    // Multiply the velocity vector by the inverse of the rotation matrix to get the velocity in the robot reference system
    Eigen::Vector2f shadow_velocity_2d(shadow_velocity[1], shadow_velocity[0]);
    Eigen::Vector2f rt_rotation_matrix_inv = rt_rotation_matrix.inverse() * shadow_velocity_2d;

    // Raw speeds
    double velocidad_x = 0.1; // mm/s
    double velocidad_y = 0.1;
    double alpha = 0.075;  // rads/s

    // Noise standard deviation (example: 5% of speed values)
    double ruido_stddev_x = 0.05 * velocidad_x;
    double ruido_stddev_y = 0.05 * velocidad_y;
    double ruido_stddev_alpha = 0.05 * alpha;

    RoboCompFullPoseEstimation::FullPoseEuler pose_data;

    // Position
    pose_data.x = -shadow_position[1];  // metros → mm
    pose_data.y = shadow_position[0];
    pose_data.z = shadow_position[2];

    // Orientation (Euler in rads) 2D
    pose_data.rx = 0.0;
    pose_data.ry = 0.0;
    pose_data.rz = orientation;  // Calculated Z-Axis

    pose_data.vx = -rt_rotation_matrix_inv(0) + generateNoise(ruido_stddev_x);
    pose_data.vy = -rt_rotation_matrix_inv(1) + generateNoise(ruido_stddev_y);
    pose_data.vz = 0;
    pose_data.vrx = 0;
    pose_data.vry = 0;
    pose_data.vrz = shadow_velocity[5] + generateNoise(ruido_stddev_alpha);
    pose_data.timestamp = timestamp;

    this->fullposeestimationpub_pubproxy->newFullPose(pose_data);
}

double SpecificWorker::generateNoise(double stddev)
{
    std::random_device rd; // Obtiene una semilla aleatoria del hardware
    std::mt19937 gen(rd()); // Generador de números aleatorios basado en Mersenne Twister
    std::normal_distribution<> d(0, stddev); // Distribución normal con media 0 y desviación estándar stddev
    return d(gen);
}

#pragma region OMNIROBOT_INTERFACE

void SpecificWorker::OmniRobot_correctOdometer(int x, int z, float alpha)
{
    printNotImplementedWarningMessage("OmniRobot_correctOdometer");
}

void SpecificWorker::OmniRobot_getBasePose(int &x, int &z, float &alpha)
{
    printNotImplementedWarningMessage("OmniRobot_getBasePose");
}

void SpecificWorker::OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state)
{
    state.x = robotNode->getField("translation")->getSFVec3f()[0];
    state.z = robotNode->getField("translation")->getSFVec3f()[1];
    state.alpha = robotNode->getField("rotation")->getSFRotation()[3];
}

void SpecificWorker::OmniRobot_resetOdometer()
{
    printNotImplementedWarningMessage("OmniRobot_resetOdometer");
}

void SpecificWorker::OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state)
{
    printNotImplementedWarningMessage("OmniRobot_setOdometer");
}

void SpecificWorker::OmniRobot_setOdometerPose(int x, int z, float alpha)
{
    printNotImplementedWarningMessage("OmniRobot_setOdometerPose");
}

void SpecificWorker::OmniRobot_setSpeedBase(float advx, float advz, float rot)
{
    advz *= 0.001;
    advx *= 0.001;

    Eigen::Vector3d input_speeds(advx, advz, rot);
    Eigen::Vector4d wheel_speeds = wheelsMatrix * input_speeds;

    std::cout << "wheelsMatrix:\n" << wheelsMatrix << std::endl;
    std::cout << "Input speeds: [" << advx << ", " << advz << ", " << rot << "]\n";
    std::cout << "Computed wheel speeds:\n" << wheel_speeds.transpose() << std::endl;    for (int i = 0; i < 4; i++)
    {
        motors[i]->setVelocity(wheel_speeds[i]);
    }
}

void SpecificWorker::OmniRobot_stopBase()
{
    for (int i = 0; i < 4; i++)
    {
        motors[i]->setVelocity(0);
    }
}

#pragma endregion OMNIROBOT_INTERFACE

#pragma region KINOVA_ARM_R_INTERFACE

bool SpecificWorker::KinovaArm_closeGripper()
{
	bool ret{};
	//implementCODE

	return ret;
}

RoboCompKinovaArm::TPose SpecificWorker::KinovaArm_getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo)
{
	RoboCompKinovaArm::TPose ret{};
	//implementCODE

	return ret;
}

RoboCompKinovaArm::TGripper SpecificWorker::KinovaArm_getGripperState()
{
	RoboCompKinovaArm::TGripper ret{};
	//implementCODE

	return ret;
}

RoboCompKinovaArm::TJoints SpecificWorker::KinovaArm_getJointsState()
{
    RoboCompKinovaArm::TJoints ret;
    ret.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

    for (int i = 0; i < 7; ++i)
    {
        RoboCompKinovaArm::TJoint joint;
        joint.id = i;

        if (kinovaArmRSensors[i])
            joint.angle = kinovaArmRSensors[i]->getValue();
        else
            joint.angle = 0.0f;

        // Not available information, for now...
        joint.velocity = 0.0f;
        joint.torque = 0.0f;
        joint.current = 0.0f;
        joint.voltage = 0.0f;
        joint.motorTemperature = 0.0f;
        joint.coreTemperature = 0.0f;

        ret.joints.push_back(joint);
    }

    return ret;
}

RoboCompKinovaArm::TToolInfo SpecificWorker::KinovaArm_getToolInfo()
{
	RoboCompKinovaArm::TToolInfo ret{};
	//implementCODE

	return ret;
}

void SpecificWorker::KinovaArm_moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles)
{
    const auto& jointAngles = angles.jointAngles;

    for (size_t i = 0; i < jointAngles.size() && i < kinovaArmRMotors.size(); ++i)
    {
        if (kinovaArmRMotors[i])
        {
            kinovaArmRMotors[i]->setPosition(jointAngles[i]);
        }
        else
        {
            std::cerr << "Motor nulo en la articulación " << i << std::endl;
        }
    }
}

void SpecificWorker::KinovaArm_moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds)
{
    const auto& jointSpeeds = speeds.jointSpeeds;

    for (size_t i = 0; i < 7 && i < jointSpeeds.size(); ++i)
    {
        if (kinovaArmRMotors[i])
        {
            kinovaArmRMotors[i]->setPosition(INFINITY);  // Desactiva el control de posición
            kinovaArmRMotors[i]->setVelocity(jointSpeeds[i]);
        }
        else
        {
            std::cerr << "Motor nulo en la articulación " << i << std::endl;
        }
    }
}

void SpecificWorker::KinovaArm_openGripper()
{
	//implementCODE

}

void SpecificWorker::KinovaArm_setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo)
{
	//implementCODE

}

#pragma endregion KINOVA_ARM_R_INTERFACE

#pragma region KINOVA_ARM_L_INTERFACE

bool SpecificWorker::KinovaArm1_closeGripper()
{
	bool ret{};
	//implementCODE

	return ret;
}

RoboCompKinovaArm::TPose SpecificWorker::KinovaArm1_getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo)
{
	RoboCompKinovaArm::TPose ret{};
	//implementCODE

	return ret;
}

RoboCompKinovaArm::TGripper SpecificWorker::KinovaArm1_getGripperState()
{
	RoboCompKinovaArm::TGripper ret{};
	//implementCODE

	return ret;
}

RoboCompKinovaArm::TJoints SpecificWorker::KinovaArm1_getJointsState()
{
    RoboCompKinovaArm::TJoints ret;
    ret.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

    for (int i = 0; i < 7; ++i)
    {
        RoboCompKinovaArm::TJoint joint;
        joint.id = i;

        if (kinovaArmLSensors[i])
            joint.angle = kinovaArmLSensors[i]->getValue();
        else
            joint.angle = 0.0f;

        // Not available information, for now...
        joint.velocity = 0.0f;
        joint.torque = 0.0f;
        joint.current = 0.0f;
        joint.voltage = 0.0f;
        joint.motorTemperature = 0.0f;
        joint.coreTemperature = 0.0f;

        ret.joints.push_back(joint);
    }

    return ret;
}

RoboCompKinovaArm::TToolInfo SpecificWorker::KinovaArm1_getToolInfo()
{
	RoboCompKinovaArm::TToolInfo ret{};
	//implementCODE

	return ret;
}

void SpecificWorker::KinovaArm1_moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles)
{
    const auto& jointAngles = angles.jointAngles;

    for (size_t i = 0; i < jointAngles.size() && i < kinovaArmLMotors.size(); ++i)
    {
        if (kinovaArmLMotors[i])
        {
            kinovaArmLMotors[i]->setPosition(jointAngles[i]);
        }
        else
        {
            std::cerr << "Motor nulo en la articulación " << i << std::endl;
        }
    }

    std::cout << "AAAAA" << std::endl;
}

void SpecificWorker::KinovaArm1_moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds)
{
    const auto& jointSpeeds = speeds.jointSpeeds;

    for (size_t i = 0; i < 7 && i < jointSpeeds.size(); ++i)
    {
        if (kinovaArmLMotors[i])
        {
            kinovaArmLMotors[i]->setPosition(INFINITY);  // Desactiva el control de posición
            kinovaArmLMotors[i]->setVelocity(jointSpeeds[i]);
        }
        else
        {
            std::cerr << "Motor nulo en la articulación " << i << std::endl;
        }
    }
}

void SpecificWorker::KinovaArm1_openGripper()
{
	//implementCODE

}

void SpecificWorker::KinovaArm1_setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo)
{
	//implementCODE

}

#pragma endregion KINOVA_ARM_L_INTERFACE

void SpecificWorker::printNotImplementedWarningMessage(const string functionName) {
    cout << "Function not implemented used: " << "[" << functionName << "]" << std::endl;
}



/**************************************/
// From the RoboCompFullPoseEstimationPub you can publish calling this methods:
// RoboCompFullPoseEstimationPub::void this->fullposeestimationpub_pubproxy->newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose)

/**************************************/
// From the RoboCompKinovaArm you can use this types:
// RoboCompKinovaArm::TPose
// RoboCompKinovaArm::TAxis
// RoboCompKinovaArm::TToolInfo
// RoboCompKinovaArm::TGripper
// RoboCompKinovaArm::TJoint
// RoboCompKinovaArm::TJoints
// RoboCompKinovaArm::TJointSpeeds
// RoboCompKinovaArm::TJointAngles

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

