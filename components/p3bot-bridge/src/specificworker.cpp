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
		
		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
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

    // Inicializa los motores y los sensores de posición.
    const char *motorNames[4] = {"wheel2", "wheel1", "wheel4", "wheel3"};
    for (int i = 0; i < 4; i++)
    {
        motors[i] = robot->getMotor(motorNames[i]);
        positionSensors[i] = motors[i]->getPositionSensor();
        positionSensors[i]->enable(this->getPeriod("Compute"));
        motors[i]->setPosition(INFINITY); // Modo de velocidad.
        motors[i]->setVelocity(0);
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
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}



//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}

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

    // Velocidades puras en mm/s y rad/s
    double velocidad_x = 0.1; // Ejemplo: 100 mm/s
    double velocidad_y = 0.1; // Ejemplo: 150 mm/s
    double alpha = 0.075; // Ejemplo: 0.05 rad/s

    // Desviación estándar del ruido (ejemplo: 5% del valor de las velocidades)
    double ruido_stddev_x = 0.05 * velocidad_x;
    double ruido_stddev_y = 0.05 * velocidad_y;
    double ruido_stddev_alpha = 0.05 * alpha;

    RoboCompFullPoseEstimation::FullPoseEuler pose_data;

    // Posición
    pose_data.x = shadow_position[0];  // metros → mm
    pose_data.y = shadow_position[1];
    pose_data.z = shadow_position[2];

    // Orientación (Euler en radianes) 2D
    pose_data.rx = 0.0;
    pose_data.ry = 0.0;
    pose_data.rz = orientation;  // Ángulo Z ya calculado

    pose_data.vx = -rt_rotation_matrix_inv(0) + generate_noise(ruido_stddev_x);
    pose_data.vy = -rt_rotation_matrix_inv(1) + generate_noise(ruido_stddev_y);
    pose_data.vz = 0;
    pose_data.vrx = 0;
    pose_data.vry = 0;
    pose_data.vrz = shadow_velocity[5] + generate_noise(ruido_stddev_alpha);
    pose_data.timestamp = timestamp;

    this->fullposeestimationpub_pubproxy->newFullPose(pose_data);
}

double SpecificWorker::generate_noise(double stddev)
{
    std::random_device rd; // Obtiene una semilla aleatoria del hardware
    std::mt19937 gen(rd()); // Generador de números aleatorios basado en Mersenne Twister
    std::normal_distribution<> d(0, stddev); // Distribución normal con media 0 y desviación estándar stddev
    return d(gen);
}

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
    double speeds[4];

    advz *= 0.001;
    advx *= 0.001;

    speeds[0] = 1.0 / WHEEL_RADIUS * (advz + advx + (LX + LY) * rot);
    speeds[1] = 1.0 / WHEEL_RADIUS * (advz - advx - (LX + LY) * rot);
    speeds[2] = 1.0 / WHEEL_RADIUS * (advz - advx + (LX + LY) * rot);
    speeds[3] = 1.0 / WHEEL_RADIUS * (advz + advx - (LX + LY) * rot);
    printf("Speeds: vx=%.2f[m/s] vy=%.2f[m/s] ω=%.2f[rad/s]\n", advx, advz, rot);
    for (int i = 0; i < 4; i++)
    {
        motors[i]->setVelocity(speeds[i]);
    }
}

void SpecificWorker::OmniRobot_stopBase()
{
    for (int i = 0; i < 4; i++)
    {
        motors[i]->setVelocity(0);
    }
}

void SpecificWorker::printNotImplementedWarningMessage(const string functionName) {
    cout << "Function not implemented used: " << "[" << functionName << "]" << std::endl;
}



/**************************************/
// From the RoboCompFullPoseEstimationPub you can publish calling this methods:
// RoboCompFullPoseEstimationPub::void this->fullposeestimationpub_pubproxy->newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose)

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

