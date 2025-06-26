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

/**
	\brief Bridge component that connects Webots with Robocomp for P3Bot Robot
	@author Robolab Group | Sergio Eslava & Alejandro Torrejón & Jorge Castellón
*/
#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

//##################################
//##################################
//#########   INCLUDES   ###########
//##################################
//##################################
#include <genericworker.h>
#include <webots/Robot.hpp>
#include <webots/Node.hpp>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/PositionSensor.hpp>
#include <fps/fps.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>


//##################################
//##################################
//#####   OTHER DEFINITIONS   ######
//##################################
//##################################
using namespace std;
using namespace Eigen;

#define TIME_STEP 33
#define WHEEL_RADIUS 0.08
#define ROTATION_INCREMENT_COEFFICIENT 6.3 // multiplier coefficient created to adjust simulated rotation into real in Webots
#define LX 0.270  // longitudinal distance from robot's COM to wheel [m].
#define LY 0.475  // lateral distance from robot's COM to wheel [m].



/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    /**
     * \brief Constructor for SpecificWorker.
     * \param configLoader Configuration loader for the component.
     * \param tprx Tuple of proxies required for the component.
     * \param startup_check Indicates whether to perform startup checks.
     */
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);

	/**
     * \brief Destructor for SpecificWorker.
     */
	~SpecificWorker();

    // #######################
    // # KinovaArm_R interface #
    // #######################
	bool KinovaArm_closeGripper();
	RoboCompKinovaArm::TPose KinovaArm_getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo);
	RoboCompKinovaArm::TGripper KinovaArm_getGripperState();
	RoboCompKinovaArm::TJoints KinovaArm_getJointsState();
	RoboCompKinovaArm::TToolInfo KinovaArm_getToolInfo();
	void KinovaArm_moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles);
	void KinovaArm_moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds);
	void KinovaArm_openGripper();
	void KinovaArm_setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo);


    // #######################
    // # KinovaArm_L interface #
    // #######################
	bool KinovaArm1_closeGripper();
	RoboCompKinovaArm::TPose KinovaArm1_getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo);
	RoboCompKinovaArm::TGripper KinovaArm1_getGripperState();
	RoboCompKinovaArm::TJoints KinovaArm1_getJointsState();
	RoboCompKinovaArm::TToolInfo KinovaArm1_getToolInfo();
	void KinovaArm1_moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles);
	void KinovaArm1_moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds);
	void KinovaArm1_openGripper();
	void KinovaArm1_setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo);

    // #######################
    // # OMNIROBOT interface #
    // #######################
    void OmniRobot_correctOdometer(int x, int z, float alpha);
	void OmniRobot_getBasePose(int &x, int &z, float &alpha);
	void OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state);
	void OmniRobot_resetOdometer();
	void OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state);
	void OmniRobot_setOdometerPose(int x, int z, float alpha);
	void OmniRobot_setSpeedBase(float advx, float advz, float rot);
	void OmniRobot_stopBase();


public slots:

	/**
	 * \brief Initializes the worker one time.
	 */
	void initialize();

	/**
	 * \brief Main compute loop of the worker.
	 */
	void compute();

	/**
	 * \brief Handles the emergency state loop.
	 */
	void emergency();

	/**
	 * \brief Restores the component from an emergency state.
	 */
	void restore();

    /**
     * \brief Performs startup checks for the component.
     * \return An integer representing the result of the checks.
     */
	int startup_check();

private:
    /**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

    /**
     * Variables to store webots types and elements
     */
    webots::Node* robotNode;
    webots::Supervisor* robot;
    webots::Motor* motors[4];
    webots::PositionSensor* positionSensors[4];

	const double SumLxLyOverRadius = (LX + LY);
	Eigen::Matrix<double, 4, 3> wheelsMatrix;

    std::vector<webots::Motor*> kinovaArmRMotors;
    std::vector<webots::PositionSensor*> kinovaArmRSensors;

    std::vector<webots::Motor*> kinovaArmLMotors;
    std::vector<webots::PositionSensor*> kinovaArmLSensors;

    // Exact names for actuators and sensors of KinovaGen3 in Webots
    std::vector<std::string> kinovaMotorNames = {
            "Actuator1", "Actuator2", "Actuator3",
            "Actuator4", "Actuator5", "Actuator6", "Actuator7"
    };

    std::vector<std::string> kinovaSensorNames = {
            "Actuator1_sensor", "Actuator2_sensor", "Actuator3_sensor",
            "Actuator4_sensor", "Actuator5_sensor", "Actuator6_sensor", "Actuator7_sensor"
    };

    void receiving_robotSpeed(webots::Supervisor* _robot, double timestamp);
    double generateNoise(double stddev);

    void moveBothArmsWithAngle(const RoboCompKinovaArm::Angles &jointAngles, vector<webots::Motor *> armMotors);
    void moveBothArmsWithSpeed(const RoboCompKinovaArm::Speeds &jointSpeeds, vector<webots::Motor *> armMotors);

    void printNotImplementedWarningMessage(const string functionName);

    RoboCompKinovaArm::TJoints getJoints(std::vector<webots::PositionSensor *> armSensors);
};

#endif
