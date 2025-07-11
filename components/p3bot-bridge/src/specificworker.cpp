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

    // Camera360 initialization
    camera360_1 = robot->getCamera("camera_360_1");
    camera360_2 = robot->getCamera("camera_360_2");
    if(camera360_1 && camera360_2){
        camera360_1->enable(this->getPeriod("Compute"));
        camera360_2->enable(this->getPeriod("Compute"));
    }

    // Helios Lidar initialization
    heliosLidar = robot->getLidar("helios");
    if(heliosLidar) heliosLidar->enable(this->getPeriod("Compute"));

    // Accelerometer initialization
    accelerometer = robot->getAccelerometer("accelerometer");
    if(accelerometer) accelerometer->enable(this->getPeriod("Compute"));

}


void SpecificWorker::compute()
{
    double now = robot->getTime() * 1000;

    if(robot) receiving_robotSpeed(robot, now);
    if(camera360_1 && camera360_2) receiving_camera360Data(camera360_1, camera360_2, now);
    if(heliosLidar) receiving_lidarData(heliosLidar, double_buffer_helios,  helios_delay_queue, now);


    robot->step(this->getPeriod("Compute"));
    fps.print("FPS:");
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

void SpecificWorker::receiving_camera360Data(webots::Camera* _camera1, webots::Camera* _camera2, double timestamp)
{
    RoboCompCamera360RGB::TImage newImage360;

    // Aseguramos de que ambas cámaras tienen la misma resolución, de lo contrario, deberás manejar las diferencias.
    if (_camera1->getWidth() != _camera2->getWidth() || _camera1->getHeight() != _camera2->getHeight())
    {
        std::cerr << "Error: Cameras with different resolutions." << std::endl;
        return;
    }

    // Timestamp calculation
    newImage360.timestamp = timestamp;

    // La resolución de la nueva imagen será el doble en el ancho ya que estamos combinando las dos imágenes.
    newImage360.width = 2 * _camera1->getWidth();
    newImage360.height = _camera1->getHeight();

    // Establecer el periodo real del compute de refresco de la imagen en milisegundos.
    newImage360.period = fps.get_period();

    const unsigned char* webotsImageData1 = _camera1->getImage();
    const unsigned char* webotsImageData2 = _camera2->getImage();
    cv::Mat img_1 = cv::Mat(cv::Size(_camera1->getWidth(), _camera1->getHeight()), CV_8UC4);
    cv::Mat img_2 = cv::Mat(cv::Size(_camera2->getWidth(), _camera2->getHeight()), CV_8UC4);

    img_1.data = (uchar *)webotsImageData1;
    cv::cvtColor(img_1, img_1, cv::COLOR_RGBA2RGB);

    img_2.data = (uchar *)webotsImageData2;
    cv::cvtColor(img_2, img_2, cv::COLOR_RGBA2RGB);

    cv::Mat img_final = cv::Mat(cv::Size(_camera1->getWidth()*2, _camera1->getHeight()), CV_8UC3);

    img_1.copyTo(img_final(cv::Rect(0, 0, _camera1->getWidth(), _camera1->getHeight())));
    img_2.copyTo(img_final(cv::Rect(_camera1->getWidth(), 0, _camera1->getWidth(), _camera2->getHeight())));

    // Asignar la imagen RGB 360 al tipo TImage de Robocomp
    newImage360.image.resize(img_final.total()*img_final.elemSize());
    memcpy(&newImage360.image[0], img_final.data, img_final.total()*img_final.elemSize());

    //newImage360.image = rgbImage360;
    newImage360.compressed = false;

    if(pars.delay)
        camera_queue.push(newImage360);

    // Asignamo el resultado final al atributo de clase (si tienes uno).
    double_buffer_360.put(std::move(newImage360));

    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - now).count() << std::endl;
}

void SpecificWorker::receiving_lidarData(webots::Lidar* _lidar, DoubleBuffer<RoboCompLidar3D::TData, RoboCompLidar3D::TData> &_lidar3dData, FixedSizeDeque<RoboCompLidar3D::TData>& delay_queue, double timestamp)
{
    if (!_lidar) { std::cout << "No lidar available." << std::endl; return; }

    const float *rangeImage = _lidar->getRangeImage();
    int horizontalResolution = _lidar->getHorizontalResolution();
    int verticalResolution = _lidar->getNumberOfLayers();
    double fov = _lidar->getFov();
    double angleResolution = fov / horizontalResolution;
    float verticalFov = 2.8;

    RoboCompLidar3D::TData newLidar3dData;

    // General Lidar values
    newLidar3dData.timestamp = timestamp;
    newLidar3dData.period = fps.get_period();

    if(!rangeImage) { std::cout << "Lidar data empty." << std::endl; return; }

    for (int j = 0; j < verticalResolution; ++j) {
        for (int i = 0; i < horizontalResolution; ++i) {
            int index = j * horizontalResolution + i;

            //distance meters to millimeters
            const float distance = rangeImage[index]; //Meters

            //TODO rotacion del eje y con el M_PI, solucionar
            float horizontalAngle = M_PI - i * angleResolution - fov / 2;

            float verticalAngle = M_PI + j * (verticalFov / verticalResolution) - verticalFov / 2;

            //Calculate Cartesian co-ordinates and rectify axis positions
            Eigen::Vector3f lidar_point(
                    distance * cos(horizontalAngle) * cos(verticalAngle),
                    distance * sin(horizontalAngle) * cos(verticalAngle),
                    distance * sin(verticalAngle));

            if (not (std::isinf(lidar_point.x()) or std::isinf(lidar_point.y()) or std::isinf(lidar_point.z())))
            {
                if (not (verticalAngle > 4.10152 or verticalAngle <2.87979)) //down limit+, uper limit-, horizon line is PI
                {
                    RoboCompLidar3D::TPoint point;

                    point.x = lidar_point.x();
                    point.y = lidar_point.y();
                    point.z = lidar_point.z();

                    point.r = lidar_point.norm();  // distancia radial
                    point.phi = horizontalAngle;  // ángulo horizontal // -x para hacer [PI, -PI] y no [-PI, PI]
                    point.theta = verticalAngle;  // ángulo vertical
                    point.distance2d = std::hypot(lidar_point.x(),lidar_point.y());  // distancia en el plano xy

                    newLidar3dData.points.push_back(point);
                }
            }
        }
    }
    //Points order to angles
    std::ranges::sort(newLidar3dData.points, {}, &RoboCompLidar3D::TPoint::phi);

    //Is it necessary to use two lidar queues? One for each lidaR?
    if(pars.delay)
        delay_queue.push(newLidar3dData);

    _lidar3dData.put(std::move(newLidar3dData));
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
    return getJoints(kinovaArmRSensors, kinovaArmRMotors);
}

RoboCompKinovaArm::TToolInfo SpecificWorker::KinovaArm_getToolInfo()
{
	RoboCompKinovaArm::TToolInfo ret{};
	//implementCODE

	return ret;
}

void SpecificWorker::KinovaArm_moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles)
{
    moveBothArmsWithAngle(angles.jointAngles, kinovaArmRMotors);
}

void SpecificWorker::KinovaArm_moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds)
{
    moveBothArmsWithSpeed(speeds.jointSpeeds, kinovaArmRMotors);
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
    return getJoints(kinovaArmLSensors, kinovaArmLMotors);
}

RoboCompKinovaArm::TToolInfo SpecificWorker::KinovaArm1_getToolInfo()
{
	RoboCompKinovaArm::TToolInfo ret{};
	//implementCODE

	return ret;
}

void SpecificWorker::KinovaArm1_moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles)
{
    moveBothArmsWithAngle(angles.jointAngles, kinovaArmLMotors);
}

void SpecificWorker::KinovaArm1_moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds)
{
    moveBothArmsWithSpeed(speeds.jointSpeeds, kinovaArmLMotors);
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

#pragma region CAMERA360RGB_INTERFACE

RoboCompCamera360RGB::TImage SpecificWorker::Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)
{
    if(pars.delay)
    {
        if(camera_queue.full())
            return camera_queue.back();
    }

    return double_buffer_360.get_idemp();
}

#pragma endregion CAMERA360RGB_INTERFACE

#pragma region LIDAR3D_INTERFACE

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor)
{
    return (pars.delay && helios_delay_queue.full()) ? helios_delay_queue.back() : double_buffer_helios.get_idemp();
}

RoboCompLidar3D::TDataImage SpecificWorker::Lidar3D_getLidarDataArrayProyectedInImage(std::string name)
{
    RoboCompLidar3D::TDataImage ret{};
    printNotImplementedWarningMessage("Lidar3D_getLidarDataArrayProyectedInImage");
    return ret;
}

RoboCompLidar3D::TDataCategory SpecificWorker::Lidar3D_getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp)
{
    RoboCompLidar3D::TDataCategory ret{};
    printNotImplementedWarningMessage("Lidar3D_getLidarDataByCategory");
    return ret;
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataProyectedInImage(std::string name)
{
	RoboCompLidar3D::TData ret{};
    printNotImplementedWarningMessage("Lidar3D_getLidarDataProyectedInImage");
	return ret;
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor)
{
	RoboCompLidar3D::TData ret{};
    printNotImplementedWarningMessage("Lidar3D_getLidarDataWithThreshold2d");
	return ret;
}

#pragma endregion LIDAR3D_INTERFACE


void SpecificWorker::moveBothArmsWithAngle(const RoboCompKinovaArm::Angles &jointAngles,
                                           std::vector<webots::Motor *> armMotors) {
    for (size_t i = 0; i < jointAngles.size() && i < armMotors.size(); ++i)
    {
        if (armMotors[i])
        {
            armMotors[i]->setPosition(jointAngles[i]);
            armMotors[i]->setVelocity(0.25);
        
        }
        else
        {
            cerr << "Motor nulo en la articulación " << i << endl;
        }
    }
}
void SpecificWorker::moveBothArmsWithSpeed(const RoboCompKinovaArm::Speeds &jointSpeeds,
                                           std::vector<webots::Motor *> armMotors) {
    for (size_t i = 0; i < 7 && i < jointSpeeds.size(); ++i)
    {
        if (armMotors[i])
        {
            armMotors[i]->setPosition(INFINITY);  // Desactiva el control de posición
            armMotors[i]->setVelocity(jointSpeeds[i]);
        }
        else
        {
            cerr << "Motor nulo en la articulación " << i << endl;
        }
    }
}
RoboCompKinovaArm::TJoints SpecificWorker::getJoints(std::vector<webots::PositionSensor *> armSensors, std::vector<webots::Motor *> armMotors ) {
    RoboCompKinovaArm::TJoints ret;
    ret.timestamp = chrono::duration_cast<chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

    for (int i = 0; i < 7; ++i)
    {
        RoboCompKinovaArm::TJoint joint;
        joint.id = i;

        if (armSensors[i]){
            joint.angle = armSensors[i]->getValue();
            joint.velocity = armMotors[i]->getVelocity();
        }
        else{
            joint.angle = 0.0f;
            joint.velocity = 0.0f;
        }

        // Not available information, for now...

        joint.torque = 0.0f;
        joint.current = 0.0f;
        joint.voltage = 0.0f;
        joint.motorTemperature = 0.0f;
        joint.coreTemperature = 0.0f;

        ret.joints.push_back(joint);
    }
    return ret;
}

void SpecificWorker::printNotImplementedWarningMessage(const string functionName) {
    cout << "Function not implemented used: " << "[" << functionName << "]" << std::endl;
}



/**************************************/
// From the RoboCompFullPoseEstimationPub you can publish calling this methods:
// RoboCompFullPoseEstimationPub::void this->fullposeestimationpub_pubproxy->newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose)

/**************************************/
// From the RoboCompCamera360RGB you can use this types:
// RoboCompCamera360RGB::TRoi
// RoboCompCamera360RGB::TImage

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
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

