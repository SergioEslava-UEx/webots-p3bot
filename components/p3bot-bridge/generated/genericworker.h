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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include <stdint.h>
#include <grafcetStep/GRAFCETStep.h>
#include <ConfigLoader/ConfigLoader.h>
#include <QStateMachine>
#include <QEvent>
#include <QString>
#include <functional>
#include <atomic>
#include <QtCore>
#include <variant>
#include <unordered_map>


#include <Camera360RGB.h>
#include <FullPoseEstimation.h>
#include <FullPoseEstimationPub.h>
#include <GenericBase.h>
#include <JoystickAdapter.h>
#include <KinovaArm.h>
#include <Lidar3D.h>
#include <OmniRobot.h>

#define BASIC_PERIOD 100

using TuplePrx = std::tuple<RoboCompFullPoseEstimationPub::FullPoseEstimationPubPrxPtr>;


class GenericWorker : public QObject
{
Q_OBJECT
public:
	GenericWorker(const ConfigLoader& configLoader, TuplePrx tprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();

	void setPeriod(const std::string& state, int period);
	int getPeriod(const std::string& state);

	QStateMachine statemachine;
	QTimer hibernationChecker;
	std::atomic_bool hibernation = false;


	RoboCompFullPoseEstimationPub::FullPoseEstimationPubPrxPtr fullposeestimationpub_pubproxy;

	virtual RoboCompCamera360RGB::TImage Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight) = 0;

	virtual bool KinovaArm_closeGripper() = 0;
	virtual RoboCompKinovaArm::TPose KinovaArm_getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo) = 0;
	virtual RoboCompKinovaArm::TGripper KinovaArm_getGripperState() = 0;
	virtual RoboCompKinovaArm::TJoints KinovaArm_getJointsState() = 0;
	virtual RoboCompKinovaArm::TToolInfo KinovaArm_getToolInfo() = 0;
	virtual void KinovaArm_moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles) = 0;
	virtual void KinovaArm_moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds) = 0;
	virtual void KinovaArm_openGripper() = 0;
	virtual void KinovaArm_setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo) = 0;
	virtual bool KinovaArm_setGripperPos(float pos) = 0;

	virtual bool KinovaArm1_closeGripper() = 0;
	virtual RoboCompKinovaArm::TPose KinovaArm1_getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo) = 0;
	virtual RoboCompKinovaArm::TGripper KinovaArm1_getGripperState() = 0;
	virtual RoboCompKinovaArm::TJoints KinovaArm1_getJointsState() = 0;
	virtual RoboCompKinovaArm::TToolInfo KinovaArm1_getToolInfo() = 0;
	virtual void KinovaArm1_moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles) = 0;
	virtual void KinovaArm1_moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds) = 0;
	virtual void KinovaArm1_openGripper() = 0;
	virtual void KinovaArm1_setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo) = 0;
	virtual bool KinovaArm1_setGripperPos(float pos) = 0;

	virtual RoboCompLidar3D::TData Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor) = 0;
	virtual RoboCompLidar3D::TDataImage Lidar3D_getLidarDataArrayProyectedInImage(std::string name) = 0;
	virtual RoboCompLidar3D::TDataCategory Lidar3D_getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp) = 0;
	virtual RoboCompLidar3D::TData Lidar3D_getLidarDataProyectedInImage(std::string name) = 0;
	virtual RoboCompLidar3D::TData Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor) = 0;

	virtual void OmniRobot_correctOdometer(int x, int z, float alpha) = 0;
	virtual void OmniRobot_getBasePose(int &x, int &z, float &alpha) = 0;
	virtual void OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state) = 0;
	virtual void OmniRobot_resetOdometer() = 0;
	virtual void OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state) = 0;
	virtual void OmniRobot_setOdometerPose(int x, int z, float alpha) = 0;
	virtual void OmniRobot_setSpeedBase(float advx, float advz, float rot) = 0;
	virtual void OmniRobot_stopBase() = 0;

	virtual void JoystickAdapter_sendData (RoboCompJoystickAdapter::TData data) = 0;


protected:
	std::unordered_map<std::string, std::unique_ptr<GRAFCETStep>> states;
	ConfigLoader configLoader;




private:

public slots:
	virtual void initialize() = 0;
	virtual void compute() = 0;
	virtual void emergency() = 0;
	virtual void restore() = 0;
	void hibernationCheck();
	void hibernationTick();
	
signals:
	void kill();
	void goToEmergency();
	void goToRestore();
};

#endif
