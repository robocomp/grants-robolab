/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }


	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

    // 2d scene initialization
    auto target_slot =  [this](QGraphicsSceneMouseEvent *e)
    {
        qDebug() << "Lambda SLOT: " << e->scenePos();
        target_buffer.put(std::move(e->scenePos()), [/*r = robot_polygon->pos()*/](auto &&t, auto &out)
        {
            out.pos = t;
            out.pos.setX(-out.pos.x());
            //out.ang = -atan2(t.x() - r.x(), t.y() - r.y()); //target ang in the direction or line joining robot-target
        });
    };
    scene.initialize(graphicsView, target_slot, ROBOT_WIDTH, ROBOT_LONG, FILE_NAME);

    this->Period = period;
	if(this->startup_check_flag)
		this->startup_check();
	else
		timer.start(Period);

}
static float sigmoid(float x, float param){
    return 2 / (exp(-x * param) + 1) - 1;
}

Eigen::Vector2f SpecificWorker::transform_world_to_robot(
        Eigen::Vector2f target_in_world,
        float robot_angle,
        Eigen::Vector2f robot_in_world) {
    Eigen::Matrix2f rot;
    rot << cos(robot_angle), -sin(robot_angle),
            sin(robot_angle),  cos(robot_angle);
    auto target_in_robot = rot * (target_in_world - robot_in_world);
    return target_in_robot;
}

void SpecificWorker::compute() {
    static Target target;
    static bool active    = false;
    RoboCompFullPoseEstimation::FullPoseEuler pose;

    const float threshold = 1000;
    const float sigma     = -0.5 * 0.5 / log(0.3);

    try {
        Eigen::Vector2f target_in_world, robot_in_world, v_dist;
        float alpha, dist;

        if(auto t = target_buffer.try_get(); t.has_value()) {
            target = t.value();
            active = true;
        }

        pose            = fullposeestimation_proxy->getFullPoseEuler();
        alpha           = pose.rz;
        target_in_world = Eigen::Vector2f(target.pos.x(), target.pos.y());
        robot_in_world  = Eigen::Vector2f(pose.x, pose.y);
        v_dist          = transform_world_to_robot(target_in_world, alpha, robot_in_world);
        dist            = v_dist.norm();

        if (dist < threshold) {
            active = false;
            omnirobot_proxy->setSpeedBase(0, 0, 0);
            return;
        }
        if (active) {
            float beta = atan2(v_dist.x(), v_dist.y());
            float angular_speed_reduction = beta / M_PI;
            float wSpeed = sigmoid(beta, 2) * angular_speed_reduction;

            cout << "Distance " << dist << endl;

            float angular_reduction = exp( -(wSpeed * wSpeed) / sigma);
            float distance_reduction = sigmoid(dist, 1/800.);
            float vSpeed = 1000 * angular_reduction * distance_reduction;
            omnirobot_proxy->setSpeedBase(0, 1*vSpeed, 10 * wSpeed);
        }
    } catch (const Ice::Exception &ex) {
        std::cout << ex << std::endl;
    }

    scene.robot_polygon->setRotation(qRadiansToDegrees(pose.rz-M_PI));
    scene.robot_polygon->setPos(pose.x, pose.y);
}

/////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

/**************************************/
// From the RoboCompFullPoseEstimation you can call this methods:
// this->fullposeestimation_proxy->getFullPoseEuler(...)
// this->fullposeestimation_proxy->getFullPoseMatrix(...)
// this->fullposeestimation_proxy->setInitialPose(...)

/**************************************/
// From the RoboCompFullPoseEstimation you can use this types:
// RoboCompFullPoseEstimation::FullPoseMatrix
// RoboCompFullPoseEstimation::FullPoseEuler

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// this->omnirobot_proxy->correctOdometer(...)
// this->omnirobot_proxy->getBasePose(...)
// this->omnirobot_proxy->getBaseState(...)
// this->omnirobot_proxy->resetOdometer(...)
// this->omnirobot_proxy->setOdometer(...)
// this->omnirobot_proxy->setOdometerPose(...)
// this->omnirobot_proxy->setSpeedBase(...)
// this->omnirobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

