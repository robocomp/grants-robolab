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
            out.pos.setX(-t.x());
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

void SpecificWorker::compute()
{
    RoboCompFullPoseEstimation::FullPoseEuler pose;
    static bool activo;
    Target target;

    try
    {
        pose = fullposeestimation_proxy->getFullPoseEuler();
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}

        if(auto t = target_buffer.try_get(); t.has_value())
        {
            activo = true;
            target = t.value();
        }

        float dist = sqrt(pow(pose.x - target.pos.x(), 2) + pow(pose.y - target.pos.y(), 2));

        if(dist < 100){
            activo=false;
            omnirobot_proxy->stopBase();
            return;
        }

        if(activo){

            auto tr = transform_world_to_robot(Eigen::Vector2f{target.pos.x(), target.pos.y()}, pose.rz, Eigen::Vector2f{pose.x, pose.y});
            //qInfo() << tr.x() << tr.y();
            float rot_speed;
            float ang= atan2(tr.x(),tr.y());
            if (fabs(ang)<0.05) rot_speed=0;
            else rot_speed=sigmoide(ang);
            float adv_speed=1000*gauss(rot_speed)*fdist(dist);

            try
            {
                omnirobot_proxy->setSpeedBase(0, 0, rot_speed);
            }
            catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}
        }



    scene.robot_polygon->setRotation(qRadiansToDegrees(pose.rz-M_PI));
    scene.robot_polygon->setPos(pose.x, pose.y);

}


Eigen::Vector2f SpecificWorker::transform_world_to_robot(Eigen::Vector2f target_in_world, float robot_angle, Eigen::Vector2f robot_in_world)
{
    // Inicializamos una matriz de 2x2 en sentido horario.
    Eigen::Matrix2f rot;


    rot <<  cos(robot_angle), sin(robot_angle),
            -sin(robot_angle), cos(robot_angle);

    auto target_in_robot = rot.transpose()* (target_in_world - robot_in_world);
    // calculation of angle of target_in_robot wrt robot’ coordinate system
    float target_angle_in_robot = atan2(target_in_robot[0], target_in_robot[1]);
    return target_in_robot;
}

float SpecificWorker::sigmoide(float x)
{
    return (2/(1+pow(M_E,-x))-1);
}

float SpecificWorker::gauss(float x)
{
    return pow(M_E,(-pow(x,2)/0.1085736205));
}

float SpecificWorker::fdist(float x)
{
    float res;
    if (x>100) res=1;
    else res=x/1000;
    return res;
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

