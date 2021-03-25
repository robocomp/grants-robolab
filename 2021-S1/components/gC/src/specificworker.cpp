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
            out.pos.x() = t.x();
            out.pos.y() = t.y();
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

void SpecificWorker::compute() {
    static float s = -pow(0.5, 2) / log(0.1);
    static int MAX_ADV = 1000;
    static Target target;
    RoboCompFullPoseEstimation::FullPoseEuler pose;

    try { pose = fullposeestimation_proxy->getFullPoseEuler(); }
    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }

    scene.robot_polygon->setRotation(qRadiansToDegrees(pose.rz));
    scene.robot_polygon->setPos(pose.x, pose.y);
    //check buffer
    if (auto t = target_buffer.try_get(); t.has_value()) {
        target.set_new_value(t.value()) ;
        draw_target(&scene, pose, target);
    }
    //check target
    if(not target.is_active()) return;
    auto robot_in_world = Eigen::Vector2f{pose.x, pose.y};
    auto dist = (target.pos-robot_in_world).norm();
    if (dist > 100) {
        auto vector = transform_world_to_robot(target.pos, pose.rz, robot_in_world);
        float ang = atan2(vector[0], vector[1]);
        float vang;
        if (fabs(ang) < 0.05)
            ang = 0;

        vang = 3.f * (2.f / (1.f + exp(-ang)) - 1.f);
        auto adv = MAX_ADV * exp(-(pow(vang, 2) / s)) * aprox(dist);
        try { omnirobot_proxy->setSpeedBase(0, 0, vang); }
        catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
        qInfo() << "error ang " << ang  << "vrot " << vang << "dist " << dist;
    }
    else {
        target.set_active(false);
        try { omnirobot_proxy->setSpeedBase(0.f, 0.f, 0.f); }
        catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    }



}
Eigen::Vector2f SpecificWorker::transform_world_to_robot(Eigen::Vector2f target_in_world, float robot_angle, Eigen::Vector2f robot_in_world)
{
    Eigen::Matrix2f rot;
    rot << cos(robot_angle), sin(robot_angle),
            -sin(robot_angle), cos(robot_angle);
    auto target_in_robot = rot.transpose()* (target_in_world - robot_in_world);
    return target_in_robot;
}
float SpecificWorker::aprox(float x)
{
    float y;
    if(x<=0){
        y=0;
    }
    else if(x<1000){
        y=x*pow(10,-3);
    }
    else{
        y=1;
    }
    return y;
}
void SpecificWorker::draw_target(Robot2DScene *scene, const RoboCompFullPoseEstimation::FullPoseEuler &robot, const Target &target)
{
    static QGraphicsEllipseItem *target_draw = nullptr;

    if (target_draw) scene->removeItem(target_draw);
    target_draw = scene->addEllipse(target.pos.x() - 50, target.pos.y() - 50, 100, 100, QPen(QColor("green")), QBrush(QColor("green")));
    // angular reference obtained from line joinning robot an target when  clicking
    float tr_x = target.pos.x() - robot.x;
    float tr_y = target.pos.y() - robot.y;
    float ref_ang = -atan2(tr_x, tr_y);   // signo menos para tener ángulos respecto a Y CCW
    auto ex = target.pos.x() + 350 * sin(-ref_ang);
    auto ey = target.pos.y() + 350 * cos(-ref_ang);  //OJO signos porque el ang está respecto a Y CCW
    auto line = scene->addLine(target.pos.x(), target.pos.y(), ex, ey, QPen(QBrush(QColor("green")), 20));
    line->setParentItem(target_draw);
    auto ball = scene->addEllipse(ex - 25, ey - 25, 50, 50, QPen(QColor("green")), QBrush(QColor("green")));
    ball->setParentItem(target_draw);
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

