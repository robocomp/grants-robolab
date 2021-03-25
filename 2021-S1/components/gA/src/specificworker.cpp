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
            qInfo() << "New target " << t;
            //out.ang = -atan2(t.x() - r.x(), t.y() - r.y()); //target ang in the direction or line joining robot-target
        });
    };
    auto sd = scene.get_dimensions();
    auto gd = Grid<>::Dimensions { .TILE_SIZE = sd.TILE_SIZE, .HMIN = sd.HMIN, .VMIN = sd.VMIN, .WIDTH = sd.WIDTH, .HEIGHT = sd.HEIGHT };
    scene.initialize(graphicsView, target_slot, ROBOT_WIDTH, ROBOT_LONG, FILE_NAME);
    grid.initialize(&scene, gd);
    grid.fill_with_obstacles(scene.get_obstacles());

    for (auto obstacle : scene.get_obstacles()) {
        for (auto point : obstacle) {
            cout << "x = " << point.x() << " y = " << point.y() << endl;
        }
        cout << "-----------" << endl;
    }

    this->Period = period;
    if(this->startup_check_flag)
        this->startup_check();
    else
        timer.start(Period);

}

void SpecificWorker::compute()
{
    static Target target;

    auto robot_pose = read_base_state();
    // check buffer
    if(auto t = target_buffer.try_get(); t.has_value())
    {
        target.set_new_value(t.value());
        auto path = grid.computePath(
                QPointF(robot_pose.x, robot_pose.y),
                QPointF(target.pos[0], target.pos[1]));
        draw_target(&scene, robot_pose, target);
        grid.draw_path(&scene, path);
    }
    // check target
    if(not target.is_active()) return;
    float dist_to_target = (target.pos - Eigen::Vector2f(robot_pose.x, robot_pose.y)).norm();
    // check distance to target
    if( dist_to_target < 100) // arrived
    {
        target.set_active(false);
        qInfo() << "Arrived to target!";
        try { omnirobot_proxy->setSpeedBase(0.f, 0.f, 0.f);}
        catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
    }
    else    // go for it
    {
        // transform target to robot coordinates
        auto target_in_robot = transform_from_world_to_robot(robot_pose, target);
        float ang_to_target_from_robot = atan2(target_in_robot.x(), target_in_robot.y());
        float vadv = 0.f; float vrot = 0.f;
        if (fabs(ang_to_target_from_robot) < 0.05)
            ang_to_target_from_robot = 0.0;  // to avoid oscillations
        // control equations
        vrot = 10 * ang_to_target_from_robot;
        vrot = std::clamp(vrot, -15.f, 15.f);
        qInfo() << "error ang " << ang_to_target_from_robot  << "vrot " << vrot << "dist " << dist_to_target;
        vadv = MAX_ADVANCE_SPEED * gaussian(vrot, 0.4, 0.2, 0.0);

        try { omnirobot_proxy->setSpeedBase(0, vadv, vrot);}
        catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
    }
}

Eigen::Vector2f SpecificWorker::transform_from_world_to_robot(const RoboCompFullPoseEstimation::FullPoseEuler &robot_pose, const Target &target)
{
    Eigen::Matrix2f rot;
    rot << cos(robot_pose.rz), -sin(robot_pose.rz),
            sin(robot_pose.rz), cos(robot_pose.rz);
    auto target_in_robot = rot.transpose() * (target.pos - Eigen::Vector2f(robot_pose.x, robot_pose.y));
    return target_in_robot;
}

float SpecificWorker::sigmoid(float t)
{
    return 2.f / (1.f + exp(-t * 1.4)) - 1.f;
}

float SpecificWorker::gaussian(float value, float xValue, float yValue, float min)
{
    if (yValue <= 0)
        return 1.f;
    float landa = -fabs(xValue) / log(yValue);
    float res = exp(-fabs(value) / landa);
    return std::max(res, min);
}

RoboCompFullPoseEstimation::FullPoseEuler SpecificWorker::read_base_state()
{
    RoboCompFullPoseEstimation::FullPoseEuler pose;
    try
    {
        pose = fullposeestimation_proxy->getFullPoseEuler();
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}
    pose.rz -= M_PI;
    //qInfo() << pose.x << pose.y << pose.rz;
    scene.robot_polygon->setRotation(qRadiansToDegrees(pose.rz));
    scene.robot_polygon->setPos(pose.x, pose.y);
    return pose;
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
