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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include "myscene.h"
#include <doublebuffer/DoubleBuffer.h>
#include <Eigen/Dense>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
	void compute();
	int startup_check();
	void initialize(int period);

private:
	bool startup_check_flag;

    // Target
    struct Target
    {
        Eigen::Vector2f pos;
        float ang;
        void set_new_value(const Target &t) { pos = t.pos; ang = t.ang; active = true; };
        bool active = false;
    public:
        void set_active(bool a) { active = a;};
        bool is_active() const { return active;}
    };

    // target
    DoubleBuffer<QPointF, Target> target_buffer;

    //robot
    const float ROBOT_WIDTH = 400;
    const float ROBOT_LONG = 450;
    const float MAX_ADVANCE_SPEED = 800;
    const std::string FILE_NAME = "../../../etc/viriato.simscene.json";

    // 2d scene
    Robot2DScene scene;
    void draw_target(Robot2DScene *scene, const RoboCompFullPoseEstimation::FullPoseEuler &robot_pose, const Target &target);
    //void draw_laser(Robot2DScene *scene, QPolygonF &laser_poly); // robot coordinates

    RoboCompFullPoseEstimation::FullPoseEuler read_base_state();
    Eigen::Vector2f transform_from_world_to_robot(const RoboCompFullPoseEstimation::FullPoseEuler &robot_pose, const Target &target);
    float gaussian(float value, float xValue, float yValue, float min);
    float sigmoid(float t);
};

#endif
