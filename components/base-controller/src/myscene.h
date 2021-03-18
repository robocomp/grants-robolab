/*
 *    Copyright (C) 2020 by YOUR NAME HERE
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

#ifndef ROBOT2DSCENE_H
#define ROBOT2DSCENE_H

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QPolygonF>
#include <QGraphicsSceneMouseEvent>
#include <doublebuffer/DoubleBuffer.h>
#include <QGraphicsRectItem>

class Robot2DScene : public QGraphicsScene
{
    Q_OBJECT
    public:
        QGraphicsPolygonItem *robot_polygon, *robot_polygon_projected;
        using Dimensions = struct{ float HMIN = -2500, VMIN = -2500, WIDTH = 5000, HEIGHT = 5000, TILE_SIZE = 40; };
        Dimensions get_dimensions() const { return dim; };
        std::vector<QPolygonF> get_obstacles() const { return obstacles; };

    signals:
        void new_target(QGraphicsSceneMouseEvent *);

    protected:
        void mousePressEvent(QGraphicsSceneMouseEvent *event)
        {
            if(event->buttons() == Qt::LeftButton)
                emit new_target(event);
        }

    public:
        template<typename T>
        QGraphicsItem*  initialize(QGraphicsView *graphicsView,
                        T &target_slot,
                        float ROBOT_WIDTH,
                        float ROBOT_LONG,
                        std::string filename)
        {
            graphicsView->setScene(this);
            graphicsView->setMinimumSize(400,400);
            dim = initializeWorld(filename);
            this->setSceneRect(dim.HMIN, dim.VMIN, dim.WIDTH, dim.HEIGHT);
            qInfo() << this->sceneRect();
            graphicsView->scale(1, -1);
            graphicsView->fitInView(this->sceneRect(), Qt::KeepAspectRatio);
            graphicsView->show();

            //robot
            auto robotXWidth = ROBOT_WIDTH;
            auto robotZLong = ROBOT_LONG;
            QPolygonF poly2;
            poly2   << QPointF(-robotXWidth/2, -robotZLong/2)
                    << QPointF(-robotXWidth/2, robotZLong/2)
                    << QPointF(robotXWidth/2, robotZLong/2)
                    << QPointF(robotXWidth/2, -robotZLong/2);
            QPolygonF marca(QRectF(-25,-25, 50, 50));
            QColor rcp("DarkRed"); rcp.setAlpha(80);
            QColor rc("MAgenta"); rc.setAlpha(80);
            robot_polygon = this->addPolygon(poly2, QPen(QColor("DarkRed")), QBrush(rc));
            auto marca_polygon = this->addPolygon(marca, QPen(QColor("White")), QBrush("White"));
            marca_polygon->setParentItem(robot_polygon);
            marca_polygon->setPos (QPointF(0, robotZLong/2 - 40));
            robot_polygon->setZValue(5);
            robot_polygon->setPos(0, 0);
            //robot_polygon_projected = this-> addPolygon(poly2, QPen(QColor("Magenta")), QBrush(rcp));
            //robot_polygon_projected->setPos(0,0);
            //robot_polygon_projected->setZValue(4);

            connect(this, &Robot2DScene::new_target, this, target_slot);
            return robot_polygon;
        }

    private:
        std::vector<QGraphicsItem *> boxes;
        std::vector<QPolygonF> obstacles;
        Dimensions dim;

        //load world model from file
        Dimensions initializeWorld(const std::string &FILE_NAME)
        {
            QString val;
            QFile file(QString::fromStdString(FILE_NAME));
            if (not file.open(QIODevice::ReadOnly | QIODevice::Text))
            {
                qDebug() << "Error reading world file, check config params:" << QString::fromStdString(FILE_NAME);
                exit(-1);
            }
            val = file.readAll();
            file.close();
            QJsonDocument doc = QJsonDocument::fromJson(val.toUtf8());
            QJsonObject jObject = doc.object();
            QVariantMap mainMap = jObject.toVariantMap();

            //load dimensions
            QVariantMap dim = mainMap[QString("dimensions")].toMap();
            Dimensions dimensions{dim["LEFT"].toFloat(), dim["BOTTOM"].toFloat(), dim["WIDTH"].toFloat(), dim["HEIGHT"].toFloat(), dim["TILESIZE"].toFloat()};
            //load tables
            QVariantMap tables = mainMap[QString("tables")].toMap();
            for (auto &t : tables)
            {
                QVariantList object = t.toList();
                auto box = this->addRect(QRectF(-object[2].toFloat() / 2, -object[3].toFloat() / 2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("SandyBrown")), QBrush(QColor("SandyBrown")));
                box->setPos(object[4].toFloat(), object[5].toFloat());
                box->setRotation(object[6].toFloat());
                boxes.push_back(box);
            }
            //load roundtables
            QVariantMap rtables = mainMap[QString("roundTables")].toMap();
            for (auto &t : rtables)
            {
                QVariantList object = t.toList();
                auto box = this->addEllipse(QRectF(-object[2].toFloat() / 2, -object[3].toFloat() / 2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("Khaki")), QBrush(QColor("Khaki")));
                box->setPos(object[4].toFloat(), object[5].toFloat());
                boxes.push_back(box);
            }
            //load walls
            QVariantMap walls = mainMap[QString("walls")].toMap();
            for (auto &t : walls)
            {
                QVariantList object = t.toList();
                auto box = this->addRect(QRectF(-object[2].toFloat() / 2, -object[3].toFloat() / 2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("Green")), QBrush(QColor("Green")));
                box->setPos(object[4].toFloat(), object[5].toFloat());
                box->setRotation(object[6].toFloat()*180/M_PI);
                boxes.push_back(box);
                obstacles.emplace_back( box->mapToScene(QPolygonF(box->rect())));
            }

            //load points
            QVariantMap points = mainMap[QString("points")].toMap();
            for (auto &t : points)
            {
                QVariantList object = t.toList();
                auto box = this->addRect(QRectF(-object[2].toFloat() / 2, -object[3].toFloat() / 2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("Brown")), QBrush(QColor("Brown")));
                box->setPos(object[4].toFloat(), object[5].toFloat());
                box->setRotation(object[6].toFloat()*180/M_PI);
                boxes.push_back(box);
            }
            //load boxes
            QVariantMap cajas = mainMap[QString("boxes")].toMap();
            for (auto &t : cajas)
            {
                QVariantList object = t.toList();
                auto box = this->addRect(QRectF(-object[2].toFloat() / 2, -object[3].toFloat() / 2, object[2].toFloat(), object[3].toFloat()), QPen(QColor("Brown")), QBrush(QColor("Orange")));
                box->setPos(object[4].toFloat(), object[5].toFloat());
                box->setRotation(object[6].toFloat()*180/M_PI);
                box->setFlag(QGraphicsItem::ItemIsMovable);
                boxes.push_back(box);
            }
            return dimensions;
        }
};

#endif
