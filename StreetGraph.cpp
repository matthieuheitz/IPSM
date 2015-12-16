#include <iostream>
#include <QDateTime>

#include "StreetGraph.h"

StreetGraph::StreetGraph(QPointF bottomLeft, QPointF topRight, TensorField *field, QObject *parent) :
    QObject(parent), mTensorField(field), mBottomLeft(bottomLeft), mTopRight(topRight)
{
    mRegionSize.rwidth() = (topRight-bottomLeft).x();
    mRegionSize.rheight() = (topRight-bottomLeft).y();
    qDebug()<<"Region size = "<<mRegionSize.width()<<", "
             <<mRegionSize.height();
    mLastNodeID = 0;
    mLastRoadID = 0;
}

void StreetGraph::createRandomSeedList(int numberOfSeeds)
{
    qsrand(QDateTime::currentDateTime().toTime_t ());
    if(mSeeds.size() != 0)
    {
        qDebug()<<"Clearing seed list before filling it";
        mSeeds.clear();
    }
    for(int i=0 ; i < numberOfSeeds ; i++)
    {
        float randX = qrand()/(float)RAND_MAX;
        float randY = qrand()/(float)RAND_MAX;
        qDebug()<<"Rand = "<<randX<<", "<<randY;
        QPointF seed(mBottomLeft.x() + randX*mRegionSize.width(),
                     mBottomLeft.y() + randY*mRegionSize.height());
        mSeeds.push_back(seed);
    }
}

std::ostream& operator<<(std::ostream& out, const Road r)
{
    out<<"Road type = ";
    out<<(r.type == Principal ? "Principal" : "Secondary");
    out<<std::endl;
    out<<"Path = ("<<r.segments.size()<<" points)"<<std::endl;
    QVector<QPointF>::const_iterator it = r.segments.begin(), it_end = r.segments.end();
    for(; it != it_end ; it++)
    {
        out<<"("<<it->x()<<","<<it->y()<<")"<<std::endl;
    }
    return out;
}

std::ostream& operator<<(std::ostream& out, const Node n)
{
    out<<"Node position = "<<n.position<<std::endl;
    out<<"Number of connected roads = "<<n.connectedRoadIDs.size()<<std::endl;
    return out;
}

std::ostream& operator<<(std::ostream& out, const QPointF p)
{
    out<<"("<<p.x()<<","<<p.y()<<")"<<std::endl;
    return out;
}
