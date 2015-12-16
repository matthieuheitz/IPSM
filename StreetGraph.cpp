#include <iostream>
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
