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

{
}
