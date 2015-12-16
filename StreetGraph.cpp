#include <iostream>
#include <cmath>
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

void StreetGraph::computeMajorHyperstreamlines()
{
    if(mTensorField == NULL)
    {
        qCritical()<<"ERROR: Tensor field is empty";
        return;
    }
    if(mSeeds.size() == 0)
    {
        qCritical()<<"ERROR: The seed list is empty. First fill it by "
                 <<"calling createRandomSeedList or another filling function";
        return;
    }
    QSize fieldSize = mTensorField->getFieldSize();
    for(int k=0 ; k<mSeeds.size() ; k++)
    {
        // Create a node
        // Grow a road starting from this node using the tensor eigen vector
        // until one of the condition is reached
        Node& node1 = mNodes[++mLastNodeID];
        Road& road = mRoads[++mLastRoadID];

        node1.position = mSeeds[k];
        road.type = Principal;

        node1.connectedRoadIDs.push_back(mLastRoadID);
        road.nodeID1 = mLastNodeID;

        float step = mRegionSize.height()/100.0f; // Should be function of curvature

        // The road contains also the position of its extreme nodes
        // Start from the node position
        QPointF currentPosition = node1.position;
        bool stopGrowth = false;
        while(!stopGrowth)
        {
            QVector2D currentDirection;
            if(road.segments.size() != 0)
            {
                currentDirection = QVector2D(currentPosition-road.segments.last());
            }
            road.segments.push_back(currentPosition);
            // Make a function for that
            int i = round((currentPosition.y()-mBottomLeft.y())/mRegionSize.height()*
                        (fieldSize.height()-1));
            int j = round((currentPosition.x()-mBottomLeft.x())/mRegionSize.width()*
                        (fieldSize.width()-1));
            QVector2D majorDirection = mTensorField->getMajorEigenVector(i,j);
            if(QVector2D::dotProduct(majorDirection,currentDirection) < 0)
            {
                majorDirection *= -1;
            }
            QPointF nextPosition = currentPosition + (step*majorDirection).toPointF();
            stopGrowth = boundaryStoppingCondition(nextPosition);
            currentPosition = nextPosition;
        }
    }
}

bool StreetGraph::boundaryStoppingCondition(QPointF nextPosition)
{
    if(nextPosition.x() <= mBottomLeft.x()
        || nextPosition.x() >= mTopRight.x()
        || nextPosition.y() <= mBottomLeft.y()
        || nextPosition.y() >= mTopRight.y())
    {
        return true;
    }
    return false;
}

bool StreetGraph::degeneratePointStoppingCondition()
{
    Q_UNIMPLEMENTED();
    return false;
}

bool StreetGraph::loopStoppingCondition()
{
    Q_UNIMPLEMENTED();
    return false;
}

bool StreetGraph::exceedingLengthStoppingCondition()
{
    Q_UNIMPLEMENTED();
    return false;
}

bool StreetGraph::exceedingDensityStoppingCondition()
{
    Q_UNIMPLEMENTED();
    return false;
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
