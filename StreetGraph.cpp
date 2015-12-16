#include <iostream>
#include <cmath>
#include <QPainter>
#include <QDateTime>

#include "StreetGraph.h"

StreetGraph::StreetGraph(QPointF bottomLeft, QPointF topRight, TensorField *field, float distSeparation, QObject *parent) :
    QObject(parent), mTensorField(field), mBottomLeft(bottomLeft), mTopRight(topRight), mDistSeparation(distSeparation)
{
    mRegionSize.rwidth() = (topRight-bottomLeft).x();
    mRegionSize.rheight() = (topRight-bottomLeft).y();
    qDebug()<<"Region size = "<<mRegionSize.width()<<", "
             <<mRegionSize.height();
    mLastNodeID = 0;
    mLastRoadID = 0;
}

void StreetGraph::createRandomSeedList(int numberOfSeeds, bool append)
{
    if(!append)
    {
        mSeeds.clear();
    }
    qsrand(QDateTime::currentDateTime().toTime_t());
    for(int i=0 ; i < numberOfSeeds ; i++)
    {
        float randX = qrand()/(float)RAND_MAX;
        float randY = qrand()/(float)RAND_MAX;
        QPointF seed(mBottomLeft.x() + randX*mRegionSize.width(),
                     mBottomLeft.y() + randY*mRegionSize.height());
        mSeeds.push_back(seed);
    }
}

void StreetGraph::computeMajorHyperstreamlines(bool clearStorage)
{
    if(clearStorage)
    {
        clearStoredStreetGraph();
    }
    if(mTensorField == NULL)
    {
        qCritical()<<"ERROR: Tensor field is empty";
        return;
    }
    // Generate the seeds
    createRandomSeedList(500, false);

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
            // TODO: Make a function for that
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
            stopGrowth = boundaryStoppingCondition(nextPosition)
                      || degeneratePointStoppingCondition(i,j)
                      || loopStoppingCondition(nextPosition,road.segments);
            currentPosition = nextPosition;
        }
    }
}

void StreetGraph::generateStreetGraph()
{
    // Compute the street graph
    computeMajorHyperstreamlines(true);
    drawStreetGraph(false);
}

QPixmap StreetGraph::drawStreetGraph(bool showSeeds)
{
    // Draw it in an image
    QSize imageSize(512,512);
    QPixmap pixmap(imageSize);
    pixmap.fill();

    QPainter painter(&pixmap);
    QPen penRoad(Qt::blue);
    penRoad.setWidth(2);
    QPen penNode(Qt::red);
    penNode.setWidth(3);

    NodeMapIterator itn = mNodes.begin(), itn_end = mNodes.end();
    RoadMapIterator itr = mRoads.begin(), itr_end = mRoads.end();

    // Draw the roads
    for(; itr != itr_end ; itr++)
    {
        painter.setPen(penRoad);
        for(int i=1 ; i < itr->segments.size() ; i++)
        {
            QPointF a = itr->segments[i-1];
            QPointF b = itr->segments[i];
            a.rx() *= imageSize.width()/mRegionSize.width();
            a.ry() *= imageSize.height()/mRegionSize.height();
            a.ry() = imageSize.height() - a.y();
            b.rx() *= imageSize.width()/mRegionSize.width();
            b.ry() *= imageSize.height()/mRegionSize.height();
            b.ry() = imageSize.height() - b.y();
            painter.drawLine(a, b);
        }
    }
    // Draw the nodes (seeds)
    if(showSeeds)
    {
        for(; itn != itn_end ; itn++)
        {
            painter.setPen(penNode);
            QPointF a = itn->position;
            a.rx() *= imageSize.width()/mRegionSize.width();
            a.ry() *= imageSize.height()/mRegionSize.height();
            a.ry() = imageSize.height() - a.y();
            painter.drawPoint(a);
        }
    }
    emit newStreetGraphImage(pixmap);
    return pixmap;
}

void StreetGraph::clearStoredStreetGraph()
{
    mNodes.clear();
    mRoads.clear();
    mLastNodeID = 0;
    mLastRoadID = 0;
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

bool StreetGraph::degeneratePointStoppingCondition(int i, int j)
{
    return isDegenerate(mTensorField->getTensor(i,j));
}

bool StreetGraph::loopStoppingCondition(QPointF nextPosition, QVector<QPointF>& segments)
{
    // TODO : Look for a better way to compare
    // It needs a larger span (maybe function of dSeperation)
    if(isFuzzyEqual(nextPosition.x(),segments.first().x())
       && isFuzzyEqual(nextPosition.y(),segments.first().y()))
    {
        return true;
    }
    return false;
}

bool StreetGraph::exceedingLengthStoppingCondition(QVector<QPointF>& segments)
{
    if(computePathLength(segments) > mDistSeparation)
    {
        return true;
    }
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

float computePathLength(const QVector<QPointF>& segments)
{
    float length = 0;
    for(int i=1 ; i<segments.size() ; i++)
    {
        length += QVector2D(segments[i]-segments[i-1]).length();
    }
    return length;
}
