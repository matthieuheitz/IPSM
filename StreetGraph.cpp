#include <iostream>
#include <cmath>
#include <QPainter>
#include <QDateTime>
#include <QCoreApplication>
#include <QProgressDialog>

#include "StreetGraph.h"

StreetGraph::StreetGraph(QPointF bottomLeft, QPointF topRight, TensorField *field, float distSeparation, QObject *parent) :
    QObject(parent), mTensorField(field), mBottomLeft(bottomLeft), mTopRight(topRight), mSeparationDistance(distSeparation)
{
    mRegionSize.rwidth() = (topRight-bottomLeft).x();
    mRegionSize.rheight() = (topRight-bottomLeft).y();
    mLastNodeID = 0;
    mLastRoadID = 0;
    mSeedInitMethod = 0;
    mDrawNodes = false;
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

void StreetGraph::createDensityConstrainedSeedList(int numberOfSeeds, bool append)
{
    if(!append)
    {
        mSeeds.clear();
    }
    qsrand(QDateTime::currentDateTime().toTime_t());
    for(int i=0 ; i < numberOfSeeds ; i++)
    {
        int counter = 0;
        bool pointIsValid = false;
        QPointF seed;
        while(!pointIsValid && counter < 10)
        {
            float randX = qrand()/(float)RAND_MAX;
            float randY = qrand()/(float)RAND_MAX;
            seed = QPointF(mBottomLeft.x() + randX*mRegionSize.width(),
                         mBottomLeft.y() + randY*mRegionSize.height());
            pointIsValid = pointRespectSeedSeparationDistance(seed,mSeparationDistance);
            counter++;
        }
        if(pointIsValid)
        {
            mSeeds.push_back(seed);
        }
    }
}

int StreetGraph::createGridSeedList(double separationDistance, bool append)
{
    if(!append)
    {
        mSeeds.clear();
    }
    int Nv = (int)(mRegionSize.height()/separationDistance);
    int Nu = (int)(mRegionSize.width()/separationDistance);
    QPointF origin(separationDistance/2.0f,separationDistance/2.0f);
    for(int i=0 ; i<Nv ; i++)
    {
        for(int j=0 ; j<Nu ; j++)
        {
            QPointF position = origin + QPointF(j*separationDistance, i*separationDistance);
            mSeeds.push_back(position);
        }
    }
    return Nv*Nu;
}

void StreetGraph::generateSeedListWithUIMethod()
{
    switch(mSeedInitMethod)
    {
    case 0:
        createGridSeedList(mSeparationDistance,false);
        break;
    case 1:
        createRandomSeedList(50, false);
        break;
    case 2:
        createDensityConstrainedSeedList(100, false);
        break;
    default:
        qWarning()<<"Unrecognized seed initialization method";
        break;
    }
}

bool StreetGraph::pointRespectSeedSeparationDistance(QPointF point, float separationDistance)
{
    for(int i=0 ; i < mSeeds.size() ; i++)
    {
        if(QVector2D(mSeeds[i]-point).length() < separationDistance)
        {
            return false;
        }
    }
    return true;
}

void StreetGraph::computeMajorHyperstreamlines(bool clearStorage)
{
    if(clearStorage)
    {
        clearStoredStreetGraph();
    }
    if(mTensorField == NULL || !(mTensorField->isFieldFilled()))
    {
        qCritical()<<"computeMajorHyperstreamlines(): Tensor field is empty";
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

void StreetGraph::computeStreetGraph(bool clearStorage)
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
    generateSeedListWithUIMethod();

    bool majorGrowth = true;
    for(int k=0 ; k<mSeeds.size() ; k++)
    {
        // Create a node
        // Grow a road starting from this node using the tensor eigen vector
        // until one of the condition is reached
        Node& node1 = mNodes[++mLastNodeID];
        node1.ID = mLastNodeID;
        node1.position = mSeeds[k];

        Road& road = mRoads[++mLastRoadID];
        road.ID = mLastRoadID;
        node1.connectedRoadIDs.push_back(mLastRoadID);
        road.type = Principal;
        road.nodeID1 = mLastNodeID;

        growRoad(road, node1, majorGrowth, false, false);
    }
}

void StreetGraph::computeStreetGraph2(bool clearStorage)
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
    generateSeedListWithUIMethod();

    bool majorGrowth = true;
    for(int k=0 ; k<mSeeds.size() ; k++)
    {
        // Create a node
        // Grow a road starting from this node using the tensor eigen vector
        // until one of the condition is reached
        Node& node1 = mNodes[++mLastNodeID];
        node1.ID = mLastNodeID;
        node1.position = mSeeds[k];

        Road& road = mRoads[++mLastRoadID];
        road.ID = mLastRoadID;
        node1.connectedRoadIDs.push_back(mLastRoadID);
        road.type = Principal;
        road.nodeID1 = mLastNodeID;

        growRoad(road, node1, majorGrowth, false, true);
    }
}

void StreetGraph::computeStreetGraph3(bool clearStorage)
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
    generateSeedListWithUIMethod();

    QProgressDialog progress("Creating Street Graph...",NULL, 0, mSeeds.size()-1);
    progress.setMinimumDuration(0);

    bool majorGrowth = true;
    for(int k=0 ; k<mSeeds.size() ; k++)
    {
        progress.setValue(k);
        progress.setRange(0, mSeeds.size()-1);
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

        // Create a node
        Node& node1 = mNodes[++mLastNodeID];
        node1.ID = mLastNodeID;
        node1.position = mSeeds[k];

        Road& road = mRoads[++mLastRoadID];
        road.ID = mLastRoadID;
        node1.connectedRoadIDs.push_back(mLastRoadID);
        road.type = Principal;
        road.nodeID1 = mLastNodeID;

        Road& road2 = mRoads[++mLastRoadID];
        road2.ID = mLastRoadID;
        node1.connectedRoadIDs.push_back(mLastRoadID);
        road2.type = Principal;
        road2.nodeID1 = mLastNodeID;

        bool useExceedLength = true;
        growRoadAndConnect(road, node1, majorGrowth, false, useExceedLength);
        growRoadAndConnect(road2, node1, majorGrowth, true, useExceedLength);

        majorGrowth = !majorGrowth;

        // Draw each time a road is added
        drawStreetGraph(mDrawNodes, false);
        QCoreApplication::processEvents();
    }
}

Node& StreetGraph::growRoad(Road& road, Node& startNode, bool growInMajorDirection,
                            bool growInOppositeDirection, bool useExceedLenStopCond)
{
    // Grow a road starting from this node using the tensor eigen vector
    // until one of the condition is reached
    float step = mRegionSize.height()/100.0f; // Should be function of curvature
    QSize fieldSize = mTensorField->getFieldSize();

    // The road contains also the position of its extreme nodes
    // Start from the node position
    QPointF currentPosition = startNode.position;
    // Holds wether road stopped because it was too long or not
    bool tooLong = false;
    bool stopGrowth = false;
    int preventInfiniteLoop = 0;
    while(!stopGrowth && preventInfiniteLoop < 1000)
    {
        QVector2D currentDirection;
        if(preventInfiniteLoop != 0)
        {
            currentDirection = QVector2D(currentPosition-road.segments.last());
        }
        road.segments.push_back(currentPosition);
        int i = round((currentPosition.y()-mBottomLeft.y())/mRegionSize.height()*
                    (fieldSize.height()-1));
        int j = round((currentPosition.x()-mBottomLeft.x())/mRegionSize.width()*
                    (fieldSize.width()-1));
        QVector2D majorDirection;
        if(growInMajorDirection)
        {
            majorDirection = mTensorField->getMajorEigenVector(i,j);
        }
        else
        {
            majorDirection = mTensorField->getMinorEigenVector(i,j);
        }
        // First condition is to not grow backwards
        // Second condition is applicable only at the beginning.
        // It allows to grow the road in the 2 opposite directions
        if(QVector2D::dotProduct(majorDirection,currentDirection) < 0
                || (preventInfiniteLoop == 0 && growInOppositeDirection))
        {
            majorDirection *= -1;
        }
        QPointF nextPosition = currentPosition + (step*majorDirection).toPointF();
        if(useExceedLenStopCond)
        {
            tooLong = exceedingLengthStoppingCondition(road.segments);
        }
        stopGrowth = boundaryStoppingCondition(nextPosition)
                  || degeneratePointStoppingCondition(i,j)
                  || loopStoppingCondition(nextPosition,road.segments)
                  || tooLong;
        currentPosition = nextPosition;
        preventInfiniteLoop++;
    }

    // Fill road lengths
    road.pathLength = computePathLength(road.segments);
    road.straightLength = computeStraightLength(road.segments);

    // Connect Nodes and Roads
    Node& node2 = mNodes[++mLastNodeID];
    node2.ID = mLastNodeID;
    node2.position = road.segments.last();
    node2.connectedNodeIDs.push_back(startNode.ID);
    node2.connectedRoadIDs.push_back(road.ID);
    startNode.connectedNodeIDs.push_back(node2.ID);
    road.nodeID2 = node2.ID;

    if(tooLong)
    {
        // Replant a seed only if it's not too close from another seed
        if(pointRespectSeedSeparationDistance(road.segments.last(),mSeparationDistance/4.0f))
        {
            mSeeds.push_back(node2.position);
        }
    }
    return node2;
}

Node& StreetGraph::growRoadAndConnect(Road& road, Node& startNode, bool growInMajorDirection,
                            bool growInOppositeDirection, bool useExceedLenStopCond)
{
    // Grow a road starting from this node using the tensor eigen vector
    // until one of the condition is reached
    float step = mRegionSize.height()/100.0f; // Should be function of curvature
    QSize fieldSize = mTensorField->getFieldSize();

    // The road contains also the position of its extreme nodes
    // Start from the node position
    QPointF currentPosition = startNode.position;
    // Road meeting
    bool meetOtherRoad = false;
    int metRoadID, closestPointID;
    QPointF intersectionPoint;
    // Holds wether road stopped because it was too long or not
    bool tooLong = false;
    bool stopGrowth = false;
    int preventInfiniteLoop = 0;
    while(!stopGrowth && preventInfiniteLoop < 1000)
    {
        QVector2D currentDirection;
        if(preventInfiniteLoop != 0)
        {
            currentDirection = QVector2D(currentPosition-road.segments.last());
        }
        road.segments.push_back(currentPosition);
        int i = round((currentPosition.y()-mBottomLeft.y())/mRegionSize.height()*
                    (fieldSize.height()-1));
        int j = round((currentPosition.x()-mBottomLeft.x())/mRegionSize.width()*
                    (fieldSize.width()-1));
        QVector2D majorDirection;
        if(growInMajorDirection)
        {
            majorDirection = mTensorField->getMajorEigenVector(i,j);
        }
        else
        {
            majorDirection = mTensorField->getMinorEigenVector(i,j);
        }
        // First condition is to not grow backwards
        // Second condition is applicable only at the beginning.
        // It allows to grow the road in the 2 opposite directions
        if(QVector2D::dotProduct(majorDirection,currentDirection) < 0
                || (preventInfiniteLoop == 0 && growInOppositeDirection))
        {
            majorDirection *= -1;
        }
        QPointF nextPosition = currentPosition + (step*majorDirection).toPointF();
        if(useExceedLenStopCond)
        {
            tooLong = exceedingLengthStoppingCondition(road.segments);
        }
        meetOtherRoad = meetsAnotherRoadAndFindIntersection(road.ID, nextPosition, metRoadID,
                                                            closestPointID, intersectionPoint);
        stopGrowth = boundaryStoppingCondition(nextPosition)
                  || degeneratePointStoppingCondition(i,j)
                  || loopStoppingCondition(nextPosition,road.segments)
                  || tooLong
                  || meetOtherRoad;
        currentPosition = nextPosition;
        preventInfiniteLoop++;
    }

    // Connect Nodes and Roads
    if(meetOtherRoad)
    {
        Road &metRoad = mRoads[metRoadID];
        int secondNodeID = -1;
        if(closestPointID == 0)
        {
            road.nodeID2 = metRoad.nodeID1;
            secondNodeID = metRoad.nodeID1;
            startNode.connectedNodeIDs.push_back(metRoad.nodeID1);
        }
        else if(closestPointID == metRoad.segments.size()-1)
        {
            road.nodeID2 = metRoad.nodeID2;
            secondNodeID = metRoad.nodeID2;
            startNode.connectedNodeIDs.push_back(metRoad.nodeID2);
        }
        else
        {
            Node& node2 = mNodes[++mLastNodeID];
            node2.ID = mLastNodeID;
//            node2.position = metRoad.segments[closestPointID];
            node2.position = intersectionPoint;
            node2.connectedNodeIDs.push_back(startNode.ID);
            node2.connectedRoadIDs.push_back(road.ID);
            road.segments.push_back(node2.position);
            road.nodeID2 = startNode.ID;
            startNode.connectedNodeIDs.push_back(node2.ID);
            secondNodeID = node2.ID;
            // TODO: Separate the crossed road in 2, and reconnect everything
        }

        // Fill road lengths
        road.pathLength = computePathLength(road.segments);
        road.straightLength = computeStraightLength(road.segments);

        return mNodes[secondNodeID];
    }
    else
    {
        // Create a new node on last point of the road
        Node& node2 = mNodes[++mLastNodeID];
        node2.ID = mLastNodeID;
        node2.position = road.segments.last();
        node2.connectedNodeIDs.push_back(startNode.ID);
        node2.connectedRoadIDs.push_back(road.ID);
        startNode.connectedNodeIDs.push_back(node2.ID);
        road.nodeID2 = node2.ID;

        if(tooLong)
        {
                mSeeds.push_back(node2.position);
        }
        // Fill road lengths
        road.pathLength = computePathLength(road.segments);
        road.straightLength = computeStraightLength(road.segments);

        return node2;
    }
}


void StreetGraph::generateStreetGraph()
{
    // Compute the street graph
    computeStreetGraph3(true);
//    computeMajorHyperstreamlines(true);

    drawStreetGraph(mDrawNodes, false);
}

QPixmap StreetGraph::drawStreetGraph(bool showNodes, bool showSeeds)
{
    // Draw it in an image
    QSize imageSize(512,512);
    QImage pixmap(imageSize, QImage::Format_ARGB32);
    pixmap.fill(QColor::fromRgb(230,230,230));

    if(mTensorField->isWatermapLoaded())
    {
        QString filename = mTensorField->getWatermapFilename();
        mWatermap = QImage(filename);
        if(mWatermap.isNull())
        {
            qCritical()<<"applyWaterMap(): File "<<filename<<" not found";
            return QPixmap();
        }
        for(int i=0; i<mWatermap.height() ; i++)
        {
            for(int j=0; j<mWatermap.width() ; j++)
            {
                if(qBlue(mWatermap.pixel(j,i)) > 0)
                {
                    pixmap.setPixel(j,i, mWatermap.pixel(j,i));
                }
            }
        }
    }

    if(!(mTensorField->isFieldFilled()))
    {
        qCritical()<<"drawStreetGraph(): Tensor field is empty";
        return QPixmap::fromImage(pixmap);
    }

    QPainter painter(&pixmap);

    QPen penRoad(Qt::yellow);
    penRoad.setWidth(2);
    QPen penRoadBlack(Qt::black);
    penRoadBlack.setWidth(4);
    QPen penNode(Qt::red);
    penNode.setWidth(3);
    QPen penSeed(Qt::darkGreen);
    penSeed.setWidth(4);

    // Draw two times in different colors to create
    // a road effect
    painter.setPen(penRoadBlack);
    drawRoads(painter, imageSize);
    painter.setPen(penRoad);
    drawRoads(painter, imageSize);

    NodeMapIterator itn = mNodes.begin(), itn_end = mNodes.end();

    // Draw the nodes
    if(showNodes)
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
    // Draw the seeds
    if(showSeeds)
    {
        for(int i=0 ; i <mSeeds.size() ; i++)
        {
            painter.setPen(penSeed);
            QPointF a = mSeeds[i];
            a.rx() *= imageSize.width()/mRegionSize.width();
            a.ry() *= imageSize.height()/mRegionSize.height();
            a.ry() = imageSize.height() - a.y();
            painter.drawPoint(a);
        }
    }
    emit newStreetGraphImage(QPixmap::fromImage(pixmap));
    return QPixmap::fromImage(pixmap);
}

bool StreetGraph::meetsAnotherRoad(Road &road, int &intersectedRoadID, int &closestPointID, float minDistance)
{
    QPointF roadEnd = road.segments.last();
    for(int i=0 ; i<mRoads.size() ; i++)
    {
        const QVector<QPointF>& currentSegments = mRoads[i].segments;
        if((&(mRoads[i]) != &road) && currentSegments.size() !=0)
        {
            //1st check: If the point is farther than mDistSeparation
            // from both ends, it can't intersect
            if(QVector2D(roadEnd - currentSegments.first()).length() > mSeparationDistance
               && QVector2D(roadEnd - currentSegments.last()).length() > mSeparationDistance)
            {
                intersectedRoadID = -1;
                closestPointID = -1;
                return false;
            }
            else
            {
                // 2nd check: If the last point is as close as the minimal distance
                // from the road, consider that it meets the road.
                // Returns the road ID
                for(int j=0 ; j<currentSegments.size() ; j++)
                {
                    // If taking the first closest one doesn't produce good results,
                    // finish the loop and take the minimum
                    if(QVector2D(roadEnd - currentSegments[j]).length() < minDistance)
                    {
                        intersectedRoadID = i;
                        closestPointID = j;
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool StreetGraph::meetsAnotherRoadAndFindIntersection(int roadID, QPointF nextPosition, int &intersectedRoadID,
                                                      int &closestPointID, QPointF &intersectionPoint)
{
    QPointF roadEnd = mRoads[roadID].segments.last();
    for(int i=0 ; i<mRoads.size() ; i++)
    {
        // To make it faster, first check something like :
//        if(QVector2D(roadEnd - currentSegments.first()).length() > mDistSeparation
//           && QVector2D(roadEnd - currentSegments.last()).length() > mDistSeparation)
        const QVector<QPointF>& currentSegments = mRoads[i].segments;
        bool isConnectedRoad = false;
        QVector<int>& connectedRoadIDs = mNodes[mRoads[roadID].nodeID1].connectedRoadIDs;
        for(int k=0 ; k<connectedRoadIDs.size() ; k++)
        {
            isConnectedRoad = isConnectedRoad || (i == connectedRoadIDs[k]);
        }
        // If road isn't the one passed, or one connected to it
        if(i != roadID && !isConnectedRoad && currentSegments.size() != 0 )
        {
            for(int j=1 ; j<currentSegments.size() ; j++)
            {
                // Find on which side of the current segments, the 2 points are
                float sideOfLast = detPointLine(currentSegments[j-1], currentSegments[j], roadEnd);
                float sideOfNext = detPointLine(currentSegments[j-1], currentSegments[j], nextPosition);
                if(isFuzzyNull(sideOfNext))
                {
                    intersectionPoint = nextPosition;
                }
                // If road end point and next point are on different sides of the road
                if(sideOfLast*sideOfNext < 0.0f)
                {
                    // If taking the first closest one doesn't produce good results,
                    // finish the loop and take the minimum
                    QPointF intersection = computeIntersectionPoint(currentSegments[j-1], currentSegments[j], roadEnd, nextPosition);
                    if(!intersection.isNull())
                    {
                        intersectedRoadID = i;
                        closestPointID = j;
                        intersectionPoint = intersection;
                        return true;
                    }
                }
            }
        }
    }
    intersectedRoadID = -1;
    closestPointID = -1;
    return false;
}

void StreetGraph::drawRoads(QPainter& painter, QSize imageSize)
{
    RoadMapIterator itr = mRoads.begin(), itr_end = mRoads.end();

    // Draw the roads
    for(; itr != itr_end ; itr++)
    {
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
            painter.drawLine(a,b);
        }
    }
}

void StreetGraph::clearStoredStreetGraph()
{
    mNodes.clear();
    mRoads.clear();
    mLastNodeID = 0;
    mLastRoadID = 0;
}

void StreetGraph::setDrawNodes(bool drawNodes)
{
    if(drawNodes != mDrawNodes)
    {
        mDrawNodes = drawNodes;
        drawStreetGraph(mDrawNodes,false);
    }
}

void StreetGraph::setSeparationDistance(double separationDistance)
{
    mSeparationDistance = separationDistance;
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
    if(isDegenerate(mTensorField->getTensor(i,j)))
    {
        return true;
    }
    return false;
}

bool StreetGraph::loopStoppingCondition(QPointF nextPosition, const QVector<QPointF>& segments)
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

bool StreetGraph::exceedingLengthStoppingCondition(const QVector<QPointF>& segments)
{
    if(computePathLength(segments) > mSeparationDistance)
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

float computeStraightLength(const QVector<QPointF>& segments)
{
    return QVector2D(segments[0]-segments[segments.size()-1]).length();
}

float det2D(QPointF V1, QPointF V2)
{
    return V1.x()*V2.y() - V1.y()*V2.x();
}

float detPointLine(QPointF A, QPointF B, QPointF M)
{
    return det2D(B-A,M-A);
}

QPointF computeIntersectionPoint(QPointF A, QPointF B, QPointF C, QPointF D)
{
    QPointF out;
    float denominator = det2D(B-A,D-C);
    if(isFuzzyNull(denominator))
    {
        return QPointF();
    }
    else
    {
        out.setX(det2D(QPointF(det2D(A,B),det2D(C,D)),QPointF(A.x()-B.x(),C.x()-D.x())));
        out.setY(det2D(QPointF(det2D(A,B),det2D(C,D)),QPointF(A.y()-B.y(),C.y()-D.y())));
        out /= denominator;
        if(QPointF::dotProduct(out-A,B-A) > 0 && QPointF::dotProduct(out-B,A-B) > 0)
        {
            return out;
        }
        else
        {
            return QPointF();
        }
    }
}
