#ifndef STREETGRAPH_H
#define STREETGRAPH_H

#include <QObject>
#include <QPointF>
#include <QSize>
#include <QMap>

#include "TensorField.h"

struct Node;

enum RoadType {
    Principal,
    Secondary
};

// Structure to store a road
struct Road {
    int ID;
    QVector<QPointF> segments;
    int nodeID1;
    int nodeID2;
    RoadType type;
    float straightLength;
    float pathLength;
};

// Structure to store an intersection (node)
struct Node {
    int ID;
    QPointF position;
    QVector<int> connectedNodeIDs;
    QVector<int> connectedRoadIDs;
};

// Convenience typedefs
typedef QMap<int,Node>::iterator NodeMapIterator;
typedef QMap<int,Road>::iterator RoadMapIterator;

class StreetGraph : public QObject
{
    Q_OBJECT
public:
    // Construct a StreetGraph object within limits passed
    explicit StreetGraph(QPointF bottomLeft, QPointF topRight, TensorField * field, float distSeparation, QObject *parent = 0);

    // Create a random seed list
    void createRandomSeedList(int numberOfSeeds, bool append);

    // Create a random seed list that respect a certain density
    void createDensityConstrainedSeedList(int numberOfSeeds, bool append);

    // Create a list of seeds spread in a grid pattern on the region
    void createGridSeedList(QSize numberOfSeeds, bool append);

    // Create a list of seeds following the method asked by the user in the UI
    void generateSeedListWithUIMethod();

    // Returns wether the point is too close from one of the existing seeds
    bool pointRespectSeedSeparationDistance(QPointF point, float separationDistance);

    // Compute the major hyperstreamlines from the stored tensor field
    void computeMajorHyperstreamlines(bool clearStorage);

    // Compute the street graph from the stored tensor field
    void computeStreetGraph(bool clearStorage);
    void computeStreetGraph2(bool clearStorage);
    void computeStreetGraph3(bool clearStorage);
    // 1 : doesn't check for segments being too long. Doesn't replant seeds
    // 2 : Checks for segments being too long. Replants seeds
    // 3 : Seeds grow in both directions

    // Grow a road until it leaves the field, is too long, or other stopping condition
    Node& growRoad(Road& road, Node& startNode, bool growInMajorDirection, bool growInOppositeDirection, bool useExceedLenStopCond);

    // Grow a road and connects it to the first road it crosses
    Node& growRoadAndConnect(Road& road, Node& startNode, bool growInMajorDirection, bool growInOppositeDirection, bool useExceedLenStopCond);

    // Draw an image with major hyperstreamlines
    QPixmap drawStreetGraph(bool showNodes, bool showSeeds);

    // Draw the road network using the painter
    void drawRoads(QPainter& painter, QSize imageSize);

    // Clear the stored street graph (Nodes, Roads)
    // Warning: Doesn't clear the seed list
    void clearStoredStreetGraph();

    // Set the tensor field to compute street graph from
    void setTensorField(TensorField * field) {mTensorField = field;}

signals:

    // Fired when a new image is drawn
    void newStreetGraphImage(QPixmap);

public slots:

    // Main function : compute and draw the street graph
    void generateStreetGraph();
    // Change method to initialize seeds
    void changeSeedInitMethod(int index) {mSeedInitMethod = index;}
    // Set the variable for drawing nodes or not
    void setDrawNodes(bool drawNodes);
    // Set the density variable
    void setSeparationDistance(double separationDistance);

private:

    // 1st condition: Reaching boundary
    bool boundaryStoppingCondition(QPointF nextPosition);
    // 2nd condition: Reaching a degenerate point
    bool degeneratePointStoppingCondition(int i, int j);
    // 3rd condition: Returning to origin
    bool loopStoppingCondition(QPointF nextPosition, const QVector<QPointF> &segments);
    // 4th condition: Exceeding user-defined max length
    bool exceedingLengthStoppingCondition(const QVector<QPointF>& segments);
    // 5th condition: Too close to other hyperstreamline
    bool exceedingDensityStoppingCondition();
    // Check if road is meeting another one. Find the closest point of the met road
    bool meetsAnotherRoad(Road &road, int &intersectedRoadID, int &closestPointID, float minDistance);
    // Check if road is meeting another one. Find the intersection of the two meeting road.
    // The intersection isn't necessarily a point of the met road, unlike in meetsAnotherRoad().
    bool meetsAnotherRoadAndFindIntersection(int roadID, QPointF nextPosition, int &intersectedRoadID,
                            int &closestPointID, QPointF &intersectionPoint);


    // Tensor field
    TensorField * mTensorField;
    // Container for nodes
    QMap<int,Node> mNodes;
    // Container for roads
    QMap<int,Road> mRoads;
    // Container for seeds
    QVector<QPointF> mSeeds;
    // Height and width of the region
    QSizeF mRegionSize;
    // Coordinates of the bottom left point
    QPointF mBottomLeft;
    // Coordinates of the top right point
    QPointF mTopRight;
    // Last IDs for Nodes and Roads
    int mLastNodeID;
    int mLastRoadID;
    // Distance for road density
    float mSeparationDistance;
    // Watermap
    QImage mWatermap;
    // Method to use for seed initialization
    int mSeedInitMethod;
    // Holds if nodes should be drawn in the street graph image
    bool mDrawNodes;

};

// Overloads writing Road to std stream
std::ostream& operator<<(std::ostream& out, const Road r);
// Overloads writing Node to std stream
std::ostream& operator<<(std::ostream& out, const Node n);
// Overloads writing QPointF to std stream
std::ostream& operator<<(std::ostream& out, const QPointF p);

// Compute the length of a road
float computePathLength(const QVector<QPointF>& segments);
// Compute the length between the 2 endpoints of a road
float computeStraightLength(const QVector<QPointF>& segments);
// Compute det(AB, AM) which determines if M is in, on the left,
// or on the right of AB
float detPointLine(QPointF A, QPointF B, QPointF M);
// Compute determinant of V1 and V2 (2x2 matrix)
float det2D(QPointF V1, QPointF V2);
// Find the intersection point between segments AB and CD.
// If there isn't, a null QPointF is returned
QPointF computeIntersectionPoint(QPointF A, QPointF B, QPointF C, QPointF D);

#endif // STREETGRAPH_H
