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
    QVector<QPointF> segments;
    int nodeID1;
    int nodeID2;
    RoadType type;
};

// Structure to store an intersection (node)
struct Node {
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

    // Returns wether the point is too close from one of the existing seeds
    bool pointRespectSeedSeparationDistance(QPointF point, float separationDistance);

    // Compute the major hyperstreamlines from the stored tensor field
    void computeMajorHyperstreamlines(bool clearStorage);

    // Compute the street graph from the stored tensor field
    void computeStreetGraph(bool clearStorage);

    // Draw an image with major hyperstreamlines
    QPixmap drawStreetGraph(bool showSeeds);

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
    float mDistSeparation;

};

// Overloads writing Road to std stream
std::ostream& operator<<(std::ostream& out, const Road r);
// Overloads writing Node to std stream
std::ostream& operator<<(std::ostream& out, const Node n);
// Overloads writing QPointF to std stream
std::ostream& operator<<(std::ostream& out, const QPointF p);

// Compute the length of a road
float computePathLength(const QVector<QPointF>& segments);

#endif // STREETGRAPH_H
