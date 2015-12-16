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
    explicit StreetGraph(QPointF bottomLeft, QPointF topRight, TensorField * field, QObject *parent = 0);

    // Create a random seed list
    void createRandomSeedList(int numberOfSeeds);

    // Compute the major hyperstreamlines from the tensor field
    void computeMajorHyperstreamlines();

    // Draw an image with major hyperstreamlines
    QPixmap drawStreetGraph(bool showSeeds);

    // Clear the stored street graph (Nodes, Roads, Seeds)
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
    bool degeneratePointStoppingCondition();
    // 3rd condition: Returning to origin
    bool loopStoppingCondition();
    // 4th condition: Exceeding user-defined max length
    bool exceedingLengthStoppingCondition();
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

};

// Overloads writing Road to std stream
std::ostream& operator<<(std::ostream& out, const Road r);
// Overloads writing Node to std stream
std::ostream& operator<<(std::ostream& out, const Node n);
// Overloads writing QPointF to std stream
std::ostream& operator<<(std::ostream& out, const QPointF p);

#endif // STREETGRAPH_H
