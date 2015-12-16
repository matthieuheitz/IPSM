#ifndef STREETGRAPH_H
#define STREETGRAPH_H

#include <QObject>

#include "TensorField.h"

class StreetGraph : public QObject
{
    Q_OBJECT
public:
    // Construct a graph within limits passed
    //
    explicit StreetGraph(QPointF bottomLeft, QPointF topRight, TensorField * field, QObject *parent = 0);

signals:

public slots:

private:

    // Tensor field
    TensorField * mTensorField;
    // Height and width of the region
    QSizeF mRegionSize;
    // Coordinates of the bottom left point
    QPointF mBottomLeft;
    // Coordinates of the top right point
    QPointF mTopRight;
};

#endif // STREETGRAPH_H
