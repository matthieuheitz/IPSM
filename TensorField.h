#ifndef TENSORFIELD_H
#define TENSORFIELD_H

#include <QObject>
#include <QImage>
#include <QGenericMatrix>
#include <QArrayData>
#include <QVector>
#include <QVector2D>
#include <QVector4D>
#include <QColor>
#include <QPixmap>
#include <QSize>

class TensorField : public QObject
{
    Q_OBJECT
public:
    explicit TensorField(QSize fieldsize = QSize(256,256), QObject *parent = 0);

    /** Getters and Setters **/

    // Get the Tensor at index (i,j)
    QVector4D getTensor(int i, int j);
    // Set the Tensor at index (i,j)
    void setTensor(int i, int j, QVector4D tensor);

    // Get the tensor field size
    QSize getFieldSize() {return this->mFieldSize;}
    // Set the tensor field size
    void setFieldSize(QSize fieldSize) {this->mFieldSize = fieldSize;}

    /** General Use Functions */

    // Creates a grid tensor field
    void fillGridBasisField(QVector2D direction);
    void fillGridBasisField(float theta, float l);

    // Output the tensor field to QDebug
    void outputTensorField();

    // Display the tensor field with 2 vectors per point
    QPixmap exportTensorVectorsImage(bool drawVector1 = true, bool drawVector2 = false,
                                     QColor color1 = Qt::blue, QColor color2 = Qt::red);



signals:

    void newTensorFieldImage(QPixmap);

public slots:

    void generateTensorField();

private:

    QSize mFieldSize;
    QVector<QVector<QVector4D> > mData;

};

#endif // TENSORFIELD_H
