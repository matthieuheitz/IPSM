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

    // Generate a grid basis field from a 2D vector
    // Don't normalize the vector as this function
    // integrates the vector's norm in the tensor
    void fillGridBasisField(QVector2D direction);
    // Generate a grid basis function from an angle and
    // a vector's length (norm)
    void fillGridBasisField(float theta, float l);

    // Output the tensor field to QDebug
    void outputTensorField();

    // Display the tensor field with 2 vectors per point
    QPixmap exportTensorVectorsImage(bool drawVector1 = true, bool drawVector2 = false,
                                     QColor color1 = Qt::blue, QColor color2 = Qt::red);



signals:

    // Fired when a new tensor field image is created
    void newTensorFieldImage(QPixmap);

public slots:

    // Generates a tensor field with default parameters
    void generateTensorField();

private:

    // Tensor field
    // A tensor is stored with a QVector4D.
    // The coordinates are as follows:
    // | x  w |
    // | y  z |
    // A traceless, real, symmetrical tensor is of the form:
    // | a  b |
    // | b -a |
    QVector<QVector<QVector4D> > mData;
    // Field size
    QSize mFieldSize;
};


/** Non-member Functions */


// Round a 2D vector
void roundVector2D(QVector2D vec);
#endif // TENSORFIELD_H
