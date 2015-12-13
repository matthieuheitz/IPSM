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

// Epsilon for float comparison
#define FLOAT_COMPARISON_EPSILON 1e-5

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

    // Returns whether the field has been filled with non-zero values
    bool isFieldFilled();

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

    // Returns the major and minor eigenvectors of the tensor at index (i,j).
    // They are normalized, then multiplied by their respective eigenvalue.
    // Warning : This only works if the tensor is traceless, real and symmetrical
    QVector4D getEigenVectors(int i, int j);
    // Returns the major and minor eigenvalues of the tensor at index (i,j).
    // Warning : This only works if the tensor is traceless, real and symmetrical
    QVector2D getEigenValues(int i, int j);
    // Returns the major eigenvector of the tensor at index (i,j).
    // It is normalized, then multiplied by its eigenvalue.
    // Warning : This only works if the tensor is traceless, real and symmetrical
    QVector2D getMajorEigenVector(int i, int j);
    // Returns the minor eigenvector of the tensor at index (i,j).
    // It is normalized, then multiplied by its eigenvalue.
    // Warning : This only works if the tensor is traceless, real and symmetrical
    QVector2D getMinorEigenVector(int i, int j);


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
    // Eigen vectors of each tensor matrix
    QVector<QVector<QVector4D> > mEigenVectors;
    // Eigen values of each tensor matrix
    QVector<QVector<QVector2D> > mEigenValues;
    // Holds wether the field has been initialized with non-zero values
    bool mFieldIsFilled;
    // Holds wether the eigen vectors and values has been computed
    bool mEigenIsComputed;
    // Field size
    QSize mFieldSize;
};


/** Non-member Functions */


// Round a 2D vector
void roundVector2D(QVector2D vec);

// Get the first vector of the 2x2 matrix
QVector2D getFirstVector(QVector4D matrix);
// Get the second vector of the 2x2 matrix
QVector2D getSecondVector(QVector4D matrix);

// Returns the normalized major and minor eigenvectors of the passed tensor
// Warning : This only works if the tensor is traceless, real and symmetrical
QVector4D getTensorEigenVectors(QVector4D tensor);
// Returns the major and minor eigenvalues of the passed tensor.
// Warning : This only works if the tensor is traceless, real and symmetrical
QVector2D getTensorEigenValues(QVector4D tensor);
// Returns the normalized major eigenvector of the passed tensor.
// Warning : This only works if the tensor is traceless, real and symmetrical
QVector2D getMajorEigenVector(QVector4D tensor);
// Returns the normalized minor eigenvector of the passed tensor.
// Warning : This only works if the tensor is traceless, real and symmetrical
QVector2D getMinorEigenVector(QVector4D tensor);

// Returns whether the tensor is real, symmetrical and traceless, or not.
bool isSymetricalAndTraceless(QVector4D tensor);
// Returns whether the tensor is degenerate or not
bool isDegenerate(QVector4D tensor);



#endif // TENSORFIELD_H
