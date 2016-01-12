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
    void setFieldSize(QSize fieldSize);

    // Returns whether the field has been filled with non-zero values
    bool isFieldFilled() {return mFieldIsFilled;}

    // Returns whether the field has been filled with non-zero values
    bool isWatermapLoaded() {return mWaterMapIsLoaded;}

    // Returns whether the field has been filled with non-zero values
    QString getWatermapFilename() {return mWatermapFilename;}

    /** General Use Functions */

    // Changes the stored tensor field and put tensor to null
    // in areas where there is water
    void applyWaterMap(QString filename);
    // Generate a grid basis field from a 2D vector
    // Don't normalize the vector as this function
    // integrates the vector's norm in the tensor
    void fillGridBasisField(QVector2D direction);
    // Generate a grid basis function from an angle and an amplitude.
    // There is no direction, so theta and theta + pi give the same result
    void fillGridBasisField(float theta, float l, QPointF center, float decay);
    // Generate a heightmap basis function from a filename
    void fillHeightBasisField(QString filename, QPointF center, float decay);
    // Same thing, using a sobel filter to approximate the gradient
    void fillHeightBasisFieldSobel(QString filename, QPointF center, float decay);
    // Generate a radial basis field
    // The center coordinates must be in [0,1], considering that
    // The bottom left corner of the image is (0,0) and bottom right
    // corner is (1,0)
    void fillRadialBasisField(QPointF center, float decay);
    // Test function to check the different angles
    void fillRotatingField(QPointF center, float decay);

    // Output the tensor field to QDebug
    void outputTensorField();

    // Display the tensor field with 2 vectors per point
    QPixmap exportEigenVectorsImage(bool drawVector1 = true, bool drawVector2 = false,
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

    // Get a filename for watermap, store it for future display
    // and call the function to apply the watermap
    void actionAddWatermap();
    // Generates a grid tensor field with default parameters
    void generateGridTensorField();
    // Generates a tensor field from a heightmap
    void generateHeightmapTensorField();
    // Generates a multi-rotation tensor field (linear variation)
    void generateMultiRotationTensorField();
    // Generates a radial tensor field with default parameters
    void generateRadialTensorField();
    // Compute the eigen vectors and values of each tensor in the field,
    // and store them internally.
    // Return the number of degenerate points (null eigenvectors)
    int computeTensorsEigenDecomposition();

private:

    // Tensor field
    // A tensor is stored with a QVector4D.
    // The coordinates are as follows:
    // | x  z |
    // | y  w |
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
    // Holds wether a watermap has been loaded
    bool mWaterMapIsLoaded;
    // Filename of the watermap
    QString mWatermapFilename;
    // Field size
    QSize mFieldSize;
};


/** Non-member Functions */


// Round a 2D vector
void roundVector2D(QVector2D &vec);
// Returns whether floating point value a is considered equal to 0
bool isFuzzyNull(float a);
// Returns wether floating point value a and b are considered equal
bool isFuzzyEqual(float a, float b);

// Get the first vector of the 2x2 matrix
QVector2D getFirstVector(QVector4D matrix);
// Get the second vector of the 2x2 matrix
QVector2D getSecondVector(QVector4D matrix);

// Returns the normalized major and minor eigenvectors of the passed tensor
// The first column vector is the one associated with the maximum eigenvalue
// Warning : This only works if the tensor is traceless, real and symmetrical
QVector4D getTensorEigenVectors(QVector4D tensor);
// Returns the major and minor eigenvalues of the passed tensor.
// The maximum eigenvalue is first, and the minimum is second
// Warning : This only works if the tensor is traceless, real and symmetrical
QVector2D getTensorEigenValues(QVector4D tensor);
// Returns the normalized major eigenvector of the passed tensor.
// Warning : This only works if the tensor is traceless, real and symmetrical
QVector2D getTensorMajorEigenVector(QVector4D tensor);
// Returns the normalized minor eigenvector of the passed tensor.
// Warning : This only works if the tensor is traceless, real and symmetrical
QVector2D getTensorMinorEigenVector(QVector4D tensor);
// Returns the image created by applying Sobel filter on x
QImage applySobelX(QImage map);
// Returns the image created by applying Sobel filter on y
QImage applySobelY(QImage map);
// Returns the sum of elements of a matrix 3x3
int sumMat3D(QMatrix3x3 matrix, QMatrix3x3 kernel);

// Returns whether the tensor is real, symmetrical and traceless, or not
bool isSymetricalAndTraceless(QVector4D tensor);
// Returns whether the tensor is degenerate or not
bool isDegenerate(QVector4D tensor);



#endif // TENSORFIELD_H
