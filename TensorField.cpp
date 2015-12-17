#include "TensorField.h"
#include "math.h"
#include "iostream"
#include "eigen3/Eigen/Dense"

#include <QPainter>
#include <QPen>


TensorField::TensorField(QSize fieldSize, QObject *parent) :
    QObject(parent), mFieldSize(fieldSize)
{
    mData.resize(fieldSize.height());
    for(int i=0 ; i < fieldSize.height() ; i++)
    {
        mData[i].resize(fieldSize.width());
    }
    mFieldIsFilled = false;
}

QVector4D TensorField::getTensor(int i, int j)
{
    return mData[i][j];
}

void TensorField::setTensor(int i, int j, QVector4D tensor)
{
    mData[i][j] = tensor;
}

void TensorField::fillGridBasisField(float theta, float l)
{
    for(int i=0; i<mFieldSize.height() ; i++)
    {
        for(int j=0; j<mFieldSize.width() ; j++)
        {
            QVector4D tensor;
            tensor.setX(cos(2.0*theta));
            tensor.setY(sin(2.0*theta));
            tensor.setZ(sin(2.0*theta));
            tensor.setW(-cos(2.0*theta));
            tensor *= l;
            mData[i][j] = tensor;
        }
    }
    mFieldIsFilled = true;
}

void TensorField::fillRotatingField()
{
    for(int i=0; i<mFieldSize.height() ; i++)
    {
        for(int j=0; j<mFieldSize.width() ; j++)
        {
            float theta = M_PI*j/(mFieldSize.width()-1) + i*M_PI/4/(mFieldSize.height()-1);
            QVector4D tensor;
            tensor.setX(cos(2.0*theta));
            tensor.setY(sin(2.0*theta));
            tensor.setZ(sin(2.0*theta));
            tensor.setW(-cos(2.0*theta));
            mData[i][j] = tensor;
        }
    }
    mFieldIsFilled = true;
}

void TensorField::fillGridBasisField(QVector2D direction)
{
    float theta = atan2(direction.y(),direction.x());
    this->fillGridBasisField(theta, direction.length());
}

void TensorField::fillRadialBasisField(QPointF center)
{
    float x;
    float y;
    for(int i=0; i<mFieldSize.height() ; i++)
    {
        for(int j=0; j<mFieldSize.width() ; j++)
        {
            x = ((float)j/(mFieldSize.height()-1) - center.x());
            y = ((float)i/(mFieldSize.width()-1) - center.y());
            QVector4D tensor;
            tensor.setX((std::pow(y,2.0)-std::pow(x,2.0)));
            tensor.setY(-2*x*y);
            tensor.setZ(-2*x*y);
            tensor.setW(-(std::pow(y,2.0)-std::pow(x,2.0)));
            mData[i][j] = tensor;
        }
    }
    mFieldIsFilled = true;
}

void TensorField::generateTensorField()
{
    qDebug()<<"Generate Tensor Field";
//    this->fillGridBasisField(M_PI/3, 1);
    this->fillRotatingField();

    this->computeTensorsEigenDecomposition();
    this->exportEigenVectorsImage(true, true);
}

void TensorField::outputTensorField()
{
    for(int i=0; i<mFieldSize.height() ; i++)
    {
        for(int j=0; j<mFieldSize.width() ; j++)
        {
            qDebug()<<mData[i][j];
        }
        qDebug();
    }
}

QPixmap TensorField::exportEigenVectorsImage(bool drawVector1, bool drawVector2,
                                              QColor color1, QColor color2)
{
    int imageSize = 512;
    QPixmap pixmap(imageSize,imageSize);
    pixmap.fill();

    QPainter painter(&pixmap);
    QPen pen1(color1);
    QPen pen2(color2);

    float dv = imageSize/(float)mFieldSize.height();
    float du = imageSize/(float)mFieldSize.width();
    QVector2D origin(du/2.0f, dv/2.0f);

    for(int i=0; i<mFieldSize.height() ; i++)
    {
        for(int j=0; j<mFieldSize.width() ; j++)
        {
            if(drawVector1)
            {
                painter.setPen(pen1);
                QVector2D base = origin + QVector2D(j*dv, i*du);
                QVector2D eigenVector = getTensorMajorEigenVector(mData[i][j]);
                eigenVector.setX(eigenVector.x()*du/2.0f);
                eigenVector.setY(eigenVector.y()*dv/2.0f);
                QVector2D tip = base + eigenVector;
                base -= eigenVector;
                roundVector2D(base);
                roundVector2D(tip);
                // Flip the y axis because the painter system has its origin
                // on the top left corner and the y axis points down
                painter.drawLine(base.x(),imageSize - base.y(),tip.x(),imageSize - tip.y());
            }
            if(drawVector2)
            {
                painter.setPen(pen2);
                QVector2D base = origin + QVector2D(j*dv, i*du);
                QVector2D eigenVector = getTensorMinorEigenVector(mData[i][j]);
                eigenVector.setX(eigenVector.x()*du/2.0f);
                eigenVector.setY(eigenVector.y()*dv/2.0f);
                QVector2D tip = base + eigenVector;
                base -= eigenVector;
                roundVector2D(base);
                roundVector2D(tip);
                // Flip the y axis because the painter system has its origin
                // on the top left corner and the y axis points down
                painter.drawLine(base.x(),imageSize - base.y(),tip.x(),imageSize - tip.y());
            }
        }
    }

    emit newTensorFieldImage(pixmap);
    return pixmap;
}

int TensorField::computeTensorsEigenDecomposition()
{
    if(!mFieldIsFilled)
    {
        qCritical()<<"Fill the tensor field before computing the eigen vectors";
        return -1;
    }
    // Initialize the vectors and values internal containers if they aren't already
    if(mEigenVectors.size() ==0 || mEigenValues.size() == 0)
    {
        mEigenVectors.resize(mFieldSize.height());
        mEigenValues.resize(mFieldSize.height());
        for(int i=0 ; i < mFieldSize.height() ; i++)
        {
            mEigenVectors[i].resize(mFieldSize.width());
            mEigenValues[i].resize(mFieldSize.width());
        }
    }
    // Fill the internal containers
    int numberOfDegeneratePoints = 0;
    for(int i=0; i<mFieldSize.height() ; i++)
    {
        for(int j=0; j<mFieldSize.width() ; j++)
        {
            mEigenVectors[i][j] = getTensorEigenVectors(mData[i][j]);
            mEigenValues[i][j] = getTensorEigenValues(mData[i][j]);
            if(isDegenerate(mEigenVectors[i][j]))
            {
                numberOfDegeneratePoints++;
            }
        }
    }
    mEigenIsComputed = true;
    return numberOfDegeneratePoints;
}

QVector4D TensorField::getEigenVectors(int i, int j)
{
    if(!mEigenIsComputed)
    {
        qCritical()<<"Unable to get the eigen vectors."
                   <<"First compute tensors Eigen decomposition";
        return QVector4D();
    }
    else
    {
        return mEigenVectors[i][j];
    }
}

QVector2D TensorField::getEigenValues(int i, int j)
{
    if(!mEigenIsComputed)
    {
        qCritical()<<"Unable to get the eigen values."
                   <<"First compute tensors Eigen decomposition";
        return QVector2D();
    }
    else
    {
        return mEigenValues[i][j];
    }
}

QVector2D TensorField::getMajorEigenVector(int i, int j)
{
    return getFirstVector(this->getEigenVectors(i,j));
}

QVector2D TensorField::getMinorEigenVector(int i, int j)
{
    return getSecondVector(this->getEigenVectors(i,j));
}




/** ******************** */
/** Non-member Functions */
/** ******************** */



void roundVector2D(QVector2D& vec)
{
    vec.setX(round(vec.x()));
    vec.setY(round(vec.y()));
}

QVector2D getFirstVector(QVector4D matrix)
{
    return QVector2D(matrix.x(),matrix.y());
}

QVector2D getSecondVector(QVector4D matrix)
{
    return QVector2D(matrix.z(),matrix.w());
}

QVector4D getTensorEigenVectors(QVector4D tensor)
{
    if(!isSymetricalAndTraceless(tensor))
    {
        qCritical()<<"The tensor must be traceless and symetrical";
        return QVector4D();
    }
    Eigen::Matrix2f m(2,2);
    m(0,0) = tensor.x();
    m(1,0) = tensor.y();
    m(0,1) = tensor.z();
    m(1,1) = tensor.w();

    Eigen::EigenSolver<Eigen::Matrix2f> es(m);
    QVector2D vec1(es.eigenvectors().col(0).real()[0],es.eigenvectors().col(0).real()[1]);
    QVector2D vec2(es.eigenvectors().col(1).real()[0],es.eigenvectors().col(1).real()[1]);
    QVector2D val = getTensorEigenValues(tensor);
    if(isFuzzyEqual(val.x(), fmax(val.x(),val.y())))
    {
        return QVector4D(vec1.x(), vec1.y(),vec2.x(), vec2.y());
    }
    else
    {
        return QVector4D(vec2.x(), vec2.y(),vec1.x(), vec1.y());
    }
}

QVector2D getTensorEigenValues(QVector4D tensor)
{
    Eigen::Matrix2f m(2,2);
    m(0,0) = tensor.x();
    m(1,0) = tensor.y();
    m(0,1) = tensor.z();
    m(1,1) = tensor.w();
    Eigen::EigenSolver<Eigen::Matrix2f> es(m);
    return QVector2D(es.eigenvalues()[0].real(),es.eigenvalues()[1].real());
}

QVector2D getTensorMajorEigenVector(QVector4D tensor)
{
    return getFirstVector(getTensorEigenVectors(tensor));
}

QVector2D getTensorMinorEigenVector(QVector4D tensor)
{
    return getSecondVector(getTensorEigenVectors(tensor));
}

bool isSymetricalAndTraceless(QVector4D tensor)
{
    return isFuzzyEqual(tensor.y(), tensor.z())
            && isFuzzyNull(tensor.x() + tensor.w());
}

bool isDegenerate(QVector4D tensor)
{
    return isFuzzyNull(tensor.x())
            && isFuzzyNull(tensor.y())
            && isFuzzyNull(tensor.z())
            && isFuzzyNull(tensor.w());
}

bool isFuzzyNull(float a)
{
    return (fabs(a) < FLOAT_COMPARISON_EPSILON);
}

bool isFuzzyEqual(float a, float b)
{
    return (fabs(a - b) / FLOAT_COMPARISON_EPSILON <= fmin(fabs(a), fabs(b)));
}
