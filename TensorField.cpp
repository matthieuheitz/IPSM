#include "TensorField.h"
#include "math.h"
#include "iostream"

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
    float k = 2.0;
    for(int i=0; i<mFieldSize.height() ; i++)
    {
        for(int j=0; j<mFieldSize.width() ; j++)
        {
            QVector4D tensor;
            tensor.setX(cos(k*theta));
            tensor.setY(sin(k*theta));
            tensor.setZ(sin(k*theta));
            tensor.setW(-cos(k*theta));
            tensor*=l;
            mData[i][j] = tensor;
        }
    }
    mFieldIsFilled = true;
}

void TensorField::fillGridBasisField(QVector2D direction)
{
    float theta = atan(direction.y()/direction.x());
    this->fillGridBasisField(theta, direction.length());
}

void TensorField::generateTensorField()
{
    qDebug()<<"Generate Tensor Field";
    QVector2D vec(sqrt(3)/2.0f,1.0f/2.0f);
//    QVector2D vec(1,0);
    this->fillGridBasisField(vec);
    this->exportTensorVectorsImage(true, true);
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

QPixmap TensorField::exportTensorVectorsImage(bool drawVector1, bool drawVector2,
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
                QVector2D base = origin + QVector2D(i*dv, j*du);
                QVector4D tensor = mData[i][j].normalized();
                // TODO : Don't use the 2 columns of the tensor,
                // but the 2 eigenvectors of the tensor instead.
                QVector2D scaledVector(tensor.x(),tensor.y());
                scaledVector.setX(scaledVector.x()*du/2.0f);
                scaledVector.setY(scaledVector.y()*dv/2.0f);
                QVector2D tip = base + scaledVector;
                roundVector2D(base);
                roundVector2D(tip);
                painter.drawLine(base.x(),base.y(),tip.x(),tip.y());
            }
            if(drawVector2)
            {
                painter.setPen(pen2);
                QVector2D base = origin + QVector2D(i*dv, j*du);
                QVector4D tensor = mData[i][j].normalized();
                QVector2D scaledVector(tensor.z(),tensor.w());
                scaledVector.setX(scaledVector.x()*du/2.0f);
                scaledVector.setY(scaledVector.y()*dv/2.0f);
                QVector2D tip = base + scaledVector;
                roundVector2D(base);
                roundVector2D(tip);
                painter.drawLine(base.x(),base.y(),tip.x(),tip.y());
            }
        }
    }

//    QLabel l;
//    l.setPixmap(pixmap);
//    l.show();
    emit newTensorFieldImage(pixmap);
    return pixmap;
}


/** ******************** */
/** Non-member Functions */
/** ******************** */



void roundVector2D(QVector2D vec)
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
        std::cerr<<"The tensor must be traceless and symmetrical"<<std::endl;
        return QVector4D();
    }
    QVector2D lambdas = getTensorEigenValues(tensor);
    if(abs(tensor.y()) > FLOAT_COMPARISON_EPSILON) // if b == 0
    {
        if(abs(tensor.x()) > FLOAT_COMPARISON_EPSILON) // if a == 0
        {
            std::cout<<"WARNING - Null tensor: the point is degenerate"<<std::endl;
            return QVector4D(0.0f,0.0f,0.0f,0.0f);
        }
        return QVector4D(1.0f,0.0f,0.0f,1.0f);
    }
    // Vi = (Li + a, b)
    QVector2D vec1(lambdas[0]+tensor.x(),tensor.y());
    QVector2D vec2(lambdas[1]+tensor.x(),tensor.y());
    vec1.normalize();
    vec2.normalize();
    return QVector4D(vec1.x(), vec1.y(),vec2.x(), vec2.y());
}

QVector2D getTensorEigenValues(QVector4D tensor)
{
    // lambda = +/- sqrt(a^2 +b^2)
    float lambda1 = sqrt(tensor.x()*tensor.x() + tensor.y()*tensor.y());
    return QVector2D(lambda1,-lambda1);
}

QVector2D getMajorEigenVector(QVector4D tensor)
{
    return getFirstVector(getTensorEigenVectors(tensor));
}

QVector2D getMinorEigenVector(QVector4D tensor)
{
    return getSecondVector(getTensorEigenVectors(tensor));
}

bool isSymetricalAndTraceless(QVector4D tensor)
{
    return tensor.y() == tensor.z()
            && (tensor.x() + tensor.w() > FLOAT_COMPARISON_EPSILON );
}

bool isDegenerate(QVector4D tensor)
{
    return tensor.x() < FLOAT_COMPARISON_EPSILON
            && tensor.y() < FLOAT_COMPARISON_EPSILON
            && tensor.z() < FLOAT_COMPARISON_EPSILON
            && tensor.w() < FLOAT_COMPARISON_EPSILON;
}
