#include "TensorField.h"
#include "math.h"

#include <QPainter>
#include <QPen>


TensorField::TensorField(QSize fieldSize, QObject *parent) :
    QObject(parent), mFieldSize(fieldSize)
{
    this->mData.resize(fieldSize.height());
    for(int i=0 ; i < fieldSize.height() ; i++)
    {
        mData[i].resize(fieldSize.width());
    }
}


void roundVector2D(QVector2D vec)
{
    vec.setX(round(vec.x()));
    vec.setY(round(vec.y()));
}

// Get the tensor at index (i,j)
QVector4D TensorField::getTensor(int i, int j)
{
    return mData[i][j];
}

// Set the tensor at index (i,j)
void TensorField::setTensor(int i, int j, QVector4D tensor)
{
    this->mData[i][j] = tensor;
}

// Generate a grid basis function from an angle and
// a vector's length (norm)
void TensorField::fillGridBasisField(float theta, float l)
{
    float k = 2.0;
    for(int i=0; i<this->mFieldSize.height() ; i++)
    {
        for(int j=0; j<this->mFieldSize.width() ; j++)
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
}

// Generate a grid basis field from a 2D vector
// Don't normalize the vector as this function
// integrates the vector's norm in the tensor
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
    for(int i=0; i<this->mFieldSize.height() ; i++)
    {
        for(int j=0; j<this->mFieldSize.width() ; j++)
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

    float dv = imageSize/(float)this->mFieldSize.height();
    float du = imageSize/(float)this->mFieldSize.width();
    QVector2D origin(du/2.0f, dv/2.0f);

    for(int i=0; i<this->mFieldSize.height() ; i++)
    {
        for(int j=0; j<this->mFieldSize.width() ; j++)
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
