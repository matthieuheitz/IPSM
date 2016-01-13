#include "TensorField.h"
#include "math.h"
#include "iostream"
#include "eigen3/Eigen/Dense"

#include <QPainter>
#include <QPen>
#include <QFileDialog>


TensorField::TensorField(QSize fieldSize, QObject *parent) :
    QObject(parent), mFieldSize(fieldSize)
{
    mData.resize(fieldSize.height());
    for(int i=0 ; i < fieldSize.height() ; i++)
    {
        mData[i].resize(fieldSize.width());
    }
    mFieldIsFilled = false;
    mEigenIsComputed = false;
    mWaterMapIsLoaded = false;
}

QVector4D TensorField::getTensor(int i, int j)
{
    return mData[i][j];
}

void TensorField::setTensor(int i, int j, QVector4D tensor)
{
    mData[i][j] = tensor;
}

void TensorField::setFieldSize(QSize fieldSize)
{
    mFieldSize = fieldSize;
    mData.resize(fieldSize.height());
    for(int i=0 ; i < fieldSize.height() ; i++)
    {
        mData[i].resize(fieldSize.width());
    }
    mFieldIsFilled = false;
}

void TensorField::applyWaterMap(QString filename)
{
    QImage waterMap = QImage(filename);
    if(waterMap.isNull())
    {
        qCritical()<<"applyWaterMap(): File "<<filename<<" not found";
        return;
    }
    if(waterMap.size() != mFieldSize)
    {
        qCritical()<<"applyWaterMap(): Watermap must be of same size as the tensor field";
        return;
    }
    for(int i=0; i<waterMap.height() ; i++)
    {
        for(int j=0; j<waterMap.width() ; j++)
        {
            if(qBlue(waterMap.pixel(j,i)) > 0)
            {
                mData[mFieldSize.height()-1-i][j] = QVector4D(0,0,0,0);
            }
        }
    }
    mWatermapFilename = filename;
    mWaterMapIsLoaded = true;
}

void TensorField::applyBoundaries(QString filename)
{
    QImage waterMap = QImage(filename);
    if(waterMap.isNull())
    {
        qCritical()<<"applyWaterMap(): File "<<filename<<" not found";
        return;
    }
    if(waterMap.size() != mFieldSize)
    {
        qCritical()<<"applyWaterMap(): Watermap must be of same size as the tensor field";
        return;
    }
    QImage mapSobelX, mapSobelY;
    QColor pixSobelX, pixSobelY;
    float theta, r;

    mapSobelX = applySobelX(waterMap);
    mapSobelY = applySobelY(waterMap);

    for(int i=0; i<mFieldSize.width()-1 ; i++)
    {
        for(int j=0; j<mFieldSize.height()-1 ; j++)
        {
            QVector4D tensor;

            pixSobelX = mapSobelX.pixel(j,i);
            pixSobelY = mapSobelY.pixel(j,i);

            theta = std::atan2(abs(pixSobelY.blue()),abs(pixSobelX.blue()))+ M_PI/2.0;
            r = std::sqrt(std::pow(pixSobelY.blue(),2.0) + std::pow(pixSobelX.blue(),2.0));

            tensor.setX(cos(2.0*theta));
            tensor.setY(sin(2.0*theta));
            tensor.setZ(sin(2.0*theta));
            tensor.setW(-cos(2.0*theta));
            tensor *= r;

            if (!tensor.isNull()){
                mData[mFieldSize.width() -1 -i][j] += tensor;
                if (j<mFieldSize.height()-2 && j>0 && i<mFieldSize.width()-2 && i>0){
                    mData[mFieldSize.width() -i][j] += tensor;
                    mData[mFieldSize.width() -1 -i][j+1] += tensor;
                    mData[mFieldSize.width() -2 -i][j] += tensor;
                    mData[mFieldSize.width() -1 -i][j-1] += tensor;
                    mData[mFieldSize.width() -i][j+1] += tensor;
                    mData[mFieldSize.width() -2 -i][j+1] += tensor;
                    mData[mFieldSize.width() -2 -i][j-1] += tensor;
                    mData[mFieldSize.width() -i][j-1] += tensor;
                }
            }
        }
    }
    mFieldIsFilled = true;
}

void TensorField::fillGridBasisField(float theta, float l, QPointF center, float decay)
{
    float x;
    float y;
    for(int i=0; i<mFieldSize.height() ; i++)
    {
        for(int j=0; j<mFieldSize.width() ; j++)
        {
            QVector4D tensor;
            x = ((float)j/(mFieldSize.height()-1) - center.x());
            y = ((float)i/(mFieldSize.width()-1) - center.y());
            tensor.setX(cos(2.0*theta));
            tensor.setY(sin(2.0*theta));
            tensor.setZ(sin(2.0*theta));
            tensor.setW(-cos(2.0*theta));
            tensor *= l;
            mData[i][j] += (1.0-std::exp(-decay*(std::pow(x,2.0)+std::pow(y,2.0))))*tensor;
        }
    }
    mFieldIsFilled = true;
}

void TensorField::fillRotatingField(QPointF center, float decay)
{
    float x;
    float y;
    for(int i=0; i<mFieldSize.height() ; i++)
    {
        for(int j=0; j<mFieldSize.width() ; j++)
        {
            float theta = M_PI*j/(mFieldSize.width()-1) + i*M_PI/4/(mFieldSize.height()-1);
            QVector4D tensor;
            x = ((float)j/(mFieldSize.height()-1) - center.x());
            y = ((float)i/(mFieldSize.width()-1) - center.y());
            tensor.setX(cos(2.0*theta));
            tensor.setY(sin(2.0*theta));
            tensor.setZ(sin(2.0*theta));
            tensor.setW(-cos(2.0*theta));
            mData[i][j] += std::exp(-decay*(std::pow(x,2.0)+std::pow(y,2.0)))*tensor;
        }
    }
    mFieldIsFilled = true;
}

void TensorField::fillGridBasisField(QVector2D direction)
{
    float theta = std::atan2(direction.y(),direction.x());
    this->fillGridBasisField(theta, direction.length(), QPoint(0.2, 0.3), 0.1);
}

void TensorField::fillHeightBasisField(QString filename, QPointF center, float decay)
{
    QImage mHeightMap = QImage(filename);
    float x;
    float y;
    if(mHeightMap.isNull())
    {
        qCritical()<<"fillHeightBasisField(): File "<<filename<<" not found";
        return;
    }
    this->setFieldSize(mHeightMap.size());
    QRgb currentPixel, nextPixelHoriz, nextPixelVert;
    QVector2D grad;
    float theta, r;
    // Origin is top-left in the image
    // Origin is bottom-left in the tensor matrix
    // We have to swap the vertical axis
    for(int i=0; i<mFieldSize.height()-1 ; i++)
    {
        for(int j=0; j<mFieldSize.width()-1 ; j++)
        {
            QVector4D tensor;
            x = ((float)j/(mFieldSize.height()-1) - center.x());
            y = ((float)i/(mFieldSize.width()-1) - center.y());
            currentPixel = mHeightMap.pixel(j,i);
            nextPixelHoriz = mHeightMap.pixel(j+1,i);
            nextPixelVert = mHeightMap.pixel(j,i+1);
            // If gradient is null, set tensor to default instead
            // of degenerate
            if(nextPixelHoriz == currentPixel && nextPixelVert == currentPixel)
            {
                mData[mFieldSize.width()-1-i][j] = QVector4D(1,0,0,-1);
            }
            else
            {
                grad.setX(qBlue(currentPixel)-qBlue(nextPixelHoriz));
                grad.setY(qBlue(currentPixel)-qBlue(nextPixelVert));
                // Invert y
                theta = std::atan2(-grad.y(), grad.x()) + M_PI/2.0;
                r = std::sqrt(std::pow(grad.y(),2.0) + std::pow(grad.x(),2.0));
                tensor.setX(cos(2.0*theta));
                tensor.setY(sin(2.0*theta));
                tensor.setZ(sin(2.0*theta));
                tensor.setW(-cos(2.0*theta));
                tensor *= r;

                mData[mFieldSize.height()-1-i][j] += std::exp(-decay*(std::pow(x,2.0)+std::pow(y,2.0)))*tensor;
            }
        }
    }
    mFieldIsFilled = true;
}

void TensorField::fillHeightBasisFieldSobel(QString filename,QPointF center, float decay)
{
    QImage mHeightMap = QImage(filename);
    float x;
    float y;
    if(mHeightMap.isNull())
    {
        qCritical()<<"fillHeightBasisField(): File "<<filename<<" not found";
        return;
    }
    this->setFieldSize(mHeightMap.size());
    QImage mapSobelX, mapSobelY;
    QColor pixSobelX, pixSobelY;
    float theta, r;

    mapSobelX = applySobelX(mHeightMap);
    mapSobelY = applySobelY(mHeightMap);

    for(int i=0; i<mFieldSize.width()-1 ; i++)
    {
        for(int j=0; j<mFieldSize.height()-1 ; j++)
        {
            QVector4D tensor;
            x = ((float)j/(mFieldSize.height()-1) - center.x());
            y = ((float)i/(mFieldSize.width()-1) - center.y());

            pixSobelX = mapSobelX.pixel(j,i);
            pixSobelY = mapSobelY.pixel(j,i);

            theta = std::atan2(abs(pixSobelY.blue()),abs(pixSobelX.blue()))+ M_PI/2.0;
            r = std::sqrt(std::pow(pixSobelY.blue(),2.0) + std::pow(pixSobelX.blue(),2.0));

            tensor.setX(cos(2.0*theta));
            tensor.setY(sin(2.0*theta));
            tensor.setZ(sin(2.0*theta));
            tensor.setW(-cos(2.0*theta));
            tensor *= r;
            mData[mFieldSize.width() -1 -i][j] += std::exp(-decay*(std::pow(x,2.0)+std::pow(y,2.0)))*tensor;
        }
    }
    mFieldIsFilled = true;
}

void TensorField::fillRadialBasisField(QPointF center, float decay)
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
            mData[i][j] += std::exp(-decay*(std::pow(x,2.0)+std::pow(y,2.0)))*tensor;
        }
    }
    mFieldIsFilled = true;
}

void TensorField::actionAddWatermap()
{
    if(!mFieldIsFilled)
    {
        qCritical()<<"actionApplyWatermap(): Tensor field is null. Initialize it first";
        return;
    }
    QString filename = QFileDialog::getOpenFileName(0, QString("Open Image"));
    if(filename.isEmpty())
    {
        return;
    }
    this->applyWaterMap(filename);
    this->applyBoundaries(filename);

    this->computeTensorsEigenDecomposition();
    this->exportEigenVectorsImage(true, true);
}

void TensorField::generateGridTensorField()
{
    this->fillGridBasisField(M_PI/3, 0.01, QPointF(0.2, 0.3), 0.1);

    this->computeTensorsEigenDecomposition();
    this->exportEigenVectorsImage(true, true);
}

void TensorField::generateHeightmapTensorField()
{
    QString filename = QFileDialog::getOpenFileName(0, QString("Open Image"));
    if(filename.isEmpty())
    {
        return;
    }
    this->fillHeightBasisField(filename, QPointF(0.2, 0.3), 0.1);

    this->computeTensorsEigenDecomposition();
    this->exportEigenVectorsImage(true, true);
}

void TensorField::generateMultiRotationTensorField()
{
    this->fillRotatingField(QPointF(0.7, 0.3), 100.0);

    this->computeTensorsEigenDecomposition();
    this->exportEigenVectorsImage(true, true);
}

void TensorField::generateRadialTensorField()
{
    this->fillRadialBasisField(QPointF(0.2,0.3), 100.0);

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

    if(!mFieldIsFilled)
    {
        qCritical()<<"exportEigenVectorsImage(): Tensor field is empty";
        return pixmap;
    }

    QPainter painter(&pixmap);
    QPen pen1(color1);
    QPen pen2(color2);

    float dv = imageSize/(float)mFieldSize.height();
    float du = imageSize/(float)mFieldSize.width();
    QVector2D origin(du/2.0f, dv/2.0f);

    int numberOfTensorsToDisplay = 32;
    int scaleI = mFieldSize.height()/numberOfTensorsToDisplay;
    int scaleJ = mFieldSize.width()/numberOfTensorsToDisplay;

    for(int i=0; i<mFieldSize.width() ; i=i+scaleI)
    {
        for(int j=0; j<mFieldSize.height() ; j=j+scaleJ)
        {
            if(drawVector1)
            {
                painter.setPen(pen1);
                QVector2D base = origin + QVector2D(j*dv, i*du);
                QVector2D eigenVector = getTensorMajorEigenVector(mData[i][j]);
                eigenVector.setX(eigenVector.x()*du/2.0f*scaleI*0.8);
                eigenVector.setY(eigenVector.y()*dv/2.0f*scaleJ*0.8);
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
                eigenVector.setX(eigenVector.x()*du/2.0f*scaleI*0.8);
                eigenVector.setY(eigenVector.y()*dv/2.0f*scaleJ*0.8);
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
        qCritical()<<"computeTensorsEigenDecomposition(): Fill the tensor field before computing the eigen vectors";
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
        qCritical()<<"getEigenValues(): Unable to get the eigen values."
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
        qCritical()<<"getTensorEigenVectors(): The tensor must be traceless and symetrical";
        return QVector4D();
    }
    if(isDegenerate(tensor))
    {
        return QVector4D(0,0,0,0);
    }
    if(isFuzzyEqual(tensor.x(), 1) && isFuzzyEqual(tensor.y(), 0))
    {
        return QVector4D(1,0,0,1);
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
    if(isDegenerate(tensor))
    {
        return QVector2D(0,0);
    }
    if(isFuzzyEqual(tensor.x(), 1) && isFuzzyEqual(tensor.y(), 0))
    {
        return QVector2D(1,-1);
    }
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

QImage applySobelX(QImage map)
{
    QSize size;
    size = map.size();
    QImage sobelX(size,QImage::Format_RGB32);
    sobelX.fill(0);
    float kii[9], mii[9];
    kii[0] = -1.0f;
    kii[1] = 0.0f;
    kii[2] = 1.0f;
    kii[3] = -2.0f;
    kii[4] = 0.0f;
    kii[5] = 2.0f;
    kii[6] = -1.0f;
    kii[7] = 0.0f;
    kii[8] = 1.0f;

    QMatrix3x3 kernel(kii);

    for (int i=1; i<size.width()-2; i++)
    {
        for (int j=1; j<size.height()-2; j++)
        {
            mii[0] = qBlue(map.pixel(i-1,j-1));
            mii[1] = qBlue(map.pixel(i,j-1));
            mii[2] = qBlue(map.pixel(i+1,j-1));
            mii[3] = qBlue(map.pixel(i-1,j));
            mii[4] = qBlue(map.pixel(i,j));
            mii[5] = qBlue(map.pixel(i+1,j));
            mii[6] = qBlue(map.pixel(i-1,j+1));
            mii[7] = qBlue(map.pixel(i,j+1));
            mii[8] = qBlue(map.pixel(i+1,j+1));
            QMatrix3x3 matrix(mii);

            sobelX.setPixel(i,j,(sumMat3D(matrix,kernel)));
        }
    }
    return sobelX;
}

QImage applySobelY(QImage map)
{
    QSize size;
    size = map.size();
    QImage sobelY(size,QImage::Format_RGB32);
    sobelY.fill(0);
    float kii[9], mii[9];
    kii[0] = -1.0f;
    kii[1] = -2.0f;
    kii[2] = -1.0f;
    kii[3] = 0.0f;
    kii[4] = 0.0f;
    kii[5] = 0.0f;
    kii[6] = 1.0f;
    kii[7] = 2.0f;
    kii[8] = 1.0f;

    QMatrix3x3 kernel(kii);

    for (int i=1; i<size.width()-2; i++)
    {
        for (int j=1; j<size.height()-2; j++)
        {
            mii[0] = qBlue(map.pixel(i-1,j-1));
            mii[1] = qBlue(map.pixel(i,j-1));
            mii[2] = qBlue(map.pixel(i+1,j-1));
            mii[3] = qBlue(map.pixel(i-1,j));
            mii[4] = qBlue(map.pixel(i,j));
            mii[5] = qBlue(map.pixel(i+1,j));
            mii[6] = qBlue(map.pixel(i-1,j+1));
            mii[7] = qBlue(map.pixel(i,j+1));
            mii[8] = qBlue(map.pixel(i+1,j+1));
            QMatrix3x3 matrix(mii);

            sobelY.setPixel(i,j,(sumMat3D(matrix,kernel)));
        }
    }
    return sobelY;
}

int sumMat3D(QMatrix3x3 matrix, QMatrix3x3 kernel)
{
    int sum;
    sum = 0;
    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {
            sum += matrix(i,j)*kernel(i,j);
        }
    }
    return sum;
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
