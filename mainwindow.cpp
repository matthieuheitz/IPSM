#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    double separationDistance = 10;
    mTensorFieldSize = QSize(32,32);
    mTensorField = new TensorField(mTensorFieldSize);
    mStreetGraph = new StreetGraph(QPointF(0,0), QPointF(100,100),mTensorField,separationDistance);
    mTensorFieldIsGlobal = true;

    ui->comboBoxSeedInit->addItem("Regular Grid");
    ui->comboBoxSeedInit->addItem("Random distribution");
    ui->comboBoxSeedInit->addItem("Poisson distribution");

    ui->spinBoxDensity->setRange(0, 100);
    ui->spinBoxDensity->setValue(separationDistance);

    QObject::connect(ui->buttonAddWatermap, SIGNAL(clicked()),
                     mTensorField, SLOT(actionAddWatermap()));
    QObject::connect(ui->buttonGenerateGridTF, SIGNAL(clicked()),
                     mTensorField, SLOT(generateGridTensorField()));
    QObject::connect(ui->buttonGenerateMultiRotTF, SIGNAL(clicked()),
                     mTensorField, SLOT(generateMultiRotationTensorField()));
    QObject::connect(ui->buttonGenerateRadialTF, SIGNAL(clicked()),
                     mTensorField, SLOT(generateRadialTensorField()));
    QObject::connect(ui->buttonGenerateHeightmapTF, SIGNAL(clicked()),
                     mTensorField, SLOT(generateHeightmapTensorField()));
    QObject::connect(ui->buttonSmoothTF, SIGNAL(clicked()),
                     mTensorField, SLOT(smoothTensorField()));
    QObject::connect(mTensorField, SIGNAL(newTensorFieldImage(QPixmap)),
                     ui->labelTensorFieldDisplay,SLOT(setPixmap(QPixmap)));
    QObject::connect(ui->buttonGeneratePrincipalRG, SIGNAL(clicked()),
                     mStreetGraph, SLOT(generateStreetGraph()));
    QObject::connect(mStreetGraph, SIGNAL(newStreetGraphImage(QPixmap)),
                     ui->labelRoadmapDisplay, SLOT(setPixmap(QPixmap)));
    QObject::connect(ui->checkBoxShowNodes, SIGNAL(toggled(bool)),
                     mStreetGraph, SLOT(setDrawNodes(bool)));
    QObject::connect(ui->spinBoxDensity, SIGNAL(valueChanged(double)),
                     mStreetGraph, SLOT(setSeparationDistance(double)));
    QObject::connect(ui->comboBoxSeedInit, SIGNAL(currentIndexChanged(int)),
                     mStreetGraph, SLOT(changeSeedInitMethod(int)));
    QObject::connect(ui->buttonResetTF, SIGNAL(clicked()),
                     mTensorField, SLOT(resetTensorField()));
    QObject::connect(ui->buttonGlobal, SIGNAL(clicked()),
                     this, SLOT(updateTFState()));
    QObject::connect(ui->buttonLocal, SIGNAL(clicked()),
                     this, SLOT(updateTFState()));
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::displayVectorFieldImage(QPixmap image)
{
    ui->labelTensorFieldDisplay->setPixmap(image);
}

void MainWindow::updateTFState()
{
    mTensorFieldIsGlobal = !mTensorFieldIsGlobal;
    std::cout<<mTensorFieldIsGlobal<<std::endl;
    if (mTensorFieldIsGlobal)
    {
        QObject::connect(ui->buttonGenerateGridTF, SIGNAL(clicked()),
                         mTensorField, SLOT(generateGridTensorField()));
        QObject::connect(ui->buttonGenerateMultiRotTF, SIGNAL(clicked()),
                         mTensorField, SLOT(generateMultiRotationTensorField()));
        QObject::connect(ui->buttonGenerateRadialTF, SIGNAL(clicked()),
                         mTensorField, SLOT(generateRadialTensorField()));
        QObject::disconnect(ui->buttonGenerateGridTF, SIGNAL(clicked()),
                         mTensorField, SLOT(generateGridTensorFieldLocal()));
        QObject::disconnect(ui->buttonGenerateMultiRotTF, SIGNAL(clicked()),
                         mTensorField, SLOT(generateMultiRotationTensorFieldLocal()));
        QObject::disconnect(ui->buttonGenerateRadialTF, SIGNAL(clicked()),
                         mTensorField, SLOT(generateRadialTensorFieldLocal()));
    }
    else
    {
        QObject::disconnect(ui->buttonGenerateGridTF, SIGNAL(clicked()),
                         mTensorField, SLOT(generateGridTensorField()));
        QObject::disconnect(ui->buttonGenerateMultiRotTF, SIGNAL(clicked()),
                         mTensorField, SLOT(generateMultiRotationTensorField()));
        QObject::disconnect(ui->buttonGenerateRadialTF, SIGNAL(clicked()),
                         mTensorField, SLOT(generateRadialTensorField()));
        QObject::connect(ui->buttonGenerateGridTF, SIGNAL(clicked()),
                         mTensorField, SLOT(generateGridTensorFieldLocal()));
        QObject::connect(ui->buttonGenerateMultiRotTF, SIGNAL(clicked()),
                         mTensorField, SLOT(generateMultiRotationTensorFieldLocal()));
        QObject::connect(ui->buttonGenerateRadialTF, SIGNAL(clicked()),
                         mTensorField, SLOT(generateRadialTensorFieldLocal()));
    }
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if (event->key()==Qt::Key_Escape)
        exit(0);
}
