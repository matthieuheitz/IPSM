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

    ui->comboBoxSeedInit->addItem("Regular Grid");
    ui->comboBoxSeedInit->addItem("Random distribution");
    ui->comboBoxSeedInit->addItem("Controlled random distribution");

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
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::displayVectorFieldImage(QPixmap image)
{
    ui->labelTensorFieldDisplay->setPixmap(image);
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if (event->key()==Qt::Key_Escape)
        exit(0);
}
