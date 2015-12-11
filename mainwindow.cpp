#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->mTensorFieldSize = QSize(32,32);
    this->mTensorField = new TensorField(mTensorFieldSize);

    QObject::connect(ui->buttonGenerateTF, SIGNAL(clicked()), mTensorField, SLOT(generateTensorField()));
    QObject::connect(mTensorField, SIGNAL(newTensorFieldImage(QPixmap)),ui->labelTensorFieldDisplay,SLOT(setPixmap(QPixmap)));
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::displayVectorFieldImage(QPixmap image)
{
    ui->labelTensorFieldDisplay->setPixmap(image);
}
