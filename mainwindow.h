#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSize>

#include "TensorField.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void displayVectorFieldImage(QPixmap image);

private:
    Ui::MainWindow *ui;
    TensorField * mTensorField;
    QSize mTensorFieldSize;
};

#endif // MAINWINDOW_H
