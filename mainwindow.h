#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSize>
#include <QKeyEvent>

#include "TensorField.h"
#include "StreetGraph.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    void keyPressEvent(QKeyEvent *event);

private slots:
    void displayVectorFieldImage(QPixmap image);

private:
    Ui::MainWindow *ui;
    TensorField * mTensorField;
    QSize mTensorFieldSize;
    StreetGraph * mStreetGraph;
};

#endif // MAINWINDOW_H
