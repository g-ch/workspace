//******mainwindow.h*******

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <string>
#include <QListWidgetItem>
#include <iostream>
#include "painterWiget.h"
#include <QTime>
#include <QTimer>
#include <QDateTime>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QKeyEvent>

#define SUCCESS_COUNTER_INIT 100

using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    StatusPainter *status_painter;//用于画图的类指针
    QPixmap *compass_arrow_pixmap;

protected:
    void paintEvent(QPaintEvent *event);
    void get_Painter_Address(StatusPainter *painter);//用于画图传参的函数

private slots:
    void state_Mode_Slot();//模式控制显示
    void battey_Slot();//电量显示
    void radio_Status_Slot();//信号强度显示
    void global_Velocity_Slot();//速度显示
    void global_GPS_Slot();//GPS信息显示
    void global_GPS_Satellites_Slot();//GPS星数
    void global_Rel_Alt_Slot();//相对高度显示
    void local_Position_Slot();//姿态显示
    void optical_Flow_Slot();//光流显示
    void temperature_Slot();//温度显示
    void time_Slot();//飞行时间显示
    void offboard_Set_Slot();
    void field_Size_Confirm_Slot();
    void pump_Status_Slot();
    void timer_Slot();


    //void on_comboBox_Num_currentIndexChanged(int index);

    void time_Update();

    void on_pushButton_Reset_FlyingTime_clicked();

    void on_pushButton_Route_Send_clicked();

    void on_pushButton_Route_Generate_clicked();

    void on_pushButton_Route_Reset_clicked();

    void on_pushButton_OFFBOARD_Imitate_clicked();


    void on_horizontalSlider_Spray_actionTriggered(int action);

private:
    Ui::MainWindow *ui;
    QTime system_time;
    QImage image_resize;

    QPoint mouse_pos;

};

#endif // MAINWINDOW_H
