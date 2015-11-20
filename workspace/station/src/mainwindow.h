#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <string>
#include <QListWidgetItem>
#include <iostream>
#include "painterWiget.h"
#include <QTime>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QKeyEvent>

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
    bool eventFilter(QObject *obj,QEvent *event);

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

    void camera_Image_Slot();//摄像头图像显示
    void camera_Capture_Show_Slot();//截图显示在scrollarea

    void on_pushButton_clicked();
    void on_pushButton_Open_Video_clicked();
    void on_pushButton_Close_Video_clicked();
    void on_pushButton_Capture_Video_clicked();

    void on_spinBox_Num_editingFinished();
    void on_comboBox_Num_currentIndexChanged(int index);


    void on_listWidget_itemDoubleClicked(QListWidgetItem *item);//用于预览图片
    void on_pushButton_Preview_clicked();
    void on_listWidget_itemClicked(QListWidgetItem *item);
    void on_listWidget_itemChanged(QListWidgetItem *item);

    void time_Update();

    void on_pushButton_Reset_FlyingTime_clicked();

    void on_pushButton_Image_Recovery_clicked();

    void on_checkBox_Ruler_clicked();

private:
    Ui::MainWindow *ui;
    QTime system_time;
    QImage image_resize;

    float preview_scale_width;
    float preview_scale_height;
    int preview_current_num;
    QPoint mouse_pos;

};

#endif // MAINWINDOW_H
