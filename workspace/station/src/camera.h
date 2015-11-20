#ifndef CAMERA_H
#define CAMERA_H

#include <QThread>
#include <QWidget>
#include <QImage>
#include <QTimer>
#include <QTime>
#include <QDateTime>
#include <QDir>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <iostream>
#include <string.h>

//注意：这里要把自己的系统用户名改掉，不然无法保存截图
#define  system_user_name "chg"

//显示图幅
#define image_area_width 784
#define image_area_height 441

//相机参数（mm）
#define camera_APS_width 23.5
#define camera_APS_height 15.6
#define camera_focal_length 20.0

using namespace std;

class Camera:public QThread
{
    Q_OBJECT
public:
    Camera();
    ~Camera();
    void run();

    QImage image;
    QImage image_temp[100];
    bool bool_show_Image;
    bool bool_open_camera;
    bool bool_show_ruler;
    unsigned int image_counter;
    char image_name[50];

public slots:
    void openCamara();      // 打开摄像头
    void readFarme();       // 读取当前帧信息
    void closeCamara();     // 关闭摄像头。
    void takingPictures();  // 拍照

private:
    QTime current_time;
    QDateTime current_date_time;
    QTimer *timer;
    CvCapture *cam;// 视频获取结构， 用来作为视频获取函数的一个参数
    IplImage  *frame;//申请IplImage类型指针，就是申请内存空间来存放每一帧图像

    bool bool_image_capture;//是否截图判断标志
    char image_name_save[100];
    char image_name_print[50];

    void camera_Send_Image()const {emit camera_Image_Signal();}
    void camera_Send_Capture()const{emit camera_Capture_Show_Signal();}

    CvScalar yellow;
    int line_thickness;
    float optical_distance_last;

    char path_image_save_partial[50];//截图保存路径

signals:
    void camera_Image_Signal()const;
    void camera_Capture_Show_Signal()const;
};

#endif
