//***********camera.cpp***********

#include "camera.h"
#include "stdio.h"
#include <string>
#include <iostream>
#include "receiver.h"

using namespace std;

extern string serial_num;
extern MavrosMessage message;


Camera::Camera()
{
    cam = NULL;
    timer = new QTimer();
    connect(timer, SIGNAL(timeout()), this, SLOT(readFarme()));

    //创建截图存储的文件夹home/chg/Pictures/Image
    QDir *temp = new QDir;
    char path_head[7]="home/";
    char path_dir[30];
    sprintf(path_dir,"/Pictures/Capture_Images");

    strcat(path_image_save_partial,path_head);
    strcat(path_image_save_partial,system_user_name);
    strcat(path_image_save_partial,path_dir);

    char full_path_image_save[50]="/";
    strcat(full_path_image_save,path_image_save_partial);

    bool exist = temp->exists(QString(full_path_image_save));
    if(!exist)temp->mkdir(QString(full_path_image_save));
    //cout<<full_path_image_save<<endl;

    bool_show_Image=true;
    bool_image_capture=false;
    bool_show_ruler=true;
    image_counter=0;

    line_thickness=2;
    yellow=cvScalar(0,255,255);
    optical_distance_last=0.0;

    //测试像素代码
    //cvNamedWindow("TEST");
}

Camera::~Camera()
{
    delete timer;
}

void Camera::run()
{
    //openCamara();
    //readFarme();
}

void Camera::openCamara()
{
    cam = cvCreateCameraCapture(1);//打开摄像头，从摄像头中获取视频

    //设定捕获图像大小及帧率
    cvSetCaptureProperty(cam,CV_CAP_PROP_FPS,30);
    cvSetCaptureProperty(cam,CV_CAP_PROP_FRAME_WIDTH,1920);
    cvSetCaptureProperty(cam,CV_CAP_PROP_FRAME_HEIGHT,1080);

    timer->start(33);              // 开始计时，超时则发出timeout()信号，30帧/s
    bool_open_camera=true;
}

void Camera::readFarme()
{
    //cvNamedWindow("video",1);
    IplImage* frame_raw = cvQueryFrame(cam);// 从摄像头中抓取并返回每一帧

    //测试代码
    //cout<<"raw"<<frame_raw->width<<"  "<<frame_raw->height<<endl;
    //cvShowImage("TEST",frame_raw);

    //标尺加入
    if(bool_show_ruler)
    {
        //画线
        cvLine(frame_raw,cvPoint(((int)frame_raw->width*0.1),((int)frame_raw->height*0.2)),cvPoint(((int)frame_raw->width*0.85),((int)frame_raw->height*0.2)),yellow,line_thickness);
        for(float i=0.03;i<0.75;i+=0.03)
        {
            cvLine(frame_raw,cvPoint((int)frame_raw->width*(0.1+i),(int)frame_raw->height*0.2),cvPoint((int)frame_raw->width*(0.1+i),(int)frame_raw->height*0.2+10),yellow,line_thickness);
        }
        cvLine(frame_raw,cvPoint(((int)frame_raw->width*0.1),((int)frame_raw->height*0.2)),cvPoint(((int)frame_raw->width*0.1),((int)frame_raw->height*0.85)),yellow,line_thickness);
        for(float i=0.05;i<0.7;i+=0.05)
        {
            cvLine(frame_raw,cvPoint((int)frame_raw->width*0.1,(int)frame_raw->height*(0.2+i)),cvPoint((int)frame_raw->width*0.1+10,(int)frame_raw->height*(0.2+i)),yellow,line_thickness);
        }

        //存储目标距离并去抖动
        if(fabs(optical_distance_last-message.optical_flow.distance)>0.1)
            optical_distance_last=message.optical_flow.distance;
        //分度
        char width_unit_real_mm_char[20];
        char height_unit_real_mm_char[20];

        if(optical_distance_last!=0)
        {
            //整体画幅对应实际尺寸
            float width_total_real_m = camera_APS_width / camera_focal_length * optical_distance_last;
            float height_total_real_m = camera_APS_height / camera_focal_length * optical_distance_last;

            //1/32横向，1/18纵向画幅对应尺寸
            int width_unit_real_mm = (int)(width_total_real_m*1000/32);
            int height_unit_real_mm = (int)(height_total_real_m*1000/18);

            //转为字符串
            sprintf(width_unit_real_mm_char,"%d mm",width_unit_real_mm);
            sprintf(height_unit_real_mm_char,"%d mm",height_unit_real_mm);
        }
        else
        {
            sprintf(width_unit_real_mm_char,"NULL");
            sprintf(height_unit_real_mm_char,"NULL");
        }

        //写标尺分度(****相机感光元件比例并非16：9，但近似，由于本身标尺误差较大，此处直接认为图像在16：9时未变形，虽然纵向单位长度算出了，但还是用横向的，方便查看)
        CvFont font;
        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,1,1,0,2);
        cvPutText(frame_raw,width_unit_real_mm_char,cvPoint((int)(frame_raw->width*0.8),(int)(frame_raw->height*0.2-20)),&font,yellow);
        cvPutText(frame_raw,width_unit_real_mm_char,cvPoint((int)(frame_raw->width*0.1-15),(int)(frame_raw->height*0.9+10)),&font,yellow);
    }

    //窗口大小适应
    if(frame_raw->height>image_area_height||frame_raw->width>image_area_width)frame=cvCreateImage(cvSize(image_area_width,image_area_height),8,3);//4:3画面
    else frame=frame_raw;
    cvResize(frame_raw,frame,CV_INTER_NN);

     // 将抓取到的帧，转换为QImage格式。QImage::Format_RGB888不同的摄像头用不同的格式。
    image = QImage((const uchar*)frame->imageData, frame->width, frame->height,QImage::Format_RGB888).rgbSwapped();

    if(bool_show_Image)camera_Send_Image();

    if(bool_image_capture)
    {
        //储存最大100幅临时截图
        CvFont font;
        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, 0.8,0.8,0,2);//初始化字体(字体名，字体格式，横向大小比例，纵向大小比例，斜度，粗细)
        cvPutText(frame_raw,image_name_print,cvPoint(frame_raw->width-800,frame_raw->height-15),&font,CV_RGB(255,255,0));//在图片中输出字符

        image_temp[image_counter] = QImage((const uchar*)frame_raw->imageData, frame_raw->width, frame_raw->height,QImage::Format_RGB888).rgbSwapped();
        if(image_counter==100)image_counter=0;

        cvSaveImage(image_name_save,frame_raw);//原画质存储截图

        camera_Send_Capture();//信号发射
        image_counter+=1;
        bool_image_capture=false;
    }

    cvReleaseImage(&frame);//释放图像占用的内存
}

void Camera::closeCamara()
{
    timer->stop();         // 停止读取数据。
    cvReleaseCapture(&cam);//释放内存；
    bool_open_camera=false;

    //测试代码
    //cvDestroyWindow("TEST");
}

void Camera::takingPictures()
{
    //以编号、高度、日期、时间来命名截图
    current_time = QTime::currentTime();
    current_date_time=QDateTime::currentDateTime();
    QString qstr = current_date_time.toString("yyyy-MM-dd"); //设置显示格式
    string str=qstr.toStdString();
    char date[20];
    char date_symbol[6]="m-D-";
    strcpy(date,str.c_str());

    char format[5]=".jpg";
    char time[20];
    sprintf(time,"-T-%d-%d-%d-%d",current_time.hour(),current_time.minute(),current_time.second(),current_time.msec());
    char image_name_save_temp[100]="/";//图片存储路径+名称
    char image_name_temp[50]="P-";
    char image_name_print_temp[50]="P-";

    char rel_altitude[10];
    if(message.global_position.rel_altitude>=0)
         memcpy(rel_altitude,&message.global_position.rel_altitude,sizeof(rel_altitude));//将double放入char数组中

    char serial_number[20];

    strcpy(serial_number,serial_num.c_str());
    strcat(image_name_print_temp,serial_number);//图片上打印的字符
    strcat(image_name_print_temp,rel_altitude);
    strcat(image_name_print_temp,date_symbol);
    strcat(image_name_print_temp,date);
    strcat(image_name_print_temp,time);

    strcat(image_name_save_temp,path_image_save_partial);//保存名称
    strcat(image_name_save_temp,"/");
    strcat(image_name_save_temp,image_name_print_temp);
    strcat(image_name_save_temp,format);
    cout<<image_name_save_temp<<endl;

    strcat(image_name_temp,serial_number);//界面截图Icon显示名称
    strcat(image_name_temp,time);

    strcpy(image_name_save,image_name_save_temp);
    strcpy(image_name,image_name_temp);
    strcpy(image_name_print,image_name_print_temp);

    bool_image_capture=true;
}
