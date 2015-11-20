#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "receiver.h"
#include "camera.h"
#include "QDesktopWidget"
#include <QPainter>
#include <math.h>
#include <QBitmap>
#include <QPainter>
#include <QMessageBox>

extern MavrosMessage message;
extern Camera camera_video;

QLabel*velocity_h_label;//水平速度显示label
QLabel*velocity_z_label;//竖直速度显示label

int choice=1;
int angle=0;
int battery_status=-1;
int battery_status_last=0;
int radio_status=-1;
int radio_status_last=0;
int GPS_status=-1;
int GPS_status_last=0;

double orientation_last=0;

bool bool_flying=false;
int flying_time=0;
unsigned int flying_status_counter=0;
unsigned int flying_status_counter_last=0;

char serial_number[20];
string serial_num="No1-1-H";//初始序列号，用于保存截图的命名
QSize capture_show_size=QSize(192,108);//16:9
bool preview_enable=false;
unsigned int preview_serial_number=0;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setMouseTracking(true);//设置鼠标跟踪

   /*
    QDesktopWidget *d = QApplication::desktop();//屏幕大小捕获
    int w = d->width();     // 返回桌面宽度
    int h = d->height();    // 返回桌面高度
    setMaximumSize(w,h);
    setMinimumSize(w,h);
    */

    //定时器
    QTimer *timer=new QTimer(this);
    timer->start(1000);//每秒更新一次

    //信号与槽连接
    QObject::connect(timer,SIGNAL(timeout()),this,SLOT(time_Update()));
    QObject::connect(&message,SIGNAL(state_Mode_Signal()),this,SLOT(state_Mode_Slot()));
    QObject::connect(&message,SIGNAL(global_GPS_Signal()),this,SLOT(global_GPS_Slot()));
    QObject::connect(&message,SIGNAL(global_GPS_Satellites_Signal()),this,SLOT(global_GPS_Satellites_Slot()));
    QObject::connect(&message,SIGNAL(battery_Signal()),this,SLOT(battey_Slot()));
    QObject::connect(&message,SIGNAL(radio_Signal()),this,SLOT(radio_Status_Slot()));
    QObject::connect(&message,SIGNAL(global_Velocity_Signal()),this,SLOT(global_Velocity_Slot()));
    QObject::connect(&message,SIGNAL(global_Rel_Alt_Signal()),this,SLOT(global_Rel_Alt_Slot()));
    QObject::connect(&message,SIGNAL(local_Orientation_Signal()),this,SLOT(local_Position_Slot()));
    QObject::connect(&message,SIGNAL(optical_Flow_Signal()),this,SLOT(optical_Flow_Slot()));
    QObject::connect(&message,SIGNAL(temperature_Signal()),this,SLOT(temperature_Slot()));
    QObject::connect(&message,SIGNAL(time_Signal()),this,SLOT(time_Slot()));

    QObject::connect(&camera_video,SIGNAL(camera_Image_Signal()),this,SLOT(camera_Image_Slot()));
    QObject::connect(&camera_video,SIGNAL(camera_Capture_Show_Signal()),this,SLOT(camera_Capture_Show_Slot()));

    //pitch、roll、yaw绘图仪表初始化
    StatusPainter *painter = new StatusPainter();
    ui->tabWidget_PaintArea->addTab(painter,"仪表显示");//添加新的画图区widget new StatusPainter(QColor(141,238,238))
    //ui->tabWidget_PaintArea->removeTab(0);
    ui->tabWidget_PaintArea->setCurrentIndex(1);//显示第二页
    get_Painter_Address(painter);

    //水平、上升速度仪表设定
    ui->frame_Velocity_H->setFrameStyle(0);
    ui->frame_Velocity_H->setFixedSize(150,150);

    QPalette   palette;
    QPixmap pixmap(":/icon/Icons/3-150.png");//方向仪表背景图片
    palette.setBrush(ui->frame_Velocity_H-> backgroundRole(),QBrush(pixmap));
    ui->frame_Velocity_H->setPalette(palette);
    ui->frame_Velocity_H->setMask(pixmap.mask());  //可以将图片中透明部分显示为透明的
    ui->frame_Velocity_H->setAutoFillBackground(true);

    velocity_h_label = new QLabel(ui->frame_Velocity_H);
    velocity_h_label->setFixedWidth(150);
    velocity_h_label->setFixedHeight(150);
    velocity_h_label->move(0,0);

    ui->frame_Velocity_Z->setFrameStyle(0);
    ui->frame_Velocity_Z->setFixedSize(150,150);

    QPixmap pixmap5(":/icon/Icons/3-150-1.png");//方向仪表背景图片
    palette.setBrush(ui->frame_Velocity_Z-> backgroundRole(),QBrush(pixmap5));
    ui->frame_Velocity_Z->setPalette(palette);
    ui->frame_Velocity_Z->setMask(pixmap5.mask());  //可以将图片中透明部分显示为透明的
    ui->frame_Velocity_Z->setAutoFillBackground(true);

    velocity_z_label = new QLabel(ui->frame_Velocity_Z);
    velocity_z_label->setFixedWidth(150);
    velocity_z_label->setFixedHeight(150);
    velocity_z_label->move(0,0);

    //右下图标
    QPixmap pixmap2;
    if(choice==1)pixmap2=QPixmap(":/icon/Icons/fengji2_100_100.png");
    else if(choice==0)pixmap2=QPixmap(":/icon/Icons/bridge_100_100.png");
    palette.setBrush(ui->frame_Icon1-> backgroundRole(),QBrush(pixmap2));
    ui->frame_Icon1->setFrameStyle(0);
    ui->frame_Icon1->setPalette(palette);
    //ui->frame_Icon1->setMask(pixmap.mask());  //可以将图片中透明部分显示为透明的
    ui->frame_Icon1->setAutoFillBackground(true);
    //电池图标
    ui->frame_Battery->setFrameStyle(0);
    QPixmap pixmap3(":/icon/Icons/battery_60_40_1.png");
    palette.setBrush(ui->frame_Battery-> backgroundRole(),QBrush(pixmap3));
    ui->frame_Battery->setPalette(palette);
    //ui->frame_Battery->setMask(pixmap.mask());  //可以将图片中透明部分显示为透明的
    ui->frame_Battery->setAutoFillBackground(true);

    //label边框设定
    ui->label_GPS_Strength_0->setFrameStyle(1);
    ui->label_GPS_Strength_1->setFrameStyle(1);
    ui->label_GPS_Strength_2->setFrameStyle(1);
    ui->label_GPS_Strength_3->setFrameStyle(1);

    ui->label_Radio_Strength_0->setFrameStyle(1);
    ui->label_Radio_Strength_1->setFrameStyle(1);
    ui->label_Radio_Strength_2->setFrameStyle(1);
    ui->label_Radio_Strength_3->setFrameStyle(1);

    ui->label_Camera->setFrameStyle(0);

    //ui->label_Mode->setFrameStyle(2);

    //消除lineEdit的边框和背景色
   // ui->lineEdit_Battery->setStyleSheet("border :1px ;background : (0x00,0xff,0x00,0x00)");

    //字体字号颜色设定
    QFont font1("宋体",12,QFont::Bold);
    ui->label_Mode->setFont(font1);
    ui->lineEdit_Optical_Flow_Distance->setFont(font1);

    QPalette palette1;//红色
    palette1.setColor(QPalette::Text,QColor(255,0,0));
    ui->label_Mode->setPalette(palette1);
    ui->label_Tips->setPalette(palette1);
    ui->lineEdit_Optical_Flow_Distance->setPalette(palette1);
    ui->lineEdit_Optical_Flow_Quality->setPalette(palette1);

    QPalette palette2;//蓝色
    palette2.setColor(QPalette::Text,QColor(0,0,255));
    ui->lineEdit_GPS_X->setPalette(palette2);
    ui->lineEdit_GPS_Y->setPalette(palette2);
    ui->lineEdit_GPS_Z->setPalette(palette2);
    ui->lineEdit_Wind_Speed->setPalette(palette2);
    ui->lineEdit_Rel_Alt->setPalette(palette2);
    ui->lineEdit_GPS_Satellites->setPalette(palette2);
    ui->lineEdit_Battery->setPalette(palette2);

    //预览区域listwiget设定
    ui->listWidget->setIconSize(capture_show_size);
    ui->listWidget->setResizeMode(QListView::Adjust);
    ui->listWidget->setViewMode(QListView::IconMode);//显示模式
    ui->listWidget->setMovement(QListView::Static);//不可移动
    ui->listWidget->setSpacing(10);//间距设定
    ui->listWidget->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);//设定滑动条
    ui->listWidget->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);

    //button使能设定
    ui->pushButton_Open_Video->setEnabled(true);
    ui->pushButton_Close_Video->setEnabled(false);
    ui->pushButton_Capture_Video->setEnabled(false);
    ui->pushButton_Preview->setEnabled(false);
    ui->pushButton_Image_Recovery->setEnabled(false);

    //事件过滤器
    ui->label_Camera->installEventFilter(this);
    preview_scale_height=1.0;
    preview_scale_width=1.0;

    //其它初始值设定
    preview_current_num=2;
    ui->checkBox_Ruler->setChecked(true);//初始显示标尺

    //编号初值设定
    ui->comboBox_Num->setCurrentIndex(0);
    ui->spinBox_Num->setValue(1);
    if(choice==1)ui->label_Yepian->setText("叶片");

    //测试按钮。。
    ui->pushButton_Test->setEnabled(false);
}


MainWindow::~MainWindow()
{
    delete ui;
}
/****显示消息槽****/
void MainWindow::state_Mode_Slot()
{
    ui->label_Mode->setText(QString::fromStdString(message.mode));
}
void MainWindow::battey_Slot()
{
    ui->lineEdit_Battery->setText(QString::number(message.battery_voltage));

    if(message.battery_voltage>18.0)
    {
        if(message.battery_voltage>25)battery_status=4;
        else if(message.battery_voltage>23.0)battery_status=3;
        else if(message.battery_voltage>21.5)battery_status=2;
        else if(message.battery_voltage>20.0)battery_status=1;
        else if(message.battery_voltage>19.5)battery_status=0;
        else battery_status=-1;
    }
    else if(message.battery_voltage>9.0)
    {
        if(message.battery_voltage>12.5)battery_status=4;
        else if(message.battery_voltage>12.0)battery_status=3;
        else if(message.battery_voltage>11.5)battery_status=2;
        else if(message.battery_voltage>11.0)battery_status=1;
        else if(message.battery_voltage>10.5)battery_status=0;
        else battery_status=-1;
    }
    else battery_status=5;

    if(battery_status!=battery_status_last)
    {
        switch(battery_status)
        {
        case 4:
        {
            ui->label_Battery_Voltage_0->setStyleSheet("background-color:rgb(0,255,127)");
            ui->label_Battery_Voltage_1->setStyleSheet("background-color:rgb(0,255,127)");
            ui->label_Battery_Voltage_2->setStyleSheet("background-color:rgb(0,255,127)");
            ui->label_Battery_Voltage_3->setStyleSheet("background-color:rgb(0,255,127)");
            ui->label_Battery_Voltage_4->setStyleSheet("background-color:rgb(0,255,127)");
            battery_status_last=4;
            break;
        }
        case 3:
        {
           ui->label_Battery_Voltage_0->setStyleSheet("background-color:rgb(0,255,127)");
           ui->label_Battery_Voltage_1->setStyleSheet("background-color:rgb(0,255,127)");
           ui->label_Battery_Voltage_2->setStyleSheet("background-color:rgb(0,255,127)");
           ui->label_Battery_Voltage_3->setStyleSheet("background-color:rgb(0,255,127)");
           ui->label_Battery_Voltage_4->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
           battery_status_last=3;
           break;
        }
        case 2:
        {
            ui->label_Battery_Voltage_0->setStyleSheet("background-color:rgb(0,255,127)");
            ui->label_Battery_Voltage_1->setStyleSheet("background-color:rgb(0,255,127)");
            ui->label_Battery_Voltage_2->setStyleSheet("background-color:rgb(0,255,127)");
            ui->label_Battery_Voltage_3->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_Battery_Voltage_4->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            battery_status_last=2;
            break;
        }
        case 1:
        {
            ui->label_Battery_Voltage_0->setStyleSheet("background-color:yellow");
            ui->label_Battery_Voltage_1->setStyleSheet("background-color:yellow");
            ui->label_Battery_Voltage_2->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_Battery_Voltage_3->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_Battery_Voltage_4->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            battery_status_last=1;
            break;
        }
        case 0:
        {
            ui->label_Battery_Voltage_0->setStyleSheet("background-color:red");
            ui->label_Battery_Voltage_1->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_Battery_Voltage_2->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_Battery_Voltage_3->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_Battery_Voltage_4->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            battery_status_last=0;
            ui->label_Tips->setText(QString("Waining: Battery Low!"));
            break;
        }
        default:
        {
            ui->label_Battery_Voltage_0->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_Battery_Voltage_1->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_Battery_Voltage_2->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_Battery_Voltage_3->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_Battery_Voltage_4->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            battery_status_last=-1;
            break;
        }
        }
    }

}

void MainWindow::radio_Status_Slot()
{
    ui->lineEdit_Radio->setText(QString::number(message.radio_rssi));
    //画信号图
    if(message.radio_rssi>-40)radio_status=3;
    else if(message.radio_rssi>-60)radio_status=2;
    else if(message.radio_rssi>-90)radio_status=1;
    else if(message.radio_rssi>-120)radio_status=0;
    else radio_status=-1;
    if(radio_status!=radio_status_last)
    {
        switch(radio_status)
        {
        case 3:
        {
           ui->label_Radio_Strength_0->setStyleSheet("background-color:green");
           ui->label_Radio_Strength_1->setStyleSheet("background-color:green");
           ui->label_Radio_Strength_2->setStyleSheet("background-color:green");
           ui->label_Radio_Strength_3->setStyleSheet("background-color:green");
           radio_status_last=3;
           break;
        }
        case 2:
        {
            ui->label_Radio_Strength_0->setStyleSheet("background-color:green");
            ui->label_Radio_Strength_1->setStyleSheet("background-color:green");
            ui->label_Radio_Strength_2->setStyleSheet("background-color:green");
            ui->label_Radio_Strength_3->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            radio_status_last=2;
            break;
        }
        case 1:
        {
            ui->label_Radio_Strength_0->setStyleSheet("background-color:yellow");
            ui->label_Radio_Strength_1->setStyleSheet("background-color:yellow");
            ui->label_Radio_Strength_2->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_Radio_Strength_3->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            radio_status_last=1;
            break;
        }
        case 0:
        {
            ui->label_Radio_Strength_0->setStyleSheet("background-color:red");
            ui->label_Radio_Strength_1->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_Radio_Strength_2->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_Radio_Strength_3->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            radio_status_last=0;
            ui->label_Tips->setText(QString("Warning: Radio Signal Strength Weak!"));
            break;
        }
        default:
        {
            ui->label_Radio_Strength_0->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_Radio_Strength_1->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_Radio_Strength_2->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_Radio_Strength_3->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            radio_status_last=-1;
            break;
        }
        }
    }
}
void MainWindow::global_GPS_Slot()
{
    ui->lineEdit_GPS_X->setText(QString::number(message.global_position.gps.x));
    ui->lineEdit_GPS_Y->setText(QString::number(message.global_position.gps.y));
    ui->lineEdit_GPS_Z->setText(QString::number(message.global_position.gps.z));
}
void MainWindow::global_Velocity_Slot()
{
   //新版mavros（master分支下），global_Velocity的topic名称已经更换，这里暂时没用

    /* ui->lineEdit_Velocity_X->setText(QString::number(message.global_position.vel.x));
    ui->lineEdit_Velocity_Y->setText(QString::number(message.global_position.vel.y));
    ui->lineEdit_Velocity_Z->setText(QString::number(message.global_position.vel.z));

    QPainter painter;
    QImage image(":/icon/Icons/3-150.png");//定义图片，并在图片上绘图方便显示
    painter.begin(&image);

    painter.translate(75,75);

    double vel_z=message.global_position.vel.z;
    if(vel_z>1.0)vel_z=1.0;//最大+-1m/s
    else if(vel_z<-1.0)vel_z=-1.0;
    painter.rotate(vel_z*120-180);

    painter.setPen(QPen(Qt::red,3));
    painter.drawLine(0.0,5.0,0.0,50.0);*/

    /*painter.setPen(QPen(Qt::white,1));
    painter.setBrush(QBrush(Qt::white));
    painter.drawEllipse(-5.0,-5.0,10.0,10.0);*/

   /* painter.end();
    velocity_z_label->setPixmap(QPixmap::fromImage(image));//在label上显示图片

    if(fabs(message.global_position.vel.z)>0.05&&(!bool_flying))//判断是否起飞
        bool_flying=true;
    flying_status_counter+=1;
    if(flying_status_counter==65535)flying_status_counter=0;*/
}

void MainWindow::global_Rel_Alt_Slot()
{
    ui->lineEdit_Rel_Alt->setText(QString::number(message.global_position.rel_altitude));
}



void MainWindow::global_GPS_Satellites_Slot()
{
    ui->lineEdit_GPS_Satellites->setText(QString::number(message.global_position.gps.satellites));

    //画信号图
    if(message.global_position.gps.satellites>10)GPS_status=3;
    else if(message.global_position.gps.satellites>5)GPS_status=2;
    else if(message.global_position.gps.satellites>2)GPS_status=1;
    else if(message.global_position.gps.satellites>0)GPS_status=0;
    else GPS_status=-1;
    if(GPS_status!=GPS_status_last)
    {
        switch(GPS_status)
        {
        case 3:
        {
           ui->label_GPS_Strength_0->setStyleSheet("background-color:green");
           ui->label_GPS_Strength_1->setStyleSheet("background-color:green");
           ui->label_GPS_Strength_2->setStyleSheet("background-color:green");
           ui->label_GPS_Strength_3->setStyleSheet("background-color:green");
           GPS_status_last=3;
           break;
        }
        case 2:
        {
            ui->label_GPS_Strength_0->setStyleSheet("background-color:green");
            ui->label_GPS_Strength_1->setStyleSheet("background-color:green");
            ui->label_GPS_Strength_2->setStyleSheet("background-color:green");
            ui->label_GPS_Strength_3->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            GPS_status_last=2;
            break;
        }
        case 1:
        {
            ui->label_GPS_Strength_0->setStyleSheet("background-color:yellow");
            ui->label_GPS_Strength_1->setStyleSheet("background-color:yellow");
            ui->label_GPS_Strength_2->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_GPS_Strength_3->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            GPS_status_last=1;
            break;
        }
        case 0:
        {
            ui->label_GPS_Strength_0->setStyleSheet("background-color:red");
            ui->label_GPS_Strength_1->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_GPS_Strength_2->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_GPS_Strength_3->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            GPS_status_last=0;
            ui->label_Tips->setText(QString("Warning:GPS Signal Strength Weak!"));
            break;
        }
        default:
        {
            ui->label_GPS_Strength_0->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_GPS_Strength_1->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_GPS_Strength_2->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            ui->label_GPS_Strength_3->setStyleSheet("background:(0x00,0xff,0x00,0x00)");
            GPS_status_last=-1;
            break;
        }
        }
    }
}

void MainWindow::local_Position_Slot()
{
    //去抖动
    if(fabs(message.local_position.orientation.pitchd+message.local_position.orientation.rolld+message.local_position.orientation.yawd-orientation_last)>=2.0)
    {
        status_painter->pitchd=message.local_position.orientation.pitchd;
        status_painter->rolld=message.local_position.orientation.rolld;
        status_painter->pitch=message.local_position.orientation.pitch;
        status_painter->roll=message.local_position.orientation.roll;
        status_painter->compassd=-message.local_position.orientation.yawd+180;
        status_painter->update();
    }

    ui->lineEdit_Orientation_Pitchd->setText(QString::number(message.local_position.orientation.pitchd));
    ui->lineEdit_Orientation_Rolld->setText(QString::number(message.local_position.orientation.rolld));
    ui->lineEdit_Orientation_Yawd->setText(QString::number(message.local_position.orientation.yawd));

    //竖直速度显示，这里的local_position.position消息为速度
    ui->lineEdit_Velocity_X->setText(QString::number(message.local_position.position.x));
    ui->lineEdit_Velocity_Y->setText(QString::number(message.local_position.position.y));
    ui->lineEdit_Velocity_Z->setText(QString::number(message.local_position.position.z));

    QPainter painter;
    QImage image(":/icon/Icons/3-150-1.png");//定义图片，并在图片上绘图方便显示
    painter.begin(&image);

    painter.translate(75,75);

    double vel_z=message.local_position.position.z;
    if(vel_z>1.0)vel_z=1.0;//最大+-1m/s
    else if(vel_z<-1.0)vel_z=-1.0;
    painter.rotate(vel_z*120-180);

    painter.setPen(QPen(Qt::red,3));
    painter.drawLine(0.0,5.0,0.0,50.0);

    painter.end();
    velocity_z_label->setPixmap(QPixmap::fromImage(image));//在label上显示图片

    //判断是否起飞
    if(fabs(message.local_position.position.z)>0.1&&(!bool_flying))
        bool_flying=true;
    flying_status_counter+=1;
    if(flying_status_counter==65535)flying_status_counter=0;

    //水平速度显示
    QPainter painter1;
    QImage image1(":/icon/Icons/3-150.png");//定义图片，并在图片上绘图方便显示
    painter1.begin(&image1);

    painter1.translate(75,75);

    double vel_h=sqrt(message.local_position.position.x*message.local_position.position.x+message.local_position.position.y*message.local_position.position.y);
    if(vel_h>5.0)vel_h=5.0;//最大5m/s

    painter1.rotate(vel_h*48+60);

    painter1.setPen(QPen(Qt::red,3));
    painter1.drawLine(0.0,5.0,0.0,50.0);

    painter1.end();
    velocity_h_label->setPixmap(QPixmap::fromImage(image1));//在label上显示图片
}

void MainWindow::optical_Flow_Slot()
{
    ui->lineEdit_Optical_Flow_Distance->setText(QString::number(message.optical_flow.distance));
    ui->lineEdit_Optical_Flow_Quality->setText(QString::number(message.optical_flow.quality));
}

void MainWindow::temperature_Slot()
{
    ui->lineEdit_Temperature->setText(QString::number(message.temperature));
}

void MainWindow::time_Slot()
{
;
}

/*****主界面绘图槽******/
void MainWindow::paintEvent(QPaintEvent *event)
{
    //QPainter mainwindow_painter(this);
    //mainwindow_painter.drawLine(QPoint(0,0),QPoint(100,100));
}

/*********视频显示槽***********/
void MainWindow::camera_Image_Slot()
{
    ui->label_Camera->setPixmap(QPixmap::fromImage(camera_video.image));//视频
}

void MainWindow::camera_Capture_Show_Slot()
{
    //将截图显示在listwiget上
    QPixmap pixmap_temp=QPixmap::fromImage(QImage(camera_video.image_temp[camera_video.image_counter]));//QImage->QPixmap
    QListWidgetItem *pItem = new QListWidgetItem((pixmap_temp.scaled(capture_show_size)),camera_video.image_name);
    pItem->setSizeHint(QSize(capture_show_size.width(),capture_show_size.height()+20));
    ui->listWidget->insertItem(camera_video.image_counter,pItem);//添加Item
    ui->listWidget->setCurrentItem(pItem);
    ui->pushButton_Preview->setEnabled(true);
}

void MainWindow::on_listWidget_itemDoubleClicked(QListWidgetItem *item)
{
  camera_video.bool_show_Image=false;

  ui->label_Camera->move(0,0);//恢复label_Camera位置大小
  ui->label_Camera->setFixedSize(image_area_width,image_area_height);

  preview_current_num = item->icon().serialNumber();
  image_resize = camera_video.image_temp[preview_current_num-2]
          .scaled(camera_video.image_temp[preview_current_num-2].width(),camera_video.image_temp[preview_current_num-2].height())
          .scaled(image_area_width,image_area_height, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
  ui->label_Camera->setPixmap(QPixmap::fromImage(image_resize));
  preview_enable=true;
  ui->pushButton_Preview->setText("视频显示");
  ui->pushButton_Image_Recovery->setEnabled(true);

  ui->label_Camera->setCursor(QCursor(Qt::OpenHandCursor));//设定鼠标形状为手形
}

void MainWindow::on_listWidget_itemClicked(QListWidgetItem *item)//选到点击的图片上
{
    preview_serial_number=item->icon().serialNumber()-2;
}

void MainWindow::on_listWidget_itemChanged(QListWidgetItem *item)//选到新的截图上
{
    preview_serial_number=item->icon().serialNumber()-2;
}

void MainWindow::on_pushButton_Open_Video_clicked()
{
    camera_video.openCamara();
    ui->pushButton_Open_Video->setEnabled(false);
    ui->pushButton_Close_Video->setEnabled(true);
    ui->pushButton_Capture_Video->setEnabled(true);
}


void MainWindow::on_pushButton_Close_Video_clicked()
{
    camera_video.closeCamara();
    ui->pushButton_Open_Video->setEnabled(true);
    ui->pushButton_Close_Video->setEnabled(false);
    ui->pushButton_Capture_Video->setEnabled(false);
}


void MainWindow::on_pushButton_Capture_Video_clicked()
{
    camera_video.takingPictures();
}

void MainWindow::on_pushButton_Preview_clicked()
{
    if(preview_enable)//切换预览与视频显示
    {
        if(camera_video.bool_open_camera)
        {
            ui->pushButton_Preview->setText("截图预览");
            preview_enable=false;//预览位置0

            ui->label_Camera->move(0,0);//恢复label_Camera位置大小
            ui->label_Camera->setFixedSize(image_area_width,image_area_height);

            camera_video.bool_show_Image=true;//显示视频
            ui->pushButton_Image_Recovery->setEnabled(false);

            ui->label_Camera->setCursor(QCursor(Qt::ArrowCursor));//鼠标转换为箭头形状
        }
        else
        {
            QMessageBox::warning(this,"提示：","请先打开视频，再切换视频显示");
        }
    }
    else
    {
        camera_video.bool_show_Image=false;//关掉视频显示

        image_resize = camera_video.image_temp[preview_current_num-2]
                .scaled(camera_video.image_temp[preview_current_num-2].width(),camera_video.image_temp[preview_current_num-2].height())
                .scaled(image_area_width,image_area_height, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
        ui->label_Camera->setPixmap(QPixmap::fromImage(image_resize));
        preview_enable=true;
        ui->pushButton_Preview->setText("视频显示");
        ui->pushButton_Image_Recovery->setEnabled(true);

        ui->label_Camera->setCursor(QCursor(Qt::OpenHandCursor));//设定鼠标形状为手形
    }
}

void MainWindow::on_spinBox_Num_editingFinished()//编号写入
{
    sprintf(serial_number,"No%d-%d-H-",ui->spinBox_Num->value(),(ui->comboBox_Num->currentIndex()+1));
    serial_num= serial_number;
}

void MainWindow::on_comboBox_Num_currentIndexChanged(int index)//编号写入
{
    sprintf(serial_number,"No%d-%d-H-",ui->spinBox_Num->value(),(ui->comboBox_Num->currentIndex()+1));
    serial_num= serial_number;
}

void MainWindow::on_pushButton_Image_Recovery_clicked()
{
    ui->label_Camera->move(0,0);//恢复label_Camera位置大小
    ui->label_Camera->setFixedSize(image_area_width,image_area_height);

    //重新载入图片
    preview_scale_width=1.0;
    preview_scale_height=1.0;
    image_resize = camera_video.image_temp[preview_current_num-2]
            .scaled(camera_video.image_temp[preview_current_num-2].width(),camera_video.image_temp[preview_current_num-2].height())
            .scaled(image_area_width*preview_scale_width,image_area_height*preview_scale_height, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    ui->label_Camera->setPixmap(QPixmap::fromImage(image_resize));
}

void MainWindow::on_checkBox_Ruler_clicked()//设定标尺显示
{
    if(camera_video.bool_show_ruler)camera_video.bool_show_ruler = false;
    else camera_video.bool_show_ruler = true;
}


/***********鼠标事件***********/
bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if(preview_enable)//截图显示时
    {
        if(obj==ui->label_Camera)//鼠标在图像显示区域时
        {
            if(event->type()==QEvent::Wheel)//滚轮放大图片，最大5倍
            {
                QWheelEvent *wheelEvent = static_cast<QWheelEvent*>(event);
                if(wheelEvent->delta()>0&&preview_scale_width<=5.0)
                {
                    preview_scale_width += 0.2;
                    preview_scale_height += 0.2;
                    //放大图片
                    image_resize = camera_video.image_temp[preview_current_num-2]
                            .scaled(camera_video.image_temp[preview_current_num-2].width(),camera_video.image_temp[preview_current_num-2].height())
                            .scaled(image_area_width*preview_scale_width,image_area_height*preview_scale_height, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
                    ui->label_Camera->setFixedSize(image_area_width*preview_scale_width,image_area_height*preview_scale_height);
                    ui->label_Camera->setPixmap(QPixmap::fromImage(image_resize));
                    return true;
                }
                else if(wheelEvent->delta()<0&&preview_scale_width>=1.0)//滚轮缩小图片，限制最小
                {
                    preview_scale_width -= 0.2;
                    preview_scale_height -= 0.2;
                    image_resize = camera_video.image_temp[preview_current_num-2]
                            .scaled(camera_video.image_temp[preview_current_num-2].width(),camera_video.image_temp[preview_current_num-2].height())
                            .scaled(image_area_width*preview_scale_width,image_area_height*preview_scale_height, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
                    ui->label_Camera->setFixedSize(image_area_width*preview_scale_width,image_area_height*preview_scale_height);
                    ui->label_Camera->setPixmap(QPixmap::fromImage(image_resize));
                    return true;
                }
                else return false;
            }

            else if(event->type()==QEvent::MouseButtonPress)//鼠标按键按下
            {
                QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
                if(mouseEvent->button()==Qt::LeftButton)//左键按下
                {
                    mouse_pos = mouseEvent->globalPos()-ui->label_Camera->pos();//计算相对位置
                    return true;
                }
            }

            else if(event->type()==QEvent::MouseMove)//鼠标移动时
            {
                QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
                if(mouseEvent->buttons()&Qt::LeftButton)//左键按下并移动时移动图片
                {
                    QPoint temp_pos;
                    temp_pos=mouseEvent->globalPos()-mouse_pos;
                    ui->label_Camera->move(temp_pos);
                    return true;
                }
            }
        }
    }

    if(preview_enable==false&&event->type()==QEvent::MouseButtonDblClick)//视频显示双击时截图
    {
       on_pushButton_Capture_Video_clicked();//使双击相当于点击截图按钮
       return true;
    }
    else
    {
        return QMainWindow::eventFilter(obj, event);
    }
}

/********其它功能槽*******/
void MainWindow::get_Painter_Address(StatusPainter *painter)
{
    status_painter=painter;
}

void MainWindow::time_Update()
{
    system_time=QTime::currentTime();
    char time_temp[20];
    sprintf(time_temp,"%d:%d:%d",system_time.hour(),system_time.minute(),system_time.second());
    ui->label_System_Time->setText(time_temp);

    if(flying_status_counter_last!=flying_status_counter)//判断是否还在飞行
        flying_status_counter_last=flying_status_counter;
    else bool_flying=false;

    if(bool_flying)//飞行时间显示
    {
        flying_time+=1;
        int minutes=flying_time/60;
        int seconds=flying_time%60;
        char flying_time_temp[20];
        sprintf(flying_time_temp,"%d:%d",minutes,seconds);
        ui->label_Flying_Time->setText(flying_time_temp);
    }
}

void MainWindow::on_pushButton_Reset_FlyingTime_clicked()
{
    flying_time=0;
    bool_flying=false;
}

/**********试验用槽**********/
void MainWindow::on_pushButton_clicked()
{

    QPainter painter;

    QImage image(":/icon/Icons/1-150.png");//定义图片，并在图片上绘图方便显示
    painter.begin(&image);

    painter.translate(75,75);
    painter.rotate(angle);

    painter.setPen(QPen(Qt::white,1));
    painter.setBrush(QBrush(Qt::white));
    QPointF points[4];
    points[0]=QPointF(0.0,-20.0);
    points[1]=QPointF(20.0,-40.0);
    points[2]=QPointF(0.0,40.0);
    points[3]=QPointF(-20.0,-40.0);
    painter.drawPolygon(points,4);

    painter.setPen(QPen(Qt::red,1));
    painter.setBrush(QBrush(Qt::red));
    QPointF points2[3];
    points2[0]=QPointF(0.0,40.0);
    points2[1]=QPointF(5.0,20.0);
    points2[2]=QPointF(-5.0,20.0);
    painter.drawPolygon(points2,3);

    painter.end();
    velocity_h_label->setPixmap(QPixmap::fromImage(image));//在label上显示图片

    angle+=10;
}



