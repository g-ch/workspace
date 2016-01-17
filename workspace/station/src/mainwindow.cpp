//********mianwindow.cpp*********

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "receiver.h"
#include "QDesktopWidget"
#include <QPainter>
#include <math.h>
#include <QBitmap>
#include <QPainter>
#include <QMessageBox>

#define FLY_POSITION_LABEL_WIDTH 720
#define FLY_POSITION_LABEL_HEIGHT 540
#define FLY_ROUTE_LABEL_WIDTH 720
#define FLY_ROUTE_LABEL_HEIGHT 540

extern MavrosMessage message;

QLabel*fly_position_label;
QLabel*fly_route_label;

int choice=0;//选择显示的图片
int angle=0;
int success_counter = SUCCESS_COUNTER_INIT;

int controller_flag=0;
int computer_flag=0;
int controller_flag_last=0;
int computer_flag_last=0;

double orientation_last=0;

bool bool_flying=false;
bool send_button_pressed = false;

int flying_time=0;
unsigned int flying_status_counter=0;
unsigned int flying_status_counter_last=0;

//以下变量用于画路径图
float field_size_length = 0.0;
float field_size_height = 5.0;
float field_size_width = 0.0;
int field_size_times = 0;
float paint_scale = 1.0;
float real_position[3600][2];
int position_num = 0;
int save_counter = 0;

float fly_distance = 0;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

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
    QObject::connect(&message,SIGNAL(offboard_Set_Signal()),this,SLOT(offboard_Set_Slot()));
    QObject::connect(&message,SIGNAL(field_Size_Confirm_Signal()),this,SLOT(field_Size_Confirm_Slot()));
    QObject::connect(&message,SIGNAL(pump_Status_Signal()),this,SLOT(pump_Status_Slot()));

    //pitch、roll、yaw绘图仪表初始化
    StatusPainter *painter = new StatusPainter();
    ui->tabWidget_PaintArea->addTab(painter,"仪表显示");//添加新的画图区widget new StatusPainter(QColor(141,238,238))
    //if(choice==0)ui->tabWidget_PaintArea->removeTab(0);
    ui->tabWidget_PaintArea->setCurrentIndex(1);//显示第二页
    get_Painter_Address(painter);


    //消除lineEdit的边框和背景色
   // ui->lineEdit_Battery->setStyleSheet("border :1px ;background : (0x00,0xff,0x00,0x00)");

    //字体字号颜色设定
    QFont font1("宋体",12,QFont::Bold);
    ui->label_Mode->setFont(font1);


    QPalette palette1;//红色
    palette1.setColor(QPalette::Text,QColor(255,0,0));
    ui->label_Mode->setPalette(palette1);
    ui->label_Tips->setPalette(palette1);

    QPalette palette2;//蓝色
    palette2.setColor(QPalette::Text,QColor(0,0,255));
    ui->lineEdit_GPS_X->setPalette(palette2);
    ui->lineEdit_GPS_Y->setPalette(palette2);
    ui->lineEdit_GPS_Z->setPalette(palette2);
    ui->lineEdit_Rel_Alt->setPalette(palette2);
    ui->lineEdit_GPS_Satellites->setPalette(palette2);
    ui->lineEdit_Battery->setPalette(palette2);

    //lineedit写入限制
    ui->lineEdit_Offboard_Height->setValidator(new QDoubleValidator(0.0, 10.0, 2, this));
    ui->lineEdit_Offboard_Length->setValidator(new QDoubleValidator(0.0, 1000.0, 2, this));
    ui->lineEdit_Offboard_Speed->setValidator(new QDoubleValidator(0.0, 20.0, 2, this));
    ui->lineEdit_Offboard_Width->setValidator(new QDoubleValidator(0.0, 1000.0, 2, this));
    ui->lineEdit_Offboard_Times->setValidator(new QIntValidator(0,1000,this));


    //飞行路径图背景设置
    ui->frame_Fly_Route->setFrameStyle(1);
    ui->frame_Fly_Route->setFixedSize(720,540);

    QPalette   palette3;
    QPixmap pixmap3(":/icon/Icons/grass-720x540-2.png");//背景图片
    palette3.setBrush(ui->frame_Fly_Route-> backgroundRole(),QBrush(pixmap3));
    ui->frame_Fly_Route->setPalette(palette3);
    ui->frame_Fly_Route->setMask(pixmap3.mask());  //可以将图片中透明部分显示为透明的
    ui->frame_Fly_Route->setAutoFillBackground(true);

    fly_route_label = new QLabel(ui->frame_Fly_Route);
    fly_route_label->setFixedWidth(720);
    fly_route_label->setFixedHeight(540);
    fly_route_label->move(0,0);


    ui->frame_Fly_Position->setFrameStyle(1);
    ui->frame_Fly_Position->setFixedSize(720,540);

    QPalette   palette4;
    QPixmap pixmap4(":/icon/Icons/grass-720x540-2.png");//背景图片
    palette4.setBrush(ui->frame_Fly_Position-> backgroundRole(),QBrush(pixmap4));
    ui->frame_Fly_Position->setPalette(palette4);
    ui->frame_Fly_Position->setMask(pixmap4.mask());  //可以将图片中透明部分显示为透明的
    ui->frame_Fly_Position->setAutoFillBackground(true);

    fly_position_label = new QLabel(ui->frame_Fly_Position);
    fly_position_label->setFixedWidth(720);
    fly_position_label->setFixedHeight(540);
    fly_position_label->move(0,0);

    //设置是否可用
    ui->pushButton_Route_Generate->setEnabled(true);
    ui->pushButton_Route_Reset->setEnabled(false);
    ui->pushButton_Route_Send->setEnabled(false);

    ui->lineEdit_Offboard_Speed->setEnabled(false);

    ui->progressBar_GPS->setRange(0,15);
    ui->progressBar_Battery->setRange(19,25);
    ui->progressBar_RC->setRange(0,200);

    //set conection display
    ui->label_Controller->setStyleSheet("background-color:red");
    ui->label_Computer->setStyleSheet("background-color:red");
}


MainWindow::~MainWindow()
{
    delete ui;
}
/****显示消息槽****/
void MainWindow::state_Mode_Slot()
{
    ui->label_Mode->setText(QString::fromStdString(message.mode));
    if(controller_flag!=100)controller_flag+=1;
    else controller_flag=1;
}

void MainWindow::battey_Slot()
{
    ui->lineEdit_Battery->setText(QString::number(message.battery_voltage));
    ui->progressBar_Battery->setValue((message.battery_voltage>25)?25:message.battery_voltage);
}

void MainWindow::radio_Status_Slot()
{
    ui->lineEdit_Radio->setText(QString::number(message.radio_rssi));
    ui->progressBar_RC->setValue(((message.radio_rssi+240)>200)?200:(message.radio_rssi+240));
}

void MainWindow::global_GPS_Slot()
{
    ui->lineEdit_GPS_X->setText(QString::number(message.global_position.gps.x));
    ui->lineEdit_GPS_Y->setText(QString::number(message.global_position.gps.y));
    ui->lineEdit_GPS_Z->setText(QString::number(message.global_position.gps.z));
}
void MainWindow::global_Velocity_Slot()
{

    ui->lineEdit_Velocity_X->setText(QString::number(message.global_position.vel.x));
    ui->lineEdit_Velocity_Y->setText(QString::number(message.global_position.vel.y));
    ui->lineEdit_Velocity_Z->setText(QString::number(message.global_position.vel.z));

}

void MainWindow::global_Rel_Alt_Slot()
{
    ui->lineEdit_Rel_Alt->setText(QString::number(message.global_position.rel_altitude));
}

void MainWindow::global_GPS_Satellites_Slot()
{
    ui->lineEdit_GPS_Satellites->setText(QString::number(message.global_position.gps.satellites));
    ui->progressBar_GPS->setValue(((message.global_position.gps.satellites>15)?15:message.global_position.gps.satellites));
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

    //竖直速度显示，这里的local_position.消息为速度
    ui->lineEdit_Velocity_X->setText(QString::number(message.local_position.speed.x));
    ui->lineEdit_Velocity_Y->setText(QString::number(message.local_position.speed.y));
    ui->lineEdit_Velocity_Z->setText(QString::number(message.local_position.speed.z));
    ui->lineEdit_Velocity->setText(QString::number(sqrt(message.local_position.speed.x*message.local_position.speed.x+message.local_position.speed.y*message.local_position.speed.y)));

    //判断是否起飞
    if(fabs(message.local_position.position.z)>=1.0)
        bool_flying=true;
    if(fabs(message.local_position.position.z)<1.0)
    {
        flying_status_counter+=1;
        if(flying_status_counter>200)
        {
            flying_status_counter=0;
            bool_flying = false;
        }
    }

    //local_position画当前位置图
    save_counter++;
    if(message.mode=="自动喷洒" && save_counter%10==1)
    {
         float yaw = message.local_position.orientation.yaw + 3.1416;
         real_position[position_num][0]=message.local_position.position.x*cos(yaw) + message.local_position.position.y*sin(yaw);
         real_position[position_num][1]=message.local_position.position.y*cos(yaw) - message.local_position.position.x*sin(yaw);

         //画出飞行图
         QPainter painter;
         QImage image(":/icon/Icons/grass-720x540-2.png");//定义图片，并在图片上绘图方便显示
         painter.begin(&image);

         //画设定路径
         painter.translate(60,60);
         painter.setPen(QPen(Qt::yellow,3));

         int line_height, line_width;
         if(paint_scale > 0)
         {
             line_height = (int)(message.field_size.length/paint_scale);
             line_width = (int)(message.field_size.width/paint_scale);
         }
         else { line_height=0;  line_width=0;}

         for(int i=0; i < message.field_size.times; i++) //竖线
         {
             painter.drawLine(i*line_width, 0, i*line_width, line_height);
         }
         for(int j=0; j < message.field_size.times-1; j++)//横线
         {
             if(j%2==0)painter.drawLine(j*line_width, 0, (j+1)*line_width, 0);
             else painter.drawLine(j*line_width, line_height, (j+1)*line_width, line_height);
         }

         //画实际位置
         painter.setPen(QPen(Qt::red,2));
         painter.translate(0,line_height);

         if(position_num == 0) painter.drawLine(0.0,0.0,2.0,2.0);
         else
         {
             for(int n=0;n<position_num;n++)
             {
                  painter.drawLine((real_position[n][0]-real_position[0][0])/paint_scale,-(real_position[n][1]-real_position[0][1])/paint_scale,
                       (real_position[n+1][0]-real_position[0][0])/paint_scale,-(real_position[n+1][1]-real_position[0][1])/paint_scale);
             }
         }

         painter.end();
         fly_position_label->setPixmap(QPixmap::fromImage(image));//在label上显示图片

         //计数器设置
         save_counter = 0;
         position_num += 1;

         if(position_num>0)
         {
             fly_distance += sqrt((real_position[position_num+1][0]-real_position[position_num][0])*(real_position[position_num+1][0]-real_position[position_num][0])
                 +(real_position[position_num+1][1]-real_position[position_num][1])*(real_position[position_num+1][1]-real_position[position_num][1]));
             ui->lineEdit_Total_Distance->setText(QString::number(fly_distance));
         }
    }
    if(message.mode!="自动喷洒")
    {
        position_num = 0; //退出offboard则清零等待重新计数
        fly_distance = 0; //距离清零
    }

}

void MainWindow::optical_Flow_Slot()
{
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

    int minutes;
    int seconds;
    if(bool_flying)//飞行时间显示
    {
        flying_time+=1;
        minutes=flying_time/60;
        seconds=flying_time%60;
        char flying_time_temp[20];
        sprintf(flying_time_temp,"%d:%d",minutes,seconds);
        ui->label_Flying_Time->setText(flying_time_temp);
    }
    if(seconds%3==0)timer_Slot();
}

void MainWindow::timer_Slot()
{
    if(controller_flag_last!=controller_flag) ui->label_Controller->setStyleSheet("background-color:green");
    else ui->label_Controller->setStyleSheet("background-color:red");

    if(computer_flag_last!=computer_flag) ui->label_Computer->setStyleSheet("background-color:green");
    else ui->label_Computer->setStyleSheet("background-color:red");

    controller_flag_last=controller_flag;
    computer_flag_last=computer_flag;
}
void MainWindow::on_pushButton_Reset_FlyingTime_clicked()
{
    flying_time=0;
    bool_flying=false;
}


void MainWindow::on_pushButton_Route_Generate_clicked()
{
    //读取
    QString length = ui->lineEdit_Offboard_Length->text();
    QString height = ui->lineEdit_Offboard_Height->text();
    QString width = ui->lineEdit_Offboard_Width->text();
    QString times = ui->lineEdit_Offboard_Times->text();
    //判断是否所有值都写好
    if(length.isEmpty()||height.isEmpty()||width.isEmpty()||times.isEmpty())
    {
        QMessageBox message_box(QMessageBox::Warning,"警告","参数编辑不全", QMessageBox::Cancel, NULL);
        message_box.exec();
    }
    else
    {
        field_size_length = length.toFloat();
        field_size_height = height.toFloat();
        field_size_width = width.toFloat();
        field_size_times = times.toInt();
        //画路径图
        QPainter painter;
        QImage image(":/icon/Icons/grass-720x540-2.png");//定义图片，并在图片上绘图方便显示
        painter.begin(&image);

        painter.translate(60,60);
        painter.setPen(QPen(Qt::yellow,2));

        float scale1 =field_size_length/(FLY_ROUTE_LABEL_HEIGHT-120);//比例尺
        float scale2 = field_size_width*(field_size_times-1)/(FLY_ROUTE_LABEL_WIDTH-120);
        float scale = (scale1>scale2)?scale1:scale2;

        int line_height, line_width;
        if(scale>0)
        {
            line_height = (int)(field_size_length/scale);
            line_width = (int)(field_size_width/scale);
        }
        else { line_height=0;  line_width=0;}

        for(int i=0; i < field_size_times; i++)//竖线
        {
            painter.drawLine(i*line_width, 0, i*line_width, line_height);
        }
        for(int i=0; i < field_size_times-1; i++)//横线
        {
            if(i%2==0)painter.drawLine(i*line_width, 0, (i+1)*line_width, 0);
            else painter.drawLine(i*line_width, line_height, (i+1)*line_width, line_height);
        }

        painter.end();
        fly_route_label->setPixmap(QPixmap::fromImage(image));//在label上显示图片

        ui->textBrowser_Offboard_Message->append("生成成功！");
        ui->pushButton_Route_Send->setEnabled(true);
    }
}


void MainWindow::on_pushButton_Route_Send_clicked()
{
    message.field_size.length = field_size_length;
    message.field_size.height = field_size_height;
    message.field_size.width = field_size_width;
    message.field_size.times = field_size_times;

    QPainter painter;
    QImage image(":/icon/Icons/grass-720x540-2.png");//定义图片，并在图片上绘图方便显示
    painter.begin(&image);

    painter.translate(60,60);
    painter.setPen(QPen(Qt::yellow,2));

    float scale1 =message.field_size.length/(FLY_ROUTE_LABEL_HEIGHT-120);//比例
    float scale2 = message.field_size.width*(message.field_size.times-1)/(FLY_ROUTE_LABEL_WIDTH-120);
    float scale = (scale1>scale2)?scale1:scale2;
    paint_scale = scale;

    int line_height, line_width;
    if(scale>0)
    {
        line_height = (int)(message.field_size.length/scale);
        line_width = (int)(message.field_size.width/scale);
    }
    else { line_height=0;  line_width=0;}

    for(int i=0; i < message.field_size.times; i++) //竖线
    {
        painter.drawLine(i*line_width, 0, i*line_width, line_height);
    }
    for(int i=0; i < message.field_size.times-1; i++)//横线
    {
        if(i%2==0)painter.drawLine(i*line_width, 0, (i+1)*line_width, 0);
        else painter.drawLine(i*line_width, line_height, (i+1)*line_width, line_height);
    }

    painter.end();
    fly_position_label->setPixmap(QPixmap::fromImage(image));//在label上显示图片


    send_button_pressed = true;
    ui->textBrowser_Offboard_Message->append("发送中...");

}

void MainWindow::offboard_Set_Slot()
{
    if(success_counter < message.success_counter)
    {
        ui->textBrowser_Offboard_Message->append("发送成功!");
        success_counter = message.success_counter;
        ui->pushButton_Route_Send->setEnabled(false);
        ui->pushButton_Route_Reset->setEnabled(true);
        position_num = 0;//重新开始画实际位置
    }
    else
    {
        ui->textBrowser_Offboard_Message->append("发送失败，请重试!");
        success_counter = message.success_counter;
        ui->pushButton_Route_Send->setEnabled(true);
        ui->pushButton_Route_Reset->setEnabled(false);
    }
}


void MainWindow::on_pushButton_Route_Reset_clicked()//重置，使飞机原设定高度悬停
{
    field_size_length = 0;
    field_size_width = 0;
    field_size_times = 0;
    on_pushButton_Route_Send_clicked();
}

void MainWindow::field_Size_Confirm_Slot()
{
    ui->lineEdit_Offboard_Height_R->setText(QString::number(message.field_size_confirm.height));
    ui->lineEdit_Offboard_Length_R->setText(QString::number(message.field_size_confirm.length));
    ui->lineEdit_Offboard_Width_R->setText(QString::number(message.field_size_confirm.width));
    ui->lineEdit_Offboard_Times_R->setText(QString::number(message.field_size_confirm.times));
    computer_flag = message.field_size_confirm.confirm;
}

void MainWindow::on_pushButton_OFFBOARD_Imitate_clicked()
{
    //message.mode = "自动喷洒";
    //message.global_position.gps.satellites= 16;
    //global_GPS_Satellites_Slot();
    message.radio_rssi=-80;
    radio_Status_Slot();
}



void MainWindow::on_horizontalSlider_Spray_actionTriggered(int action)
{
    message.pump.spray_speed_sp = ((float) ui->horizontalSlider_Spray->value())/10.0;
    message.pump.pump_speed_sp = 1.0;
}

void MainWindow::pump_Status_Slot()
{
    ui->lineEdit_Spray_Speed->setText(QString::number((message.pump.spray_speed+1)*50));
}
