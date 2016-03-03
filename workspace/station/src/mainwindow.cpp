//********mianwindow.cpp*********

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "receiver.h"
#include "QDesktopWidget"


extern MavrosMessage message;

int choice=0;//选择显示的图片
bool send_button_pressed = false;//used in receiver.cpp, changed here


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


    init_paras();

    //set cout precision
    cout<<fixed;
    cout.precision(8);

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


    //飞行路径图背景设置
    ui->frame_Fly_Route->setFrameStyle(1);
    ui->frame_Fly_Route->setFixedSize(FLY_ROUTE_LABEL_WIDTH,FLY_ROUTE_LABEL_HEIGHT);

    QPalette   palette3;
    QPixmap pixmap3(":/icon/Icons/grass-720x540-2.png");//背景图片
    palette3.setBrush(ui->frame_Fly_Route-> backgroundRole(),QBrush(pixmap3));
    ui->frame_Fly_Route->setPalette(palette3);
    ui->frame_Fly_Route->setMask(pixmap3.mask());  //可以将图片中透明部分显示为透明的
    ui->frame_Fly_Route->setAutoFillBackground(true);

    fly_route_label = new QLabel(ui->frame_Fly_Route);
    fly_route_label->setFixedWidth(FLY_ROUTE_LABEL_WIDTH);
    fly_route_label->setFixedHeight(FLY_ROUTE_LABEL_HEIGHT);
    fly_route_label->move(0,0);


    ui->frame_Fly_Position->setFrameStyle(1);
    ui->frame_Fly_Position->setFixedSize(FLY_POSITION_LABEL_WIDTH,FLY_POSITION_LABEL_HEIGHT);

    QPalette   palette4;
    QPixmap pixmap4(":/icon/Icons/grass-720x540-2.png");//背景图片
    palette4.setBrush(ui->frame_Fly_Position-> backgroundRole(),QBrush(pixmap4));
    ui->frame_Fly_Position->setPalette(palette4);
    ui->frame_Fly_Position->setMask(pixmap4.mask());  //可以将图片中透明部分显示为透明的
    ui->frame_Fly_Position->setAutoFillBackground(true);

    fly_position_label = new QLabel(ui->frame_Fly_Position);
    fly_position_label->setFixedWidth(FLY_POSITION_LABEL_WIDTH);
    fly_position_label->setFixedHeight(FLY_POSITION_LABEL_HEIGHT);
    fly_position_label->move(0,0);

    //设置是否可用
    ui->pushButton_Route_Generate->setEnabled(true);
    ui->pushButton_Route_Reset->setEnabled(false);
    ui->pushButton_Route_Send->setEnabled(false);
    ui->pushButton_Delete_Point->setEnabled(false);
    ui->pushButton_OFFBOARD_Imitate->deleteLater();

    ui->progressBar_GPS->setRange(0,15);
    ui->progressBar_Battery->setRange(190,240);
    ui->progressBar_RC->setRange(0,200);

    //set conection display
    ui->label_Controller->setStyleSheet("background-color:red");
    ui->label_Computer->setStyleSheet("background-color:red");

    //offset initialize
    ui->dial_Offset_Angle->setValue(180);
    ui->lineEdit_Offset_Dist->setText(QString::number(0.0));
}


MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::init_paras()
{
    //value initialize
    gps_num = 0;
    gps_num_cp1 = 0;
    gps_num_cp2 = 0;
    gps_num_cp3 = 0;

    diraction_p_num = 0;
    diraction_k = 0.0;

    offset_angle_d = 30.0;
    offset_dist_m = 2.0;

    list_seq = 0;
    list_seq_cp1 = 0;
    list_seq_cp2 = 0;
    list_seq_cp3 = 0;

    home_lat = 31.027505;
    home_lon = 121.443987;

    dist_between_lines = IDEAL_SPRAY_WIDTH;

    intersection_num = 0;

    success_counter = SUCCESS_COUNTER_INIT;

    controller_flag=0;
    computer_flag=0;
    controller_flag_last=0;
    computer_flag_last=0;

    orientation_last=0;

    bool_flying=false;

    flying_time=0;
    flying_status_counter=0;
    flying_status_counter_last=0;

    //以下变量用于画路径图
    paint_scale = 1.0;
    position_num = 0;
    save_counter = 0;

    fly_distance = 0;
}

/****显示消息槽****/
void MainWindow::state_Mode_Slot()
{
    ui->label_Mode->setText(QString::fromStdString(message.mode));
    if(controller_flag!=10000)controller_flag+=1;
    else controller_flag=1;
}

void MainWindow::battey_Slot()
{
    ui->lineEdit_Battery->setText(QString::number(message.battery_voltage));
    ui->progressBar_Battery->setValue(((message.battery_voltage>24.5)?24.5:message.battery_voltage)*10);
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
    if(message.mode=="自动喷洒" && save_counter%5==1) //draw every 5 points
    {
         //translate coordinate
         real_position[position_num][1]= message.local_position.position.x; //N: x->y
         real_position[position_num][0]= -message.local_position.position.y; //W->E: y->x

         draw_route(2); //draw

         //计数器设置
         save_counter = 0;
         position_num += 1;

         //calculate total flying distance
         if(position_num>0)
         {
             fly_distance += point_dist(real_position[position_num][0],real_position[position_num][1],real_position[position_num+1][0],real_position[position_num+1][1]);
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
    ;
}

void MainWindow::temperature_Slot()
{
    ui->lineEdit_Temperature->setText(QString::number(message.temperature));
}

void MainWindow::time_Slot()
{
;
}


void MainWindow::paintEvent(QPaintEvent *event) /*****主界面绘图槽******/
{
    //QPainter mainwindow_painter(this);
    //mainwindow_painter.drawLine(QPoint(0,0),QPoint(100,100));
}

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
    timer_Slot();
}

void MainWindow::timer_Slot()
{
    if(controller_flag>0) ui->label_Controller->setStyleSheet("background-color:green");
    else ui->label_Controller->setStyleSheet("background-color:red");

    if(computer_flag>0) ui->label_Computer->setStyleSheet("background-color:green");
    else ui->label_Computer->setStyleSheet("background-color:red");

    controller_flag_last=controller_flag;
    computer_flag_last=computer_flag;
}

void MainWindow::on_pushButton_Reset_FlyingTime_clicked()
{
    flying_time=0;
    bool_flying=false;
}


int MainWindow::on_pushButton_Route_Generate_clicked()
{
    if(home_lat == 0)
    {
        QMessageBox message_box(QMessageBox::Warning,"警告","未连接到飞机或无GPS信号", QMessageBox::Cancel, NULL);
        message_box.exec();
        return 1;
    }
    else
    {
        //判断是否所有值都写好
        if(diraction_k!=0.0 && gps_fence[0][0]!= 0)
        {
            //if distance from home to first gps fence point is too long, reject
            float gps_fence_local_x;
            float gps_fence_local_y;
            gps_to_local(gps_fence[0][0], gps_fence[0][1], &gps_fence_local_x, &gps_fence_local_y);

            //if distance from home to first diraction point is too long, reject
            float diraction_local_x;
            float diraction_local_y;
            gps_to_local(gps_diraction[0][0], gps_diraction[0][1], &diraction_local_x, &diraction_local_y);

            if(point_dist(0.0, 0.0, gps_fence_local_x, gps_fence_local_y) > 5000)
            {
                QMessageBox message_box(QMessageBox::Warning,"警告","GPS围栏距离太远(>5km)", QMessageBox::Cancel, NULL);
                message_box.exec();
                return 2;
            }
            else if(point_dist(0.0, 0.0, diraction_local_x, diraction_local_y)>5000)
            {
                QMessageBox message_box(QMessageBox::Warning,"警告","GPS方向点距离太远(>5km)", QMessageBox::Cancel, NULL);
                message_box.exec();
                return 3;
            }
            else
            {
                offset_dist_m = ui->lineEdit_Offset_Dist->text().toFloat();
                turn_point_cal(); //calculate
            }

        }
        else if(gps_fence[0][0]==0) {
            QMessageBox message_box(QMessageBox::Warning,"警告","GPS围栏未导入", QMessageBox::Cancel, NULL);
            message_box.exec();
        }
        else if(diraction_k==0.0){
            QMessageBox message_box(QMessageBox::Warning,"警告","GPS方向未导入", QMessageBox::Cancel, NULL);
            message_box.exec();
        }
    }


    ui->textBrowser_Offboard_Message->append("生成成功！");
    ui->pushButton_Route_Send->setEnabled(true);
    return 0;
}


void MainWindow::on_pushButton_Route_Send_clicked()
{
    //message.field_size.length = field_size_length;
    //message.field_size.height = field_size_height;
    //message.field_size.width = field_size_width;
    //message.field_size.times = field_size_times;

    draw_route(1);

    //send_button_pressed = true;
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
    //field_size_length = 0;
    //field_size_width = 0;
    //field_size_times = 0;
    on_pushButton_Route_Send_clicked();
}

void MainWindow::field_Size_Confirm_Slot()
{
    //ui->lineEdit_Offboard_Height_R->setText(QString::number(message.field_size_confirm.height));
    //ui->lineEdit_Offboard_Length_R->setText(QString::number(message.field_size_confirm.length));
    //ui->lineEdit_Offboard_Width_R->setText(QString::number(message.field_size_confirm.width));
    //ui->lineEdit_Offboard_Times_R->setText(QString::number(message.field_size_confirm.times));
    //computer_flag = message.field_size_confirm.confirm;
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
    ui->lineEdit_Spray_Speed->setText(QString::number(message.pump.spray_speed));
}

void MainWindow::draw_gps_fence()
{

    //find min,max and calculate scale
    double lat_max = gps_fence[0][0];
    double lat_min = gps_fence[0][0];
    double lon_max = gps_fence[0][1];
    double lon_min = gps_fence[0][1];
    for(int i=1;i<=gps_num;i++)
    {
        if(gps_fence[i][0]>lat_max) lat_max = gps_fence[i][0];
        if(gps_fence[i][0]<lat_min) lat_min = gps_fence[i][0];
        if(gps_fence[i][1]>lon_max) lon_max = gps_fence[i][1];
        if(gps_fence[i][1]<lon_min) lon_min = gps_fence[i][1];
    }
    float scale_lat = (FLY_ROUTE_LABEL_HEIGHT-80)/(lat_max-lat_min); //40 for edge
    float scale_lon = (FLY_ROUTE_LABEL_WIDTH-120)/(lon_max-lon_min); //60 for edge
    float scale = (scale_lat<scale_lon) ? scale_lat : scale_lon;

    //draw lines
    QPainter painter;
    QImage image("/home/chg/catkin_ws/src/station/src/Icons/grass-720x540-2.png");//定义图片，并在图片上绘图方便显示
    painter.begin(&image);
    painter.setPen(QPen(Qt::blue,4));

    for(int j=0;j<gps_num;j++)
    {
        float px=(gps_fence[j][1]-lon_min)*scale+60;
        float py=FLY_ROUTE_LABEL_HEIGHT-(gps_fence[j][0]-lat_min)*scale-60;
        float pnx=(gps_fence[j+1][1]-lon_min)*scale+60;
        float pny=FLY_ROUTE_LABEL_HEIGHT-(gps_fence[j+1][0]-lat_min)*scale-60;
        painter.drawLine(px,py,pnx,pny);
        painter.drawEllipse(px,py,10,10);
        QRectF rect(px+20, py, px+75, py-20);
        painter.drawText(rect, Qt::AlignLeft,tr("Point")+QString::number(gps_fence[j][2]+1));
    }

    //draw line between the last and the first point
    painter.drawLine((gps_fence[gps_num][1]-lon_min)*scale+60,FLY_ROUTE_LABEL_HEIGHT-(gps_fence[gps_num][0]-lat_min)*scale-60,(gps_fence[0][1]-lon_min)*scale+60,FLY_ROUTE_LABEL_HEIGHT-(gps_fence[0][0]-lat_min)*scale-60);

   //draw last point
    float last_point_x = (gps_fence[gps_num][1]-lon_min)*scale+60;
    float last_point_y = FLY_ROUTE_LABEL_HEIGHT-(gps_fence[gps_num][0]-lat_min)*scale-60;
    painter.drawEllipse(last_point_x,last_point_y,10,10);
    QRectF rect(last_point_x+20, last_point_y, last_point_x+75, last_point_y-20);
    painter.drawText(rect, Qt::AlignLeft,tr("Point")+QString::number(gps_fence[gps_num][2]+1));

    painter.end();
    fly_route_label->setPixmap(QPixmap::fromImage(image));//在label上显示图片

}

void MainWindow::draw_route(int window)
{
    /*calculate scale with local position of fence and home position*/
    float scale = 0.0;
    float min_x = 0.0, max_x = 0.0, min_y = 0.0, max_y = 0.0;
    for(int i =0;i<=gps_num;i++)
    {
        if(min_x > gps_fence_local[i][0]) min_x = gps_fence_local[i][0];
        if(max_x < gps_fence_local[i][0]) max_x = gps_fence_local[i][0];
        if(min_y > gps_fence_local[i][1]) min_y = gps_fence_local[i][1];
        if(max_y < gps_fence_local[i][1]) max_y = gps_fence_local[i][1];
    }
    float scale_x = (FLY_ROUTE_LABEL_WIDTH - 120)/(max_x - min_x);
    float scale_y = (FLY_ROUTE_LABEL_HEIGHT - 80)/(max_y - min_y);
    scale = (scale_x < scale_y) ? scale_x : scale_y;

    QPainter painter;
    QImage image("/home/chg/catkin_ws/src/station/src/Icons/grass-720x540-2.png");//定义图片，并在图片上绘图方便显示
    painter.begin(&image);
    painter.setPen(QPen(Qt::blue,4));
    /*draw fence*/
    for(int j=0;j<gps_num;j++)
    {
        float px=(gps_fence_local[j][0]-min_x)*scale+60;
        float py=(FLY_ROUTE_LABEL_HEIGHT-(gps_fence_local[j][1]-min_y)*scale-60);
        float pnx=(gps_fence_local[j+1][0]-min_x)*scale+60;
        float pny=(FLY_ROUTE_LABEL_HEIGHT-(gps_fence_local[j+1][1]-min_y)*scale-60);
        painter.drawLine(px,py,pnx,pny);
        painter.drawEllipse(px,py,10,10);
        QRectF rect(px+20, py, px+75, py-20);
        painter.drawText(rect, Qt::AlignLeft,tr("Point")+QString::number(gps_fence[j][2]+1));
    }

    //draw line between the last and the first point
    painter.drawLine((gps_fence_local[gps_num][0]-min_x)*scale+60,FLY_ROUTE_LABEL_HEIGHT-(gps_fence_local[gps_num][1]-min_y)*scale-60,(gps_fence_local[0][0]-min_x)*scale+60,FLY_ROUTE_LABEL_HEIGHT-(gps_fence_local[0][1]-min_y)*scale-60);

    //draw last point
    float last_point_x = (gps_fence_local[gps_num][0]-min_x)*scale+60;
    float last_point_y = FLY_ROUTE_LABEL_HEIGHT-(gps_fence_local[gps_num][1]-min_y)*scale-60;
    painter.drawEllipse(last_point_x,last_point_y,10,10);
    QRectF rect(last_point_x+20, last_point_y, last_point_x+75, last_point_y-20);
    painter.drawText(rect, Qt::AlignLeft,tr("Point")+QString::number(gps_fence[gps_num][2]+1));

    /*draw route*/
    painter.setPen(QPen(Qt::yellow,3));
    painter.drawLine((0-min_x)*scale+60,FLY_ROUTE_LABEL_HEIGHT-(0-min_y)*scale-60,(route_p_local[0][0]-min_x)*scale+60,FLY_ROUTE_LABEL_HEIGHT-(route_p_local[0][1]-min_y)*scale-60);
    for(int i=0;i<intersection_num;i++)
    {
        painter.drawLine((route_p_local[i][0]-min_x)*scale+60,FLY_ROUTE_LABEL_HEIGHT-(route_p_local[i][1]-min_y)*scale-60,(route_p_local[i+1][0]-min_x)*scale+60,FLY_ROUTE_LABEL_HEIGHT-(route_p_local[i+1][1]-min_y)*scale-60);
    }

    painter.setPen(QPen(Qt::red,6));
    float home_local_x = (0-min_x)*scale+60;
    float home_local_y = FLY_ROUTE_LABEL_HEIGHT-(0-min_y)*scale-60;
    painter.drawEllipse(home_local_x,home_local_y,5,5);

    if(window==0)
    {
        painter.end();
        fly_route_label->setPixmap(QPixmap::fromImage(image));//在label上显示图片
    }
    else if(window==1)
    {
        painter.end();
        fly_position_label->setPixmap(QPixmap::fromImage(image));//在label上显示图片
    }
    else
    {
        painter.setPen(QPen(Qt::red,3));
        painter.translate(home_local_x,home_local_y);

        if(position_num == 0) painter.drawLine(0.0,0.0,1.0,1.0);
        else
        {
            for(int n=0;n<position_num;n++)
            {
                 painter.drawLine((real_position[n][0]-real_position[0][0])/scale,-(real_position[n][1]-real_position[0][1])/scale,
                      (real_position[n+1][0]-real_position[0][0])/scale,-(real_position[n+1][1]-real_position[0][1])/scale);
            }
        }
        painter.translate(-home_local_x,-home_local_y);
        painter.end();
        fly_position_label->setPixmap(QPixmap::fromImage(image));//在label上显示图片
    }
}

void MainWindow::delete_point(int x) //x start with 0
{
    //store
    memcpy(gps_fence_cp3,gps_fence_cp2,16000);
    memcpy(gps_fence_cp2,gps_fence_cp1,16000);
    memcpy(gps_fence_cp1,gps_fence,16000);
    gps_num_cp3=gps_num_cp2;
    gps_num_cp2=gps_num_cp1;
    gps_num_cp1=gps_num;
    list_seq_cp3=list_seq_cp2;
    list_seq_cp2=list_seq_cp1;
    list_seq_cp1=list_seq;

    //delete
    for(int i=x;i<gps_num;i++)
    {
        gps_fence[i][0]=gps_fence[i+1][0];
        gps_fence[i][1]=gps_fence[i+1][1];
        gps_fence[i][2]=gps_fence[i+1][2];
    }
    gps_num -= 1;
}

bool MainWindow::restore_point()
{
    if(gps_fence_cp1[0][0]>0){
        memcpy(gps_fence,gps_fence_cp1,16000);
        gps_num=gps_num_cp1;

        memcpy(gps_fence_cp1,gps_fence_cp2,16000);
        gps_num_cp1=gps_num_cp2;

        memcpy(gps_fence_cp2,gps_fence_cp3,16000);
        gps_fence_cp3[0][0]=0.0;
        gps_num_cp2=gps_num_cp3;

        //cout<<gps_fence[2][0]<<endl;

        return true;
    }
    return false;
}

void MainWindow::gps_to_local(double lat, double lon, float *x, float *y) //y is North, x is East
{
    double lat_rad = lat * DEG_TO_RAD;
    double lon_rad = lon * DEG_TO_RAD;
    double home_lat_rad = home_lat * DEG_TO_RAD;
    double home_lon_rad = home_lon * DEG_TO_RAD;

    //algorithm from px4
    /*double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double sin_home_lat = sin(home_lat_rad);
    double cos_home_lat = cos(home_lon_rad);
    double cos_d_lon = cos(lon_rad - home_lon_rad);

    double arg = sin_home_lat * sin_lat + cos_home_lat * cos_lat * cos_d_lon;
    cout<<"arg="<<arg<<endl;

    if (arg > 1.0) {
            arg = 1.0;

    } else if (arg < -1.0) {
            arg = -1.0;
    }

    double c = acos(arg);
    double k = (fabs(c) < DBL_EPSILON) ? 1.0 : (c / sin(c));

    *x = k * (cos_home_lat * sin_lat - sin_home_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
    *y = k * cos_lat * sin(lon_rad - home_lon_rad) * CONSTANTS_RADIUS_OF_EARTH;*/

    //easiest algorithm
    if(EASTERN_HEMISPHERE==1) *x = cos((lat_rad+home_lat_rad)/2)*CONSTANTS_RADIUS_OF_EARTH*(lon_rad - home_lon_rad); //East is positive
    else *x = -cos((lat_rad+home_lat_rad)/2)*CONSTANTS_RADIUS_OF_EARTH*(lon_rad - home_lon_rad);

    if(NORTHERN_HEMISPHERE==1) *y = (lat_rad - home_lat_rad)*CONSTANTS_RADIUS_OF_EARTH;  //NE Coodinate
    else *y = -(lat_rad - home_lat_rad)*CONSTANTS_RADIUS_OF_EARTH;

    //http://blog.sina.com.cn/s/blog_658a93570101hynw.html
}

void MainWindow::local_to_gps(float x, float y, double *lat, double *lon)
{
    double home_lat_rad = home_lat * DEG_TO_RAD;
    double home_lon_rad = home_lon * DEG_TO_RAD;

    *lat = (((double)x) / CONSTANTS_RADIUS_OF_EARTH + home_lat_rad) * RAD_TO_DEG;
    *lon = (home_lon_rad - ((double)y) / (CONSTANTS_RADIUS_OF_EARTH * cos(x/2.0/CONSTANTS_RADIUS_OF_EARTH + home_lat_rad))) * RAD_TO_DEG;
}

void MainWindow::turn_point_cal()
{
    //initial
    intersection_num = 0;

    //calculate fence local position
    if(gps_fence[0][0]>0){
        for(int i=0;i<=gps_num;i++)
        {
            gps_to_local(gps_fence[i][0],gps_fence[i][1],&gps_fence_local[i][0],&gps_fence_local[i][1]);
            //cout<<gps_fence_local[i][0]<<" + "<<gps_fence_local[i][1]<<endl;
        }
    }
    //calculate lines, form: y=kx+b
    float line_paras[gps_num+1][4]; //(k,b,x1,x2), para[0] is for the line between Point0 and Point1
    for(int i=0;i<=gps_num;i++)
    {
        if(i==gps_num)
        {
            line_paras[i][0] = (gps_fence_local[i][1]-gps_fence_local[0][1])/(gps_fence_local[i][0]-gps_fence_local[0][0]);
            line_paras[i][1] = gps_fence_local[i][1]-line_paras[i][0]*gps_fence_local[i][0];
            line_paras[i][2] = (gps_fence_local[i][0] < gps_fence_local[0][0]) ? gps_fence_local[i][0] : gps_fence_local[0][0];
            line_paras[i][3] = (gps_fence_local[i][0] > gps_fence_local[0][0]) ? gps_fence_local[i][0] : gps_fence_local[0][0];
        }
        else
        {
            line_paras[i][0] = (gps_fence_local[i][1]-gps_fence_local[i+1][1])/(gps_fence_local[i][0]-gps_fence_local[i+1][0]);
            line_paras[i][1] = gps_fence_local[i][1]-line_paras[i][0]*gps_fence_local[i][0];
            line_paras[i][2] = (gps_fence_local[i][0] < gps_fence_local[i+1][0]) ? gps_fence_local[i][0] : gps_fence_local[i+1][0];
            line_paras[i][3] = (gps_fence_local[i][0] > gps_fence_local[i+1][0]) ? gps_fence_local[i][0] : gps_fence_local[i+1][0];
        }
    }
    /*calculate turning points*/
    /*1.calculate proper y cross distance between lines*/
    float angle_k = atan(diraction_k);
    //if(angle_k<0) angle_k += PI;
    float cos_k = cos(angle_k);//= sqrt(1/(diraction_k*diraction_k+1));

    float min_b = 100000.0, max_b = -100000.0;

    for(int i=0;i<=gps_num;i++)
    {
        float b_temp = gps_fence_local[i][1] - diraction_k*gps_fence_local[i][0];
        min_b = (min_b > b_temp) ? b_temp : min_b;
        max_b = (max_b < b_temp) ? b_temp : max_b;
    }

    float delt_b_ideal = fabs(IDEAL_SPRAY_WIDTH / cos_k); //positive

    float b_distance = max_b - min_b;
    float times_f = b_distance / delt_b_ideal;
    float delt_b = 0.0;
    int times = 0;

    if(fabs(b_distance/((int)times_f)-IDEAL_SPRAY_WIDTH) < fabs(b_distance/((int)(times_f+1))-IDEAL_SPRAY_WIDTH))
    {
        times = (int)times_f;
        delt_b = b_distance/times;
    }
    else {
        times = (int)(times_f+1);
        delt_b = b_distance/times;
    }
   //cout<<"delt_b="<<delt_b<<endl;

    dist_between_lines = delt_b * cos_k;
    cout<<"dist_between_lines="<<dist_between_lines<<endl;

    /*2.calculate local intersection point*/
    float b_start = min_b + delt_b/2;
    for(int i=0;i<times;i++)
    {
        float intersection_temp[20][2]; //set 20 intersection points max for a line
        int intersection_num_temp = 0;
        //calculate
        for(int j=0;j<=gps_num;j++)
        {
            float x = (b_start - line_paras[j][1]) / (line_paras[j][0] - diraction_k);
            //cout<<"x="<<x<<endl;
            if(x>line_paras[j][2] && x<line_paras[j][3])
            {
                //cout<<"get in &&&"<<endl;
                intersection_temp[intersection_num_temp][0] = x;
                intersection_temp[intersection_num_temp][1] = diraction_k * x + b_start;
                intersection_num_temp ++;
            }
        }
        intersection_num_temp --;

        /*arrange intersection points sequence by value of x from small to large (point(x,y))*/
        float ex_x = 0.0, ex_y = 0.0;
        for(int m=0;m<=intersection_num_temp;m++)
        {
            for(int n=m+1;n<=intersection_num_temp;n++)
            {
                if(intersection_temp[m][0] > intersection_temp[n][0])
                {
                    ex_x = intersection_temp[m][0];
                    ex_y = intersection_temp[m][1];
                    intersection_temp[m][0] = intersection_temp[n][0];
                    intersection_temp[m][1] = intersection_temp[n][1];
                    intersection_temp[n][0] = ex_x;
                    intersection_temp[n][1] = ex_y;
                }
                //cout<<"intersection_temp[m][0]"<<intersection_temp[m][0]<<endl;
            }
        }
        //cout<<"intersection_temp[0][0]"<<intersection_temp[0][0]<<endl;

        /*store the first and the last intersection point on a line*/
        intersection_p_local[intersection_num][0] = intersection_temp[0][0];
        intersection_p_local[intersection_num][1] = intersection_temp[0][1];
        intersection_p_local[intersection_num+1][0] = intersection_temp[intersection_num_temp][0];
        intersection_p_local[intersection_num+1][1] = intersection_temp[intersection_num_temp][1];

        //cout<<"("<<intersection_p_local[intersection_num][0]<<","<<intersection_p_local[intersection_num][1]<<")"<<endl;
        //cout<<"("<<intersection_p_local[intersection_num+1][0]<<","<<intersection_p_local[intersection_num+1][1]<<")"<<endl;

        intersection_num += 2;

        b_start += delt_b;
    }
    intersection_num -= 1; //correct intersection_num

    /*eliminate the effect from spray length*/
    /*float e_x = IDEAL_SPRAY_LENTH/2 * cos(angle_k);
    float e_y = IDEAL_SPRAY_LENTH/2 * sin(angle_k);
    for(int i=0;i<= intersection_num;i+=2)
    {
        if(diraction_k > 0)
        {
            intersection_p_local[i][0] += e_x;
            intersection_p_local[i][1] += e_y;
            intersection_p_local[i+1][0] -= e_x;
            intersection_p_local[i+1][1] -= e_y;
        }
        else
        {
            intersection_p_local[i][0] += e_x;
            intersection_p_local[i][1] -= e_y;
            intersection_p_local[i+1][0] -= e_x;
            intersection_p_local[i+1][1] += e_y;
        }
    }*/

    /*3.arrange the best route points sequence according to home position*/
    float min_start_dist = 10000.0;
    float d[4];
    int method_num = -1;
    d[0] = point_dist(intersection_p_local[0][0], intersection_p_local[0][1], 0.0, 0.0);
    d[1] = point_dist(intersection_p_local[1][0], intersection_p_local[1][1], 0.0, 0.0);
    d[2] = point_dist(intersection_p_local[intersection_num-1][0], intersection_p_local[intersection_num-1][1], 0.0, 0.0);
    d[3] = point_dist(intersection_p_local[intersection_num][0], intersection_p_local[intersection_num][1], 0.0, 0.0);

    for(int i=0;i<3;i++)
    {
        if(d[i] < min_start_dist)
        {
            min_start_dist = d[i];
            method_num = i;
        }
    }
    cout<<"intersection_num="<<intersection_num<<endl;
    switch(method_num)
    {
        case 0:
        {
            int num = 0; //start from p0
            route_p_local[0][0] = intersection_p_local[num][0];
            route_p_local[0][1] = intersection_p_local[num][1];
            for(int i=1;i<=intersection_num;i++)
            {
                switch(i%4)
                {
                    case 1: num += 1; break;
                    case 2: num += 2; break;
                    case 3: num -= 1; break;
                    case 0: num += 2; break;
                }
               route_p_local[i][0] = intersection_p_local[num][0];
               route_p_local[i][1] = intersection_p_local[num][1];
            }
            break;
        }
        case 1:
        {
            int num = 1; //start from p1
            route_p_local[0][0] = intersection_p_local[num][0];
            route_p_local[0][1] = intersection_p_local[num][1];
            for(int i=1;i<=intersection_num;i++)
            {
                switch(i%4)
                {
                    case 1: num -= 1; break;
                    case 2: num += 2; break;
                    case 3: num += 1; break;
                    case 0: num += 2; break;
                }
               route_p_local[i][0] = intersection_p_local[num][0];
               route_p_local[i][1] = intersection_p_local[num][1];
            }
            break;
        }
        case 2:
        {
            int num = intersection_num-1; //start from p(n-1)
            route_p_local[0][0] = intersection_p_local[num][0];
            route_p_local[0][1] = intersection_p_local[num][1];
            for(int i=1;i<=intersection_num;i++)
            {
                switch(i%4)
                {
                    case 1: num += 1; break;
                    case 2: num -= 2; break;
                    case 3: num -= 1; break;
                    case 0: num -= 2; break;
                }
               route_p_local[i][0] = intersection_p_local[num][0];
               route_p_local[i][1] = intersection_p_local[num][1];
            }
            break;
        }
        case 3:
        {
            int num = intersection_num; //start from pn
            route_p_local[0][0] = intersection_p_local[num][0];
            route_p_local[0][1] = intersection_p_local[num][1];
            for(int i=1;i<=intersection_num;i++)
            {
                switch(i%4)
                {
                    case 1: num -= 1; break;
                    case 2: num -= 2; break;
                    case 3: num += 1; break;
                    case 0: num -= 2; break;
                }
               route_p_local[i][0] = intersection_p_local[num][0];
               route_p_local[i][1] = intersection_p_local[num][1];
            }
            break;
        }
        default: break;
    }

    /*offset to eliminate the effect from wind*/
    if(offset_dist_m != 0){
        float offset_x = offset_dist_m * cos(offset_angle_d*DEG_TO_RAD);
        float offset_y = offset_dist_m * sin(offset_angle_d*DEG_TO_RAD);
        for(int i=0;i<=intersection_num;i++)
        {
            route_p_local[i][0] += offset_x;
            route_p_local[i][1] += offset_y;
        }
    }


    /*4.translate to global coordnate, for flying back to break point*/
    for(int y=0;y<=intersection_num;y++)
    {
        local_to_gps(route_p_local[y][0],route_p_local[y][1],&route_p_gps[y][0],&route_p_gps[y][1]);
    }

    //draw
    draw_route(0);
}

float MainWindow::point_dist(float x1, float y1, float x2, float y2)
{
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

float MainWindow::point_line_dist(float m, float n, float k, float b)
{
    return (fabs(k*m-n+b))/(sqrt(k*k+1));
}



void MainWindow::on_pushButton_Open_Fence_clicked()
{
    //open a file using QFileDialog
    QString fileName = QFileDialog::getOpenFileName(this, tr("open file"), "/home",  tr("track(*.gpx)"));

    if(fileName.length()!=0) //if a file choosed
    {
        //initial
        gps_num = 0;
        gps_num_cp1 = 0;
        gps_num_cp2 = 0;
        gps_num_cp3 = 0;

        ui->listWidget_GPS_Point->clear();
        fstream gps_f;
        char *path = fileName.toLatin1().data();
        gps_f.open(path,ios::in);

        gps_num = 0; //initialize
        while(!gps_f.eof()){   //while not the end of file
            char str[300];
            gps_f >> str;
            //cout<<endl<<str;

            if(str[0]=='l'&&str[1]=='a'&&str[2]=='t') //lattitude
            {
                double lat=0.0;
                int point_p=8;

                for(int i=5;str[i]!='"';i++) //find '.' first
                    if(str[i]=='.') point_p=i;

                for(int m=point_p-1;str[m]!='"';m--) //integer
                    lat += (double)(str[m]-'0')*pow(10,(point_p-1-m));

                for(int n=point_p+1;str[n]!='"';n++) //fractional
                    lat += ((double)(str[n]-'0'))/pow(10,(n-point_p));
                //cout<<endl<<"lat:"<<lat;
                gps_fence[gps_num][0]=lat;//save
            }

            if(str[0]=='l'&&str[1]=='o'&&str[2]=='n') //longitude
            {
                double lon=0.0;
                int point_p=8;

                for(int i=5;str[i]!='"';i++) //find '.' first
                    if(str[i]=='.') point_p=i;

                for(int m=point_p-1;str[m]!='"';m--) //integer
                    lon += (double)(str[m]-'0')*pow(10,(point_p-1-m));

                for(int n=point_p+1;str[n]!='"';n++) //fractional
                    lon += ((double)(str[n]-'0'))/pow(10,(n-point_p));
                //cout<<endl<<"lon:"<<lon;
                gps_fence[gps_num][1]=lon;//save
                gps_fence[gps_num][2]=gps_num;
                gps_num++;//counter for next point
            }
        }
        gps_num -= 1; //correct num
        gps_f.close(); //reading finished

      //judge if gpx file can be used
        if(gps_num < 3)
        {
            gps_fence[0][0] = 0.0;
            QMessageBox message_box(QMessageBox::Warning,"警告","GPS点少于3个,请重新选择!", QMessageBox::Cancel, NULL);
            message_box.exec();
        }
        else{
            //add items
            for(int i=1;i<=gps_num+1;i++)
            {
                char name[10]="Point";
                char num[4];
                sprintf(num,"%d",i);
                strcat(name,num);
                ui->listWidget_GPS_Point->addItem(new QListWidgetItem(QObject::tr(name)));
            }
            //cout<<"$$"<<gps_fence[4][1];
            draw_gps_fence();
        }
  }
}


void MainWindow::on_pushButton_Open_Diraction_clicked()
{
    //open a file using QFileDialog
    QString fileName = QFileDialog::getOpenFileName(this, tr("open file"), "/home",  tr("track(*.gpx)"));

    if(fileName.length()!=0) //if a file choosed
    {
        fstream gps_d;
        char *path = fileName.toLatin1().data();
        gps_d.open(path,ios::in);

        diraction_p_num = 0; //initialize
        while(!gps_d.eof()){   //while not the end of file
            char str[300];
            gps_d >> str;
            //cout<<endl<<str;

            if(str[0]=='l'&&str[1]=='a'&&str[2]=='t') //lattitude
            {
                double lat=0.0;
                int point_p=8;

                for(int i=5;str[i]!='"';i++) //find '.' first
                    if(str[i]=='.') point_p=i;

                for(int m=point_p-1;str[m]!='"';m--) //integer
                    lat += (double)(str[m]-'0')*pow(10,(point_p-1-m));

                for(int n=point_p+1;str[n]!='"';n++) //fractional
                    lat += ((double)(str[n]-'0'))/pow(10,(n-point_p));
                cout<<endl<<"lat:"<<lat;
                gps_diraction[diraction_p_num][0]=lat;//save
            }

            if(str[0]=='l'&&str[1]=='o'&&str[2]=='n') //longitude
            {
                double lon=0.0;
                int point_p=8;

                for(int i=5;str[i]!='"';i++) //find '.' first
                    if(str[i]=='.') point_p=i;

                for(int m=point_p-1;str[m]!='"';m--) //integer
                    lon += (double)(str[m]-'0')*pow(10,(point_p-1-m));

                for(int n=point_p+1;str[n]!='"';n++) //fractional
                    lon += ((double)(str[n]-'0'))/pow(10,(n-point_p));
                cout<<endl<<"lon:"<<lon;
                gps_diraction[diraction_p_num][1]=lon;//save
                diraction_p_num++;//counter for next point
            }
        }
        diraction_p_num -= 1; //correct num
        gps_d.close(); //reading finished

        //judge if gpx file can be used
        if(diraction_p_num < 1)
        {
            gps_diraction[0][0] = 0.0;
            QMessageBox message_box(QMessageBox::Warning,"警告","GPS点少于2个,请重新选择!", QMessageBox::Cancel, NULL);
            message_box.exec();
        }
        else
        {
            //calculate diraction k with the first and the last point
            float first_px = 0.0;
            float first_py = 0.0;
            float last_px = 0.0;
            float last_py = 0.0;
            gps_to_local(gps_diraction[0][0],gps_diraction[0][1],&first_px,&first_py);
            gps_to_local(gps_diraction[diraction_p_num][0],gps_diraction[diraction_p_num][1],&last_px,&last_py);
            diraction_k = (last_py-first_py)/(last_px-first_px);

            //information out ***
            cout<<"diraction_k "<<diraction_k<<endl;
        }

    }
}


void MainWindow::on_pushButton_Delete_Point_clicked()
{
    //store seq and item
    list_seq = ui->listWidget_GPS_Point->currentRow();
    item_cp3=item_cp2;
    item_cp2=item_cp1;
    item_cp1=ui->listWidget_GPS_Point->currentItem();

    delete_point(list_seq);
    ui->listWidget_GPS_Point->takeItem(list_seq);

    //cout<<gps_fence[2][0]<<endl;
    draw_gps_fence();
}


void MainWindow::on_pushButton_Restore_Point_clicked()
{
    if(restore_point())
    {
        ui->listWidget_GPS_Point->insertItem(list_seq_cp1,item_cp1);
        item_cp1=item_cp2;
        item_cp2=item_cp3;

        list_seq_cp1=list_seq_cp2;
        list_seq_cp2=list_seq_cp3;
    }
    draw_gps_fence();
    //cout<<endl;
}

void MainWindow::on_listWidget_GPS_Point_itemClicked()
{
    list_seq = ui->listWidget_GPS_Point->currentRow();
    ui->pushButton_Delete_Point->setEnabled(true);
    //cout<<list_seq<<endl;
}

void MainWindow::on_dial_Offset_Angle_valueChanged(int value)
{
    //cout<<"value="<<value<<endl;
    offset_angle_d = (float)(- value + 270);
}
