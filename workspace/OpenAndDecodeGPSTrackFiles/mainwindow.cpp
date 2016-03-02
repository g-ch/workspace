#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <fstream>
#include <iostream>
#include <math.h>


using namespace std;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->pushButton_2->setEnabled(false);

    gps_fence_label = new QLabel(ui->frame);
    gps_fence_label->setFixedWidth(FENCE_AREA_LEN);
    gps_fence_label->setFixedHeight(FENCE_AREA_HEI);
    gps_fence_label->move(0,0);

    gps_num = 0;
    gps_num_cp1 = 0;
    gps_num_cp2 = 0;
    gps_num_cp3 = 0;

    diraction_p_num = 0;
    diraction_k = 0.0;

    list_seq = 0;
    list_seq_cp1 = 0;
    list_seq_cp2 = 0;
    list_seq_cp3 = 0;

    home_lat = 31.027505;
    home_lon = 121.443987;

    dist_between_lines = IDEAL_SPRAY_WIDTH;

    intersection_num = 0;

    //set cout precision
    cout<<fixed;
    cout.precision(8);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked() //open fence file
{
    //open a file using QFileDialog
    QString fileName = QFileDialog::getOpenFileName(this, tr("open file"), "/home",  tr("track(*.gpx)"));

    if(fileName.length()!=0) //if a file choosed
    {
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
                cout<<endl<<"lat:"<<lat;
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
                cout<<endl<<"lon:"<<lon;
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
                ui->listWidget->addItem(new QListWidgetItem(QObject::tr(name)));
            }
            cout<<"$$"<<gps_fence[4][1];
            draw_gps_fence();
        }
  }

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
    float scale_lat = (FENCE_AREA_HEI-80)/(lat_max-lat_min); //40 for edge
    float scale_lon = (FENCE_AREA_LEN-120)/(lon_max-lon_min); //60 for edge
    float scale = (scale_lat<scale_lon) ? scale_lat : scale_lon;

    //draw lines
    QPainter painter;
    QImage image("/home/chg/catkin_ws/src/station/src/Icons/grass-720x540-2.png");//定义图片，并在图片上绘图方便显示
    painter.begin(&image);
    painter.setPen(QPen(Qt::blue,4));

    for(int j=0;j<gps_num;j++)
    {
        float px=(gps_fence[j][1]-lon_min)*scale+60;
        float py=(FENCE_AREA_HEI-(gps_fence[j][0]-lat_min)*scale-60)/1.2; //1.2 to calibrate
        float pnx=(gps_fence[j+1][1]-lon_min)*scale+60;
        float pny=(FENCE_AREA_HEI-(gps_fence[j+1][0]-lat_min)*scale-60)/1.2;
        painter.drawLine(px,py,pnx,pny);
        painter.drawEllipse(px,py,10,10);
        QRectF rect(px+20, py+20, px+75, py+75);
        painter.drawText(rect, Qt::AlignLeft,tr("Point")+QString::number(gps_fence[j][2]+1));
    }

    //draw line between the last and the first point
    painter.drawLine((gps_fence[gps_num][1]-lon_min)*scale+60,(FENCE_AREA_HEI-(gps_fence[gps_num][0]-lat_min)*scale-60)/1.2,(gps_fence[0][1]-lon_min)*scale+60,(FENCE_AREA_HEI-(gps_fence[0][0]-lat_min)*scale-60)/1.2);
    float last_point_x = (gps_fence[gps_num][1]-lon_min)*scale+60;
    float last_point_y = (FENCE_AREA_HEI-(gps_fence[gps_num][0]-lat_min)*scale-60)/1.2;

    //draw last point
    painter.drawEllipse(last_point_x,last_point_y,10,10);
    QRectF rect(last_point_x+20, last_point_y+20, last_point_x+75, last_point_y+75);
    painter.drawText(rect, Qt::AlignLeft,tr("Point")+QString::number(gps_fence[gps_num][2]+1));
    painter.end();
    gps_fence_label->setPixmap(QPixmap::fromImage(image));//在label上显示图片

}

void MainWindow::draw_route()
{
    ;
}

void MainWindow::on_listWidget_itemClicked()
{
    list_seq = ui->listWidget->currentRow();
    ui->pushButton_2->setEnabled(true);
    cout<<list_seq<<endl;
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

        cout<<gps_fence[2][0]<<endl;

        return true;
    }
    return false;
}

void MainWindow::on_pushButton_2_clicked() //store and delete point
{
    //store seq and item
    list_seq = ui->listWidget->currentRow();
    item_cp3=item_cp2;
    item_cp2=item_cp1;
    item_cp1=ui->listWidget->currentItem();

    delete_point(list_seq);
    ui->listWidget->takeItem(list_seq);

    cout<<gps_fence[2][0]<<endl;
    draw_gps_fence();
}

void MainWindow::on_pushButton_3_clicked() //restore point
{
    if(restore_point())
    {
        ui->listWidget->insertItem(list_seq_cp1,item_cp1);
        item_cp1=item_cp2;
        item_cp2=item_cp3;

        list_seq_cp1=list_seq_cp2;
        list_seq_cp2=list_seq_cp3;
    }
    draw_gps_fence();
    cout<<endl;
}

void MainWindow::gps_to_local(double lat, double lon, float *x, float *y)
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
    *x = (lat_rad - home_lat_rad)*CONSTANTS_RADIUS_OF_EARTH;  //NWU Coodinate
    *y = cos((lat_rad+home_lat_rad)/2)*CONSTANTS_RADIUS_OF_EARTH*(home_lon_rad-lon_rad);
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
    //calculate fence local position

    if(gps_fence[0][0]>0){
        for(int i=0;i<=gps_num;i++)
        {
            gps_to_local(gps_fence[i][0],gps_fence[i][1],&gps_fence_local[i][0],&gps_fence_local[i][1]);
            cout<<gps_fence_local[i][0]<<" + "<<gps_fence_local[i][1]<<endl;
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
    //1.calculate proper y cross distance between lines
    float cos_k = sqrt(1/(diraction_k*diraction_k+1));

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
    cout<<"delt_b="<<delt_b<<endl;

    dist_between_lines = delt_b * cos_k;
    cout<<"dist_between_lines="<<dist_between_lines<<endl;

    //2.calculate local intersection point
    float b_start = min_b + delt_b/2;
    for(int i=0;i<times;i++)
    {
        float intersection_temp[20][2]; //set 20 intersection points max for a line
        int intersection_num_temp = 0;
        //calculate
        for(int j=0;j<=gps_num;j++)
        {
            float x = (b_start - line_paras[j][1]) / (line_paras[j][0] - diraction_k);
            if(x>line_paras[j][2] && x<line_paras[j][3])
            {
                intersection_temp[intersection_num_temp][0] = x;
                intersection_temp[intersection_num_temp][1] = diraction_k * x + b_start;
                intersection_num_temp ++;
            }
        }
        intersection_num_temp --;

        //arrange intersection points sequence by value of x from small to large (point(x,y))
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
            }
        }
        //store the first and the last intersection point a line
        intersection_p_local[intersection_num][0] = intersection_temp[0][0];
        intersection_p_local[intersection_num][1] = intersection_temp[0][1];
        intersection_p_local[intersection_num+1][0] = intersection_temp[intersection_num_temp][0];
        intersection_p_local[intersection_num+1][1] = intersection_temp[intersection_num_temp][1];
        intersection_num += 2;

        b_start += delt_b;
    }
    intersection_num -= 2; //correct intersection_num

    //3.translate to global coordnate, for flying back to break point
    for(int y=0;y<=intersection_num;y++)
    {
        local_to_gps(intersection_p_local[y][0],intersection_p_local[y][1],&intersection_p_gps[y][0],&intersection_p_gps[y][1]);
    }

    //4.arrange the best route points sequence according to home position


}

void MainWindow::on_pushButton_4_clicked() //calculate turning points
{
    if(diraction_k!=0.0 && gps_fence[0][0]!= 0) turn_point_cal(); //calculate
    else if(gps_fence[0][0]==0) {
        QMessageBox message_box(QMessageBox::Warning,"警告","GPS围栏未导入", QMessageBox::Cancel, NULL);
        message_box.exec();
    }
    else if(diraction_k==0.0){
        QMessageBox message_box(QMessageBox::Warning,"警告","GPS方向未导入", QMessageBox::Cancel, NULL);
        message_box.exec();
    }
}

float MainWindow::point_dist(float x1, float y1, float x2, float y2)
{
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

float MainWindow::point_line_dist(float m, float n, float k, float b)
{
    return (fabs(k*m-n+b))/(sqrt(k*k+1));
}

void MainWindow::on_pushButton_5_clicked() //read diraction
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
        if(diraction_p_num < 2)
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

