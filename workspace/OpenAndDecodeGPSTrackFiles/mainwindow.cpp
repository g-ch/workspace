#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <fstream>
#include <iostream>
#include <math.h>
#include <QFileDialog>

using namespace std;

double gps_fence[1000][2];
double gps_fence_cp1[1000][2];
double gps_fence_cp2[1000][2];
double gps_fence_cp3[1000][2];

int gps_num = 0;
int gps_num_cp1 = 0;
int gps_num_cp2 = 0;
int gps_num_cp3 = 0;

int list_seq = 0;
int list_seq_cp = 0;
int list_seq_cp1 = 0;
int list_seq_cp2 = 0;
int list_seq_cp3 = 0;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->pushButton_2->setEnabled(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
  //set cout precision
  cout<<fixed;
  cout.precision(8);

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
              gps_num++;//counter for next point
          }
      }

      gps_f.close(); //reading finished
      //add items
      for(int i=1;i<=gps_num;i++)
      {
          char name[10]="Point";
          char num[4];
          sprintf(num,"%d",i);
          strcat(name,num);
          ui->listWidget->addItem(new QListWidgetItem(QObject::tr(name)));
      }

  }

}

void MainWindow::on_listWidget_itemClicked(QListWidgetItem *item)
{
    list_seq = ui->listWidget->currentRow();
    ui->pushButton_2->setEnabled(true);
    cout<<list_seq<<endl;
}

void delete_point(int x) //x start with 0
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
    list_seq_cp1=list_seq_cp;
    list_seq_cp =list_seq;

    //delete
    for(int i=x;i<gps_num;i++)
    {
        gps_fence[i][0]=gps_fence[i+1][0];
        gps_fence[i][1]=gps_fence[i+1][1];
    }
    gps_num -= 1;
}

bool restore_point()
{
    if(gps_fence_cp1[0][0]>0){
        memcpy(gps_fence,gps_fence_cp1,16000);
        gps_num=gps_num_cp1;
        list_seq_cp=list_seq_cp1;

        if(gps_fence_cp2[0][0]>0){
            memcpy(gps_fence_cp1,gps_fence_cp2,16000);
            gps_num_cp1=gps_num_cp2;
            list_seq_cp1=list_seq_cp2;

            if(gps_fence_cp2[0][0]>0){
                memcpy(gps_fence_cp2,gps_fence_cp3,16000);
                gps_fence_cp3[0][0]=0.0;
                gps_num_cp2=gps_num_cp3;
                list_seq_cp2=list_seq_cp1;
            }
        }
        cout<<gps_fence[2][0]<<endl;
        return true;
    }
    return false;
}

void MainWindow::on_pushButton_2_clicked()
{
    delete_point(list_seq);
    ui->listWidget->takeItem(list_seq);
    cout<<gps_fence[2][0]<<endl;
}

void MainWindow::on_pushButton_3_clicked()
{
    if(restore_point())
    {
        QListWidgetItem *newitem = new QListWidgetItem;
        newitem->setText(tr("Point")+QString::number(list_seq_cp));
        ui->listWidget->insertItem(list_seq_cp,newitem);
    }
}
