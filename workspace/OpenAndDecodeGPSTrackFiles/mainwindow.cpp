#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <fstream>
#include <iostream>
#include <math.h>
#include <QFileDialog>
using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
  cout<<fixed;
  cout.precision(8);
  QString fileName = QFileDialog::getOpenFileName(this, tr("open file"), "/home",  tr("track(*.gpx)"));

  if(fileName.length()!=0)
  {
      fstream gps_f;
      char *path = fileName.toLatin1().data();
      gps_f.open(path,ios::in);

      while(!gps_f.eof()){
          char str[300];
          gps_f >> str;
          //cout<<endl<<str;

          if(str[0]=='l'&&str[1]=='a'&&str[2]=='t') //lattitude
          {
              double lat=0.0;
              int point_p=8;

              for(int i=5;str[i]!='"';i++) //find '.' first
                  if(str[i]=='.') point_p=i;

              for(int m=point_p-1;str[m]!='"';m--)
                  lat += (double)(str[m]-'0')*pow(10,(point_p-1-m));

              for(int n=point_p+1;str[n]!='"';n++)
                  lat += ((double)(str[n]-'0'))/pow(10,(n-point_p));
              cout<<endl<<"lat:"<<lat;
          }

          if(str[0]=='l'&&str[1]=='o'&&str[2]=='n') //longitude
          {
              double lon=0.0;
              int point_p=8;

              for(int i=5;str[i]!='"';i++) //find '.' first
                  if(str[i]=='.') point_p=i;

              for(int m=point_p-1;str[m]!='"';m--)
                  lon += (double)(str[m]-'0')*pow(10,(point_p-1-m));

              for(int n=point_p+1;str[n]!='"';n++)
                  lon += ((double)(str[n]-'0'))/pow(10,(n-point_p));
              cout<<endl<<"lon:"<<lon;
          }
      }

      gps_f.close();
  }

}
