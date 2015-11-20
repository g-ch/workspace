#include "mainwindow.h"
#include <QApplication>
#include "ros/ros.h"
#include "receiver.h"
#include "camera.h"
#include "painterWiget.h"
#include <QTextCodec>
MavrosMessage message;
Camera camera_video;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    //中文字符设置
    QTextCodec *codec = QTextCodec::codecForName("UTF-8");
    QTextCodec::setCodecForTr(codec);
    QTextCodec::setCodecForLocale(QTextCodec::codecForLocale());
    QTextCodec::setCodecForCStrings(QTextCodec::codecForLocale());

    MainWindow w;
    w.showMaximized();//最大化显示

    ros::init(argc, argv, "receiver");
    message.start();
    camera_video.start();

    return a.exec();
}
