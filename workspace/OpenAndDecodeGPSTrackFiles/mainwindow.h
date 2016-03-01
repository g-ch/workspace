#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#define FENCE_AREA_LEN 720
#define FENCE_AREA_HEI 540

#define DEG_TO_RAD 	0.01745329251994
#define RAD_TO_DEG 	57.2957795130823

#define PI 3.14159265358979323846
#define PI_2 1.57079632679489661923

#define DBL_EPSILON 2.2204460492503131e-16
#define CONSTANTS_RADIUS_OF_EARTH 6371393

#define MAX_POINT_NUM 1000
#define MAX_DIRACTION_POINT_NUM 200

#define IDEAL_SPRAY_WIDTH 3.0

#include <QMainWindow>
#include <QListWidgetItem>
#include <QLabel>
#include <QPainter>
#include <QFileDialog>
#include <QMessageBox>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();

    void on_listWidget_itemClicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void delete_point(int x);

    bool restore_point();

    void draw_gps_fence();

    void gps_to_local(double lat, double lon, float *x, float *y);

    void local_to_gps(float x, float y, double *lat, double *lon);

    void turn_point_cal();

    float point_dist(float x1, float y1, float x2, float y2);

    float point_line_dist(float m, float n, float k, float b);

    void on_pushButton_5_clicked();

private:
    Ui::MainWindow *ui;
    QListWidgetItem *item_cp1;
    QListWidgetItem *item_cp2;
    QListWidgetItem *item_cp3;

    QLabel *gps_fence_label;


    //fence
    double gps_fence[MAX_POINT_NUM][3]; //(lat, lon, initial_sequence)
    double gps_fence_cp1[MAX_POINT_NUM][3];
    double gps_fence_cp2[MAX_POINT_NUM][3];
    double gps_fence_cp3[MAX_POINT_NUM][3];
    int gps_num;//start from 0
    int gps_num_cp1;
    int gps_num_cp2;
    int gps_num_cp3;

    //diraction
    double gps_diraction[MAX_DIRACTION_POINT_NUM][2]; //(lat, lon)
    float diraction_k;
    int diraction_p_num;

    //home position
    double home_lat;
    double home_lon;

    //distance between lines
    float dist_between_lines;

    //item sequence for listwidget
    int list_seq;
    int list_seq_cp1;
    int list_seq_cp2;
    int list_seq_cp3;
};

#endif // MAINWINDOW_H
