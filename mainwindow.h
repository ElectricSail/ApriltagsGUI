#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <opencv2/opencv.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
#include <string>
#include <fstream>

// April tags detector and various families that can be selected by command line option
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"
using namespace cv;

#include <iostream>
using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    VideoCapture cap;

    //--------------------------------------------------------------

    AprilTags::TagDetector* m_tagDetector;

    bool m_draw = true; // draw image and April tag detections?
    bool m_arduino; // send tag detections to serial port?
    bool m_timing = false; // print timing information for each tag extraction call

    int m_width = 640; // image size in pixels
    int m_height = 480;
    double m_tagSize = .2; // April tag side length in meters of square black frame
    double m_fx = 600; // camera focal length in pixels
    double m_fy = 600;
    double m_px = m_width/2; // camera principal point
    double m_py = m_width/2;


    int m_deviceId; // camera id (in case of multiple cameras)

    list<string> m_imgNames;

    cv::VideoCapture m_cap;

    int m_exposure;
    int m_gain;
    int m_brightness;

    double OPTIMIZED_X;
    double OPTIMIZED_Y;
    double OPTIMIZED_PITCH;
    double OPTIMIZED_ROLL;
    double OPTIMIZED_YAW;
    double xnew = OPTIMIZED_X;
    double ynew = OPTIMIZED_Y;
    time_t tstart = time(0);
    int start_s = clock();
    double timenow;

    double delta_t;
    double timeold;
    double delta_x;
    double delta_y;
    double xold;
    double yold;
    double velmag ;
    double veltheta;

    string TOD;
    string optimizedheader;


    //-------------------------------------------------------------------------

private slots:

    void on_pushButton_Webcam_clicked();

    void on_pushButton_Apriltags_clicked();

    void update_window();
    void update_window2();

    void loop();

    void processImage(cv::Mat& image, cv::Mat& image_gray);

    void print_detection(AprilTags::TagDetection& detection);

    void on_checkBox_crosshair_clicked();

    void on_checkBox_data_clicked();

    void on_checkBox_plot_clicked();

    void plot();

    void centerAndResize();


private:
    Ui::MainWindow *ui;

    QVector<double> qv_x, qv_y;

    QTimer *timer;
    //VideoCapture cap;

    Mat frame;
    QImage qt_image;

    bool crosshair=false;
    bool data=true; //record data on/off
    bool m_plot=false; //plot data on/off
    bool m_click = true;  //turn webcam on/off with one button
    bool m_april = true;  //turn apriltags on/off with one button

    int i;
};

#endif // MAINWINDOW_H
