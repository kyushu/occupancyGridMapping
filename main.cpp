
/*
The robot equipped with eight sonar rangefinder sensors circulates in 
    an environment to map it. This robot is provided with its exact 
    poses at each timestamp.

measurement.txt : The measurements from the sonar rangefinder sensors 
                    attached to the robot at each time stamp recorded 
                    over a period of 413 seconds. (timestamp, measurement 1-8).

poses.txt       : The exact robot poses at each timestamp recorded over 
                    a period of 413 seconds. (timestamp, x, y, ϴ).
 */

#include <stdio.h> // C
#include <iostream>
#include <math.h>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

// Sensor characteristic: Min and Max ranges of the beams
double Zmax = 5000, Zmin = 170;
// Defining free cells(lfree), occupied cells(locc), unknown cells(l0) log odds values
double l0 = 0, locc = 0.4, lfree = -0.4;
// Grid dimensions
double gridWidth = 100, gridHeight = 100;
// Map dimensions
double mapWidth = 30000, mapHeight = 15000;
// Robot size with respect to the map 
double robotXOffset = mapWidth / 5, robotYOffset = mapHeight / 3;
// Defining an l vector to store the log odds values of each cell
vector< vector<double> > l(mapWidth/gridWidth, vector<double>(mapHeight/gridHeight));

double inverseSensorModel(double x, double y, double theta, double xi, double yi, double sensorData[])
{
    //******************Code the Inverse Sensor Model Algorithm**********************//
    // Defining Sensor Characteristics
    double Zk, thetaK, sensorTheta;
    double minDelta = -1;
    double alpha = 200, beta = 20;
    
    //******************TODO: Compute r and phi**********************//
    double r = sqrt(pow(xi-x, 2) + pow(yi-y, 2));
    double phi = atan2(yi-y, xi-x) - theta;

    //Scaling Measurement to [-90 -37.5 -22.5 -7.5 7.5 22.5 37.5 90]
    for (int i = 0; i < 8; i++) {
        if (i == 0) {
            sensorTheta = -90 * (M_PI / 180);
        }
        else if (i == 1) {
            sensorTheta = -37.5 * (M_PI / 180);
        }
        else if (i == 6) {
            sensorTheta = 37.5 * (M_PI / 180);
        }
        else if (i == 7) {
            sensorTheta = 90 * (M_PI / 180);
        }
        else {
            sensorTheta = (-37.5 + (i - 1) * 15) * (M_PI / 180);
        }

        if (fabs(phi - sensorTheta) < minDelta || minDelta == -1) {
            Zk = sensorData[i];
            thetaK = sensorTheta;
            minDelta = fabs(phi - sensorTheta);
        }
    }

    //******************TODO: Evaluate the three cases**********************//
    // You also have to consider the cells with Zk > Zmax or Zk < Zmin as unkown states
    if (r > min((double)Zmax, Zk + alpha / 2) || fabs(phi - thetaK) > beta / 2 || Zk > Zmax || Zk < Zmin) {
        return l0;
    }
    else if (Zk < Zmax && fabs(r - Zk) < alpha / 2) {
        return locc;
    }
    else if (r <= Zk) {
        return lfree;
    }
}

void occupancyGridMapping(double Robotx, double Roboty, double Robottheta, double sensorData[])
{
    //1 - TODO: Generate a grid (size 300x150) and then loop through all the cells
    //2- TODO: Compute the center of mass of each cell xi and yi 
    //double xi = x * gridWidth + gridWidth / 2 - robotXOffset;
    //double yi = -(y * gridHeight + gridHeight / 2) + robotYOffset;
    //3- TODO: Check if each cell falls under the perceptual field of the measurements
    
    for(size_t x=0; x<mapWidth/gridWidth; x++) {
        for(size_t y=0; y<mapHeight/gridHeight;y++) {
            // Compute the center of mass of each cell xi and yi
            double xi = x*gridWidth + gridWidth/2 - robotXOffset;
            double yi = -(y*gridHeight + gridHeight/2) + robotYOffset;
            
            // [xi, yi]         : the position of cell we want to check
            //                      if it is in the perceptual field
            //                      of the measurements.
            // [Robotx, Roboty] : the position of Robot.
            // Zmax             : The maximum range of beam of sensor.
            double dist_robot_cell = sqrt(pow(xi - Robotx, 2) + pow(yi-Roboty, 2));
            if(dist_robot_cell <= Zmax) {
                // binary Bayes filter algorithm
                l[x][y] = l[x][y] + inverseSensorModel(Robotx, Roboty, Robottheta, xi, yi, sensorData) - l0;
            }
        }
    }   
}

cv::Mat visualization()
{
    int imgW = int(mapWidth / gridWidth)+1;
    int imgH = int(mapHeight / gridHeight)+1;
    cv::Mat image = cv::Mat::zeros(imgW, imgH, CV_8UC3);

    std::cout << "mapWidth / gridWidth: " << mapWidth / gridWidth << '\n';
    std::cout << "mapHeight / gridHeight: " << mapHeight / gridHeight << '\n';

    // Draw every grid of the map:
    for (double x = 0; x < mapWidth / gridWidth; x++) {
        cout << "Remaining Rows= " << mapWidth / gridWidth - x << endl;
        for (double y = 0; y < mapHeight / gridHeight; y++) {
            // std::cout << "y" << x << '\n';
            if (l[x][y] == 0) { //Green unkown state
                image.at<cv::Vec3b>(x,y)[0] = 0;
                image.at<cv::Vec3b>(x,y)[1] = 255;
                image.at<cv::Vec3b>(x,y)[2] = 0;
            }
            else if (l[x][y] > 0) { //Black occupied state
                image.at<cv::Vec3b>(x,y)[0] = 0;
                image.at<cv::Vec3b>(x,y)[1] = 0;
                image.at<cv::Vec3b>(x,y)[2] = 0;
            }
            else { //Red free state
                image.at<cv::Vec3b>(x,y)[0] = 0;
                image.at<cv::Vec3b>(x,y)[1] = 0;
                image.at<cv::Vec3b>(x,y)[2] = 255;
            }
        }
    }

    return image;
}

int main()
{
    double timeStamp;
    double measurementData[8];
    double robotX, robotY, robotTheta;

    FILE* posesFile = fopen("../data/poses.txt", "r");
    if(posesFile == NULL)
    {
        printf("can't read poses.txt\n");
        return 0;
    }
    FILE* measurementFile = fopen("../data/measurement.txt", "r");
    if(measurementFile == NULL)
    {
        printf("can't read measurement.txt\n");
        return 0;
    }

    // Scanning the files and retrieving measurement and poses at each timestamp
    while (fscanf(posesFile, "%lf %lf %lf %lf", &timeStamp, &robotX, &robotY, &robotTheta) != EOF) {
        fscanf(measurementFile, "%lf", &timeStamp);
        for (int i = 0; i < 8; i++) {
            fscanf(measurementFile, "%lf", &measurementData[i]);
        }
        occupancyGridMapping(robotX, robotY, (robotTheta / 10) * (M_PI / 180), measurementData);
    }
    
    // Visualize the map at the final step
    cout << "Wait for the image to generate" << endl;
    cv::Mat image = visualization();
    cv::imshow("map", image);
    cv::waitKey(0);
    cout << "Done!" << endl;
    
    return 0;
}