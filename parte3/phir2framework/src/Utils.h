#ifndef UTILS_H
#define UTILS_H

#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <vector>

//using namespace std;

enum ConnectionMode {SIMULATION, SERIAL, WIFI};
enum LogMode { NONE, RECORDING, PLAYBACK};
enum MotionMode {MANUAL_SIMPLE, MANUAL_VEL, WANDER, WALLFOLLOW, POTFIELD, ENDING};
enum MovingDirection {STOP, FRONT, BACK, LEFT, RIGHT, RESTART, DEC_ANG_VEL, INC_ANG_VEL, INC_LIN_VEL, DEC_LIN_VEL};

#define DEG2RAD(x) x*M_PI/180.0
#define RAD2DEG(x) x*180.0/M_PI

float normalizeAngleDEG(float a);
float normalizeAngleRAD(float a);

class Pose{
    public:
        Pose();
        Pose(double a, double b, bool c);

        friend std::ostream& operator<<(std::ostream& os, const Pose& p);

        double x, y, theta;
        bool up;
};

class LogFile
{
    public:
        LogFile(LogMode mode, std::string name);

        Pose readPose(std::string info);
        std::vector<float> readSensors(std::string info);

        void writePose(std::string info, Pose pose);
        void writeSensors(std::string s, std::vector<float> sensors);

        bool hasEnded();

    private:
        std::fstream file;
        std::string filename;
};

class Timer{
    public:
        Timer();

        void startCounting();
        void startLap();
        void stopCounting();

        float getTotalTime();
        float getLapTime();

    private:
        struct timeval tstart, tlapstart, tnow;
};

#endif // UTILS_H
