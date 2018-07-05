#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
//using namespace std;

#include "Grid.h"
#include "PioneerBase.h"
#include "Potential.h"
#include "Utils.h"
#include "MCL.h"

class Robot
{
public:
    Robot();
    ~Robot();

    void initialize(ConnectionMode cmode, LogMode lmode, std::string fname, std::string mapName);
    void run();

    void move(MovingDirection dir);
    void draw(double xRobot, double yRobot, double angRobot);

    const Pose& getCurrentPose();

    void drawPath();
    void drawMCL();

    bool isReady();
    bool isRunning();

    Grid* grid;
    Potential* pot;
    MotionMode motionMode_;
    int viewMode;
    int numViewModes;
    MCL* mcl;

protected:

    Pose currentPose_;
    Pose prevLocalizationPose_;
    std::vector<Pose> path_;

    bool ready_;
    bool running_;

    // ARIA stuff
    PioneerBase base;

    // Log stuff
    LogFile* logFile_;
    LogMode logMode_;
    void writeOnLog();
    bool readFromLog();
    Pose readInitialPose();

    // Navigation stuff
    void wanderAvoidingCollisions();
    void wallFollow();
    bool isFollowingLeftWall_;

    void followPotentialField();

    // Mapping stuff
    float getOccupancyFromLogOdds(float logodds);
    void mappingWithHIMMUsingLaser();
    void mappingWithLogOddsUsingLaser();
    void mappingWithLogOddsUsingSonar();

    // Mapping stuff
    int minX_, minY_, maxX_, maxY_;

    void drawPotGradient(double scale);

    Timer controlTimer;
    void waitTime(float t);
    bool firstIteration;

};

#endif // ROBOT_H
