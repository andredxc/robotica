#ifndef PIONEERBASE_H
#define PIONEERBASE_H

#include <Aria.h>

#include "Utils.h"

class PioneerBase
{
public:
    PioneerBase();

    // ARIA stuff
    bool initialize(ConnectionMode cmode);
    void closeARIAConnection();

    // Draw stuff
    void drawBase();
    void drawSonars(bool drawCones=false);
    void drawLasers(bool fill=true);

    // Navigation stuff
    void setMovementSimple(MovingDirection dir);
    void setMovementVel(MovingDirection dir);

    void setWheelsVelocity_fromLinAngVelocity(float linV, float angV);
    void setWheelsVelocity(float vl, float vr);
    bool isMoving();
    void resumeMovement();
    void stopMovement();

    // Sensors stuff
    bool readOdometryAndSensors();
    const Pose& getOdometry();
    const std::vector<float>& getLaserReadings();
    const std::vector<float>& getSonarReadings();
    int getNumLasers();
    int getNumSonars();
    float getMaxLaserRange();
    float getMaxSonarRange();
    void setOdometry(const Pose &o);
    void setSonarReadings(const std::vector<float> &s);
    void setLaserReadings(const std::vector<float> &l);

    float getMinSonarValueInRange(int idFirst, int idLast);
    float getMinLaserValueInRange(int idFirst, int idLast, int kernelSize=0);

    int getNearestSonarBeam(float angle);
    float getAngleOfSonarBeam(int k);
    float getKthSonarReading(int k);

    int getNearestLaserBeam(float angle);
    float getAngleOfLaserBeam(int k);
    float getKthLaserReading(int k);

private:
    // ARIA stuff
    ArRobot robot_;
    ArRobotConnector *robotConnector_;
    ArArgumentParser *parser_;
    ArSonarDevice sonarDev_;
    ArSick sick_;
    ArLaserConnector *laserConnector_;
    bool initARIAConnection(int argc, char** argv);
    void resetSimPose();

    bool resetSimPose_;

    // Navigation stuff
    double vLeft_, vRight_;
    double oldVLeft_, oldVRight_;
    double linVel_, angVel_;

    // Sensors stuff
    int numSonars_;
    std::vector<float> sonars_;
    float maxSonarRange_;
    int numLasers_;
    std::vector<float> lasers_;
    float maxLaserRange_;

    Pose odometry_;
    Pose truePose_;
};

#endif // PIONEERBASE_H
