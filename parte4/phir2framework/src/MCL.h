#ifndef MCL_H
#define MCL_H

#include <string>
#include <vector>
#include <random>

#include "Grid.h"
#include "Utils.h"

typedef struct{
    double rot1;
    double trans;
    double rot2;
} Action;

typedef struct{
    Pose p;
    double w;
} MCLparticle;

class MCL
{
public:
    MCL(Pose initialPose, float maxRange, std::string mapName);
    ~MCL();

    void run(const Action &u, const std::vector<float> &z);

    void draw();
    int mapWidth;
    int mapHeight;

    bool transparency;

private:

    void sampling(const Action &u);
    void weighting(const std::vector<float> &z);
    void resampling();

    float computeExpectedMeasurement(int index, Pose &pose);
    double measurementLikelihood(double measuredValue, double expectedValue, double variance);

    void readMap(std::string mapName);
    void initParticles();

    std::default_random_engine* generator;

    CellType** mapCells;
    double scale;
    float maxRange;

    Pose curPose;
    std::vector<Pose> robotPath;

    int numParticles;
    std::vector<MCLparticle> particles;

};

#endif // MCL_H
