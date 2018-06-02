#ifndef __POTENTIAL_H__
#define __POTENTIAL_H__

class Potential;

#include <pthread.h>
#include <queue>
#include "Robot.h"
#include "Grid.h"

typedef struct
{
    int x,y;
} robotCell;

typedef struct
{
    int minX, maxX, minY, maxY;
} bbox;

class Potential {  
	public:
        Potential();
        ~Potential();

		void run();

        void initialize();

        void setNewRobotPose(Pose p);
        void setGrid(Grid* g);
        void setLocalRadius(int r);

        Grid* grid;
        double curPref;

	private:

        void updateCellsTypes();
        void expandObstacles();
        void initializePotentials();
        void iteratePotentials();
        void updateGradient();

        int radius;

        robotCell curPose;
        bbox curLimits;

        robotCell newPose;
        bbox newLimits;

};


#endif /* __POTENTIAL_H__ */
