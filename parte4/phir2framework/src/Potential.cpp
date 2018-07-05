#include "Potential.h"

////////////////////////
/// Métodos Públicos ///
////////////////////////

Potential::Potential()
{
    curPref = 0.0;

    newPose.x = 0;
    newPose.y = 0;

    newLimits.minX = newLimits.minY = 1000;
    newLimits.maxX = newLimits.maxY = -1000;
}

Potential::~Potential()
{}

void Potential::setGrid(Grid *g)
{
    grid = g;
}

void Potential::setLocalRadius(int r)
{
    radius = 1.2*r*grid->getMapScale();
}

void Potential::setNewRobotPose(Pose p)
{
    newPose.x = (int)(p.x*grid->getMapScale());
    newPose.y = (int)(p.y*grid->getMapScale());

    newLimits.minX = std::min(newLimits.minX,newPose.x-radius);
    newLimits.maxX = std::max(newLimits.maxX,newPose.x+radius);
    newLimits.minY = std::min(newLimits.minY,newPose.y-radius);
    newLimits.maxY = std::max(newLimits.maxY,newPose.y+radius);
}

void Potential::run()
{
    // use last pose informed by the robot
    curPose = newPose;
    curLimits = newLimits;

    pthread_mutex_lock(grid->mutex);
    updateCellsTypes();
    expandObstacles();
    pthread_mutex_unlock(grid->mutex);

    initializePotentials();

    for(int i=0; i<100; i++)
        iteratePotentials();

    updateGradient();

}

////////////////////////
/// Métodos Privados ///
////////////////////////

void Potential::updateCellsTypes()
{
    Cell* c;

    // the type of a cell can be defined as:
    // c->type = UNEXPLORED
    // c->type = OCCUPIED
    // c->type = FREE


    // TODO: classify cells surrounding the robot
    //
    //  (curPose.x-radius,curPose.y+radius)  -------  (curPose.x+radius,curPose.y+radius)
    //                     |                       \                         |
    //                     |                        \                        |
    //                     |                         \                       |
    //  (curPose.x-radius,curPose.y-radius)  -------  (curPose.y+radius,curPose.y-radius)





}

void Potential::expandObstacles()
{
    int width=1;
    Cell *c, *n;

    for(int i=curPose.x-radius;i<=curPose.x+radius;i++){
        for(int j=curPose.y-radius;j<=curPose.y+radius;j++){
            c = grid->getCell(i,j);
            if(c->type == NEAROBSTACLE)
                c->type = FREE;
        }
    }

    for(int i=curPose.x-radius;i<=curPose.x+radius;i++){
        for(int j=curPose.y-radius;j<=curPose.y+radius;j++){
            c = grid->getCell(i,j);

            if(c->type == OCCUPIED){
                for(int x=i-width;x<=i+width;x++){
                    for(int y=j-width;y<=j+width;y++){
                        n = grid->getCell(x,y);
                        if(n->type == FREE){
                            n->type = NEAROBSTACLE;
                        }
                    }
                }
            }

        }
    }
}

void Potential::initializePotentials()
{
    Cell *c;
    for(int i=curPose.x-radius;i<=curPose.x+radius;i++){
        for(int j=curPose.y-radius;j<=curPose.y+radius;j++){
            c = grid->getCell(i,j);

            if(c->type == OCCUPIED || c->type == NEAROBSTACLE)
                c->pot = 1.0;
            else if(c->type == UNEXPLORED)
                c->pot = 0.0;

            c->pref = curPref;
        }
    }
}

void Potential::iteratePotentials()
{
    Cell* c;

    // the potential of a cell is stored in:
    // c->pot
    // the preference of a cell is stored in:
    // c->pref

    Cell *left,*right,*up,*down;

    // the update of a FREE cell in position (i,j) will use the potential of the four adjacent cells
    // where, for example:
    //     left  = grid->getCell(i-1,j);


    // TODO: iterate the potential field in the known map (defined by curLimits)
    //
    //  (curLimits.minX,curLimits.maxY)  -------  (curLimits.maxX,curLimits.maxY)
    //                     |                       \                         |
    //                     |                        \                        |
    //                     |                         \                       |
    //  (curLimits.minX,curLimits.minY)  -------  (curLimits.maxX,curLimits.minY)








}

void Potential::updateGradient()
{
    Cell* c;

    // the components of the descent gradient of a cell are stored in:
    // c->dirX and c->dirY

    Cell *left,*right,*up,*down;

    // the gradient of a FREE cell in position (i,j) is computed using the potential of the four adjacent cells
    // where, for example:
    //     left  = grid->getCell(i-1,j);


    // TODO: compute the gradient of the FREE cells in the known map (defined by curLimits)
    //
    //  (curLimits.minX,curLimits.maxY)  -------  (curLimits.maxX,curLimits.maxY)
    //                     |                       \                         |
    //                     |                        \                        |
    //                     |                         \                       |
    //  (curLimits.minX,curLimits.minY)  -------  (curLimits.maxX,curLimits.minY)











}
