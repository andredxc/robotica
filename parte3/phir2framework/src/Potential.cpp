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
    Cell *c;
    int x, y;

    // Limites usados quando a célula se encontra como UNEXPLORED
    float initLimitOccupied = 0.6;
    float initLimitFree = 0.4;
    // Limites usados depois de a célula deixar de ser UNEXPLORED
    float limitOccupied = 0.6;
    float limitFree = 0.5;

    for(x = (curPose.x-radius); x < (curPose.x+radius); x++){
        for(y = (curPose.y-radius); y < (curPose.y+radius); y++){
            // Percorre todas as celulas envolta do robô
            c = grid->getCell(x,y);
            if(c->type == UNEXPLORED){
                if(c->occupancy >= initLimitOccupied){
                    c->type = OCCUPIED;
                }
                else if(c->occupancy <= initLimitFree){
                    c->type = FREE;
                }
            }
            else{
                if(c->type == OCCUPIED && c->occupancy <= limitFree){
                    c->type = FREE;
                }
                else if(c->type == FREE && c->occupancy >= limitOccupied){
                    c->type = OCCUPIED;
                }
            }
        }
    }
}

void Potential::expandObstacles()
{
    int width=3;
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
    int x, y;
    float h, d;
    Cell *left,*right,*up,*down;

    for(x = curLimits.minX; x < curLimits.maxX; x++){
        for(y = curLimits.minY; y < curLimits.maxY; y++){

            c = grid->getCell(x,y);
            if(c->type == FREE){
                left = grid->getCell(x-1, y);
                right = grid->getCell(x+1, y);
                up = grid->getCell(x, y+1);
                down = grid->getCell(x, y-1);

                h = (left->pot + right->pot + up->pot + down->pot)/4;
                d = abs(up->pot - down->pot)/2 + abs(right->pot - left->pot)/2;
                c->pot = h - (c->pref/4)*d;
            }
        }
    }
}

void Potential::updateGradient()
{
    Cell* c;
    int x, y;
    double norm;
    Cell *left,*right,*up,*down;

    for(x = curLimits.minX; x < curLimits.maxX; x++){
        for(y = curLimits.minY; y < curLimits.maxY; y++){

            c = grid->getCell(x,y);
            if(c->type == FREE){
                left = grid->getCell(x-1, y);
                right = grid->getCell(x+1, y);
                up = grid->getCell(x, y+1);
                down = grid->getCell(x, y-1);

                c->dirY = -(up->pot - down->pot)/2;
                c->dirX = -(right->pot - left->pot)/2;
                norm = sqrt(pow(c->dirX,2) + pow(c->dirY,2));
                c->dirX /= norm;
                c->dirY /= norm;
            }
            else{
                c->dirX = 0;
                c->dirY = 0;
            }
        }
    }
}
