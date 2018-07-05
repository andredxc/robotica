#include "MCL.h"

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

#include <GL/glut.h>

#define PI 3.1415

MCL::MCL(Pose initialPose, float maxRange, std::string mapName):
    curPose(initialPose), maxRange(maxRange)
{
    scale = 10;
    transparency = false;

    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = new std::default_random_engine(seed);

    // read map description
    readMap(mapName);

    // set the number of particles used by the filter
    numParticles = 10000;

    // initialize particles
    initParticles();
}

void MCL::run(const Action &u, const std::vector<float> &z)
{
    sampling(u);
    weighting(z);
    resampling();

    // updating robot pose (given by odometry)
    curPose.x += u.trans*cos(curPose.theta + u.rot1);
    curPose.y += u.trans*sin(curPose.theta + u.rot1);
    curPose.theta += u.rot1 + u.rot2;
    robotPath.push_back(curPose);
}

//////////////////////////////////////////////////
//// Métodos SAMPLING, WEIGHTING e RESAMPLING ////
//////////////////////////////////////////////////

void MCL::sampling(const Action &u)
{
    /// TODO: propagar todas as particulas de acordo com o modelo de movimento baseado em odometria
    int i;
    double varRot1, varTrans, varRot2;

    /// Odometria definida pela estrutura Action, composta por 3 variaveis double:
    /// rot1, trans e rot2
    std::cout << "rot1 " << RAD2DEG(u.rot1) << " trans " << u.trans << " rot2 " << RAD2DEG(u.rot2) << std::endl;

    std::normal_distribution<double> sampler1(0.0, u.rot1+u.trans);
    std::normal_distribution<double> sampler2(0.0, u.trans+u.rot1+u.rot2);
    std::normal_distribution<double> sampler3(0.0, u.rot2+u.trans);

    for(i = 0; i < particles.size(); i++){

        /// Seguindo o modelo de Thrun, devemos gerar 3 distribuicoes normais, uma para cada componente da odometria
        /// Para definir uma distribuição normal X de media M e variancia V, pode-se usar:
        varRot1 = u.rot1 - sampler1(*generator);
        varTrans = u.trans - sampler2(*generator);
        varRot2 = u.rot2 - sampler3(*generator);

        particles.at(i).p.x += varTrans*cos(particles.at(i).p.theta + varRot1);
        particles.at(i).p.y += varTrans*sin(particles.at(i).p.theta + varRot1);
        particles.at(i).p.theta += varRot1 + varRot2;
    }
}

void MCL::weighting(const std::vector<float> &z)
{
    /// TODO: faça a pesagem de todas as particulas
    int i, j;
    float zPart, zRobot, curProb;
    float var = 200;
    float mult, sum;

    sum = 0;
    for(i = 0; i < particles.size(); i++){
        // 1: elimine particulas fora do espaco livre e fora do espaço do mapa
        if(particles.at(i).p.x*scale < 0 || particles.at(i).p.x*scale > mapWidth ||
            particles.at(i).p.y*scale < 0 || particles.at(i).p.y*scale > mapHeight){

            particles.at(i).w = 0;
            continue;
        }

        if(mapCells[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)] != FREE){
            particles.at(i).w = 0;
            continue;
        }

        // 2: compare as observacoes da particula com as observacoes z do robo
        mult = 1.0;
        for(j = 0; j < 180; j+= 10){
            zPart = computeExpectedMeasurement(j, particles.at(i).p);
            zRobot = z[j];
            curProb = (1/sqrt(2*PI*var))*exp(-0.5*(pow(zRobot-zPart,2)/var));

            mult *= curProb;
        }
        particles.at(i).w = mult;

        // 3: normalize os pesos
        sum += particles.at(i).w;
    }

    if(sum == 0){
        sum = 1;
    }

    for(i = 0; i < particles.size(); i++){

        particles.at(i).w /= sum;
    }
}

void MCL::resampling()
{
    // gere uma nova geração de particulas com o mesmo tamanho do conjunto atual
    std::vector<MCLparticle> nextGeneration;
    float r, c, u;
    int i, j;


    r = rand()% (1/particles.size() + 1);
    c = particles.at(1).w;
    i = 0;
    for(j = 0; j < particles.size(); j++){
        u = r + (1/particles.size())*(j-1);
        while(u > c){
            i++;
            c += particles.at(i).w;
        }
        nextGeneration.push_back(particles.at(i));
    }

    particles = nextGeneration;

    /// Para gerar amostras de uma distribuição uniforme entre valores MIN e MAX, pode-se usar:
    // std::uniform_real_distribution<double> samplerU(MIN,MAX));
    /// Para gerar amostras segundo a distribuicao acima, usa-se:
    // double amostra = samplerU(*generator)
}

/////////////////////////////////////////////////////
//// Método Auxiliar para o Modelo de Observacao ////
/////////////////////////////////////////////////////

float MCL::computeExpectedMeasurement(int index, Pose &pose)
{
    double angle = pose.theta + double(90-index)*M_PI/180.0;

    // Ray-casting using DDA
    double dist;
    double difX=cos(angle);
    double difY=sin(angle);
    double deltaX, deltaY;

    if(tan(angle)==1 || tan(angle)==-1){
        deltaX=deltaY=1.0;
        dist = difX*maxRange;
    }else if(difX*difX > difY*difY){
        deltaX=1.0;
        deltaY=difY/difX;
        dist = difX*maxRange;
    }else{
        deltaX=difX/difY;
        deltaY=1.0;
        dist = difY*maxRange;
    }
    if(deltaX*difX < 0.0)
        deltaX = -deltaX;
    if(deltaY*difY < 0.0)
        deltaY = -deltaY;
    if(dist < 0.0)
        dist = -dist;

    dist *= scale;

    double i=pose.x*scale;
    double j=pose.y*scale;
    for(int k=0;k<(int)(dist);k++){

        if(mapCells[(int)i][(int)j] == OCCUPIED){
            // the real obstacle is one step ahead due to wall thickening
            return sqrt(pow(pose.x*scale-(i+deltaX),2)+pow(pose.y*scale-(j+deltaY),2))/scale;
        }

        i+=deltaX;
        j+=deltaY;
    }

    return maxRange;
}

//////////////////////////////////
//// Métodos de Inicializacao ////
//////////////////////////////////

void MCL::readMap(std::string mapName)
{
    std::string name("../phir2framework/DiscreteMaps/");
    name += mapName;
    std::ifstream file;
    file.open(name.c_str(), std::ifstream::in);

    if( !file.good() )
    {
        std::cerr << "The file \"" << name << "\"  does not exit!" << std::endl;
        return;
    }

    // Read dimensions.
    file >> mapWidth >> mapHeight;
    std::cout << "map.width " << mapWidth << " map.height " << mapHeight << std::endl;

    mapCells = new CellType*[mapWidth];
        for(int i=0;i<mapWidth;i++)
            mapCells[i] = new CellType[mapHeight];

    // Read grid from file.
    char read;
    for(int y=0; y < mapHeight; y++)
    {
        for(int x=0; x < mapWidth; x++)
        {
            file >> read;
            switch(read)
            {
                case '1':
                    mapCells[x][y] = OCCUPIED;
                    break;
                case '0':
                    mapCells[x][y] = FREE;
                    break;
                case '-':
                    mapCells[x][y] = UNEXPLORED;
                    break;
            }
        }
    }

    file.close();
}

void MCL::initParticles()
{
    particles.resize(numParticles);

    std::uniform_real_distribution<double> randomX(0.0,mapWidth/scale);
    std::uniform_real_distribution<double> randomY(0.0,mapHeight/scale);
    std::uniform_real_distribution<double> randomTh(-M_PI,M_PI);

    // generate initial set
    for(int i=0; i<numParticles; i++){

        bool valid = false;
        do{
            // sample particle pose
            particles[i].p.x = randomX(*generator);
            particles[i].p.y = randomY(*generator);
            particles[i].p.theta = randomTh(*generator);

            // check if particle is valid (known and not obstacle)
            if(mapCells[(int)(particles[i].p.x*scale)][(int)(particles[i].p.y*scale)] == FREE)
                valid = true;

        }while(!valid);

        std::cout << "Particle (" << i << "): "
                  << particles[i].p.x << ' '
                  << particles[i].p.y << ' '
                  << RAD2DEG(particles[i].p.theta) << std::endl;
    }
}

//////////////////////////////////////////
//// Método de desenho das particulas ////
//////////////////////////////////////////

void MCL::draw()
{
    // Draw map
    for(int x=0;x<mapWidth;x++){
        for(int y=0;y<mapHeight;y++){

            if(mapCells[x][y] == OCCUPIED)
                glColor3f(0.0,0.0,0.0);
            else if (mapCells[x][y] == UNEXPLORED)
                glColor3f(0.5,0.5,0.5);
            else
                glColor3f(1.0,1.0,1.0);

            glBegin( GL_QUADS );
            {
                glVertex2f(x  ,y  );
                glVertex2f(x+1,y  );
                glVertex2f(x+1,y+1);
                glVertex2f(x  ,y+1);
            }
            glEnd();
        }
    }

    double dirScale=5;
    glPointSize(4);
    glLineWidth(2);

    float alpha;
    if(transparency)
        alpha = 100.0/numParticles;
    else
        alpha = 1.0;

    // Draw particles
    for(int p=0;p<particles.size();p++){

        double x=particles[p].p.x*scale;
        double y=particles[p].p.y*scale;
        double th=particles[p].p.theta;

        // Draw point
        glColor4f(1.0,0.0,0.0,alpha);
        glBegin( GL_POINTS );
        {
            glVertex2f(x, y);
        }
        glEnd();

        // Draw direction
        glColor4f(0.0, 0.0, 1.0, alpha);
        glBegin( GL_LINES );
        {
            glVertex2f(x, y);
            glVertex2f(x+dirScale*cos(th), y+dirScale*sin(th));
        }
        glEnd();
    }
    glLineWidth(1);

    // Draw current robot pose
    double xRobot = curPose.x*scale;
    double yRobot = curPose.y*scale;
    double angRobot = RAD2DEG(curPose.theta);

    glTranslatef(xRobot,yRobot,0.0);
    glRotatef(angRobot,0.0,0.0,1.0);
    glScalef(1.0/5.0,1.0/5.0,1.0/5.0);

    glColor3f(0.0,1.0,0.0);
    glBegin( GL_POLYGON );
    {
        glVertex2f(-20, -8);
        glVertex2f(-13, -15);
        glVertex2f(8, -15);
        glVertex2f(15, -8);
        glVertex2f(15, 8);
        glVertex2f(8, 15);
        glVertex2f(-13, 15);
        glVertex2f(-20, 8);
    }
    glEnd();
    glColor3f(0.0,0.0,0.0);
    glBegin( GL_LINE_STRIP );
    {
        glVertex2f(0, 0);
        glVertex2f(30, 0);
    }
    glEnd();

    glScalef(5,5,5);
    glRotatef(-angRobot,0.0,0.0,1.0);
    glTranslatef(-xRobot,-yRobot,0.0);

    // Draw robot path
    if(robotPath.size() > 1){
        glScalef(scale,scale,scale);
        glColor3f(0.0,0.6,0.0);
        glLineWidth(3);
        glBegin( GL_LINE_STRIP );
        {
            for(unsigned int i=0;i<robotPath.size()-1; i++){
                glVertex2f(robotPath[i].x, robotPath[i].y);
                glVertex2f(robotPath[i+1].x, robotPath[i+1].y);
            }
        }
        glEnd();
        glLineWidth(1);
        glScalef(1.0/scale,1.0/scale,1.0/scale);

    }
}
