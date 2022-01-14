#include <iostream>
#include <string>
#include <math.h>
#include <random>
#include <vector>

#include <cstdlib>
#include <unistd.h>

#include <chrono>
#include <thread>

#include <opencv2/opencv.hpp>

namespace TDMineSweeper{
class Physics{
protected:
    double x;
    double y;
    double z;
public:
    Physics(double xx,double yy,double zz){
        x=xx;
        y=yy;
        z=zz;
    }
    ~Physics(){
    }

    double getx(){
        return x;
    }

    double gety(){
        return y;
    }

    double getz(){
        return z;
    }

    void setx(double xx){
        x=xx;
    }

    void sety(double yy){
        y=yy;
    }

    void setz(double zz){
        z=zz;
    }

    void rotation(double rotZ,double rotY){
        double dx,dy,dz;
        dx=x*cos(rotZ)-y*sin(rotZ);
        dy=x*sin(rotZ)+y*cos(rotZ);
        x=dx;
        y=dy;
        
        rotY=-rotY;

        dz=z*cos(rotY)-x*sin(rotY);
        dx=z*sin(rotY)+x*cos(rotY);
        z=dz;
        x=dx;
    }
};

class Controller : public Physics{
private:
    std::int8_t ex;
    std::int8_t ey;
    std::int8_t ez;
public:
    Controller(double xx,double yy,double zz,std::int8_t exx,std::int8_t eyy,std::int8_t ezz):Physics(xx,yy,zz)
    {
        x=xx;
        y=yy;
        z=zz;
        ex=exx;
        ey=eyy;
        ez=ezz;
    }

    ~Controller(){
    }

    std::int8_t getex(){
        return ex;
    }

    std::int8_t getey(){
        return ey;
    }

    std::int8_t getez(){
        return ez;
    }
};

class Cell : public Physics{
private:
    cv::Scalar color;
    std::string label;
    std::int8_t vx;
    std::int8_t vy;
    std::int8_t vz;
public:   
    Cell(double xx,double yy,double zz,cv::Scalar cc,std::string ll,std::int8_t exx,std::int8_t eyy,std::int8_t ezz):Physics(xx,yy,zz){
        x=xx;
        y=yy;
        z=zz;
        color=cc;
        label=ll;
        vx=exx;
        vy=eyy;
        vz=ezz;
    }

    ~Cell(){
    }

    cv::Scalar getcolor(){
        return color;
    }

    std::string getlabel(){
        return label;
    }

    std::int8_t getvx(){
        return vx;
    }

    std::int8_t getvy(){
        return vy;
    }

    std::int8_t getvz(){
        return vz;
    }

    void setcolor(cv::Scalar cc){
        color=cc;
    }

    void setlabel(std::string ll){
        label=ll;
    }
};

class MineSweeper{
private:
    std::int16_t CANVAS_WIDTH;
    std::int16_t CANVAS_HEIGHT;

    std::int16_t mouseDownX;
    std::int16_t mouseDownY;
    std::int16_t mouseEscapeX;
    std::int16_t mouseEscapeY;
    std::int16_t mouseUpdateX;
    std::int16_t mouseUpdateY;
    std::int16_t mouseUpX;
    std::int16_t mouseUpY;
    bool mouseDownFlag;
    bool leftClickFlag;
    bool longPressFlag;
    bool initFlag;

    bool gameover;
    bool gameclear;

    std::int8_t boardX;
    std::int8_t boardY;
    std::int8_t boardZ;
    std::int8_t mines;

    std::int8_t interval;
    std::int8_t cellsize;

    std::vector<std::vector<std::vector<bool>>> danger;
    std::vector<std::vector<std::vector<bool>>> demined;
    std::vector<std::vector<std::vector<std::int8_t>>> visual;
    std::vector<std::int8_t> cursor;

    std::vector<Controller> controllerlist;
    std::vector<Cell> mainlist;

    cv::Mat canvas;
    
    std::default_random_engine generator;
    
    std::random_device rd;

    std::int16_t time;
        
public:
    MineSweeper(){
        CANVAS_WIDTH=800;
        CANVAS_HEIGHT=400;

        mouseDownX=0;
        mouseDownY=0;
        mouseEscapeX=0;
        mouseEscapeY=0;
        mouseUpdateX=0;
        mouseUpdateY=0;
        mouseUpX=0;
        mouseUpY=0;
        mouseDownFlag=false;
        leftClickFlag=false;
        longPressFlag=false;
        initFlag=false;

        gameover=false;
        gameclear=false;

        boardX=6;
        boardY=6;
        boardZ=6;
        mines=10;

        interval=CANVAS_HEIGHT/10;
        cellsize=interval/2.2;

        cursor={3,3,3};

        time=0;
    }
    ~MineSweeper(){
        delete &danger;
        delete &demined;
        delete &visual;
        delete &cursor;

        delete &controllerlist;
        delete &mainlist;

        canvas.release();
    
    }
    void setCANVAS_WIDTH(std::int16_t xx){
        CANVAS_WIDTH=xx;
    }
    void setCANVAS_HEIGHT(std::int16_t yy){
        CANVAS_HEIGHT=yy;
    }
    void setmouseDownX(std::int16_t xx){
        mouseDownX=xx;
    }
    void setmouseDownY(std::int16_t yy){
        mouseDownY=yy;
    }
    void setmouseEscapeX(std::int16_t xx){
        mouseEscapeX=xx;
    }
    void setmouseEscapeY(std::int16_t yy){
        mouseEscapeY=yy;
    }
    void setmouseUpdateX(std::int16_t xx){
        mouseUpdateX=xx;
    }
    void setmouseUpdateY(std::int16_t yy){
        mouseUpdateY=yy;
    }
    void setmouseUpX(std::int16_t xx){
        mouseUpX=xx;
    }
    void setmouseUpY(std::int16_t yy){
        mouseUpY=yy;
    }
    void setmouseDownFlag(bool ff){
        mouseDownFlag=ff;
    }
    void setleftClickFlag(bool ff){
        leftClickFlag=ff;
    }
    void setlongPressFlag(bool ff){
        longPressFlag=ff;
    }
    void setinitFlag(bool ff){
        initFlag=ff;
    }
    void setgameover(bool gg){
        gameover=gg;
    }
    void setgameclear(bool gg){
        gameclear=gg;
    }
    void setboardX(std::int8_t xx){
        boardX=xx;
    }
    void setboardY(std::int8_t yy){
        boardY=yy;
    }
    void setboardZ(std::int8_t zz){
        boardZ=zz;
    }
    void setmines(std::int8_t mm){
        mines=mm;
    }
    void setinterval(std::int8_t ii){
        interval=ii;
    }
    void setcellsize(std::int8_t cc){
        cellsize=cc;
    }
    void setdangerAll(std::vector<std::vector<std::vector<bool>>> dd){
        for(int i=0;i<boardX;i++){
            for(int j=0;j<boardY;j++){
                for(int k=0;k<boardZ;k++){
                    danger[i][j][k]=dd[i][j][k];
                }
            }
        }
    }
    void setdangerAllClear(){
        danger.clear();
        
        danger.resize(boardX);
        for(int i=0;i<boardX;i++){
            danger[i].resize(boardY);
            for(int j=0;j<boardY;j++){
                danger[i][j].resize(boardZ);
            }
        }
    }
    void setdanger(std::int8_t x,std::int8_t y,std::int8_t z,bool dd){
        danger[x][y][z]=dd;
    }
    void setdeminedAll(std::vector<std::vector<std::vector<bool>>> dd){
        for(int i=0;i<boardX;i++){
            for(int j=0;j<boardY;j++){
                for(int k=0;k<boardZ;k++){
                    demined[i][j][k]=dd[i][j][k];
                }
            }
        }
    }
    void setdeminedAllClear(){
        demined.clear();
        
        demined.resize(boardX);
        for(int i=0;i<boardX;i++){
            demined[i].resize(boardY);
            for(int j=0;j<boardY;j++){
                demined[i][j].resize(boardZ);
            }
        }
    }
    void setdemined(std::int8_t x,std::int8_t y,std::int8_t z,bool dd){
        demined[x][y][z]=dd;
    }
    void setvisualAll(std::vector<std::vector<std::vector<std::int8_t>>> vv){
        for(int i=0;i<boardX;i++){
            for(int j=0;j<boardY;j++){
                for(int k=0;k<boardZ;k++){
                    visual[i][j][k]=vv[i][j][k];
                }
            }
        }
    }
    void setvisualAllClear(){
        visual.clear();
        
        visual.resize(boardX);
        for(int i=0;i<boardX;i++){
            visual[i].resize(boardY);
            for(int j=0;j<boardY;j++){
                visual[i][j].resize(boardZ);
            }
        }

    }
    void setvisual(std::int8_t x,std::int8_t y,std::int8_t z,std::int8_t vv){
        visual[x][y][z]=vv;
    }
    void setcursor(std::int8_t i,std::int8_t v){
	    cursor[i]=v;
    }
    void setcontrollerlist(std::vector<Controller> cc){
        for(int i=0;i<cc.size();i++){
            controllerlist.push_back(cc[i]);
        }
    }
    void setcontrollerlistClear(){
        controllerlist.clear();
    }
    void setmainlist(std::vector<Cell> mm){
        for(int i=0;i<mm.size();i++){
            mainlist.push_back(mm[i]);
        }
    }
    void setmainlistClear(){
        mainlist.clear();
    }
    void setcanvas(cv::Mat cc){
        canvas=cc;
    }
    void settime(std::int16_t tt){
        time=tt;
    }
    std::int16_t getCANVAS_WIDTH(){
        return CANVAS_WIDTH;
    }
    std::int16_t getCANVAS_HEIGHT(){
        return CANVAS_HEIGHT;
    }
    std::int16_t getmouseDownX(){
        return mouseDownX;
    }
    std::int16_t getmouseDownY(){
        return mouseDownY;
    }
    std::int16_t getmouseEscapeX(){
        return mouseEscapeX;
    }
    std::int16_t getmouseEscapeY(){
        return mouseEscapeY;
    }
    std::int16_t getmouseUpdateX(){
        return mouseUpdateX;
    }
    std::int16_t getmouseUpdateY(){
        return mouseUpdateY;
    }
    std::int16_t getmouseUpX(){
        return mouseUpX;
    }
    std::int16_t getmouseUpY(){
        return mouseUpY;
    }
    bool getmouseDownFlag(){
        return mouseDownFlag;
    }
    bool getleftClickFlag(){
        return leftClickFlag;
    }
    bool getlongPressFlag(){
        return longPressFlag;
    }
    bool getinitFlag(){
        return initFlag;
    }
    bool getgameover(){
        return gameover;
    }
    bool getgameclear(){
        return gameclear;
    }
    std::int8_t getboardX(){
        return boardX;
    }
    std::int8_t getboardY(){
        return boardY;
    }
    std::int8_t getboardZ(){
        return boardZ;
    }
    std::int8_t getmines(){
        return mines;
    }
    std::int8_t getinterval(){
        return interval;
    }
    std::int8_t getcellsize(){
        return cellsize;
    }
    std::vector<std::vector<std::vector<bool>>> getdangerAll(){
        return danger;
    }
    bool getdanger(std::int8_t x,std::int8_t y,std::int8_t z){
        return danger[x][y][z];
    }
    std::vector<std::vector<std::vector<bool>>> getdeminedAll(){
        return demined;
    }
    bool getdemined(std::int8_t x,std::int8_t y,std::int8_t z){
        return demined[x][y][z];
    }
    std::vector<std::vector<std::vector<std::int8_t>>> getvisualAll(){
        return visual;
    }
    std::int8_t getvisual(std::int8_t x,std::int8_t y,std::int8_t z){
        return visual[x][y][z];
    }
    std::vector<std::int8_t> getcursorAll(){
        return cursor;
    }
    std::int8_t getcursor(std::int8_t i){
        return cursor[i];
    }
    std::vector<Controller> getcontrollerlistAll(){
        return controllerlist;
    }
    Controller getcontrollerlist(std::int8_t i){
        return controllerlist[i];
    }
    Controller * getcontrollerlistPtr(std::int8_t i){
        return &controllerlist[i];
    }
    std::vector<Cell> getmainlistAll(){
        return mainlist;
    }
    Cell getmainlist(std::int16_t i){
        return mainlist[i];
    }
    Cell * getmainlistPtr(std::int16_t i){
        return &mainlist[i];
    }
    cv::Mat getcanvas(){
        return canvas;
    }
    std::int16_t random(){
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> distrib(0,boardX*boardY*boardZ-1);
        return distrib(gen);
    }
    std::int16_t gettime(){
        return time;
    }
};

MineSweeper MS;

void timeCounter();
std::int8_t flagCounter();
void generateLists();
void modeChange(std::int8_t);
cv::Scalar cellColor(unsigned short);
void gameClearJudge();
void gameInitialize();
void boardInitialize();
void dangerInitialize();
void safechain(std::int8_t,std::int8_t,std::int8_t);
void safechain2(std::int8_t,std::int8_t,std::int8_t,std::int8_t,std::int8_t,std::int8_t);
bool safe(std::int8_t,std::int8_t,std::int8_t);
bool safe2(std::int8_t,std::int8_t,std::int8_t,std::int8_t,std::int8_t,std::int8_t);
std::int8_t count(std::int8_t,std::int8_t,std::int8_t);
std::int8_t count2(std::int8_t,std::int8_t,std::int8_t,std::int8_t,std::int8_t,std::int8_t);
void gameDisplay();

void timeCounter(){
    while(true){
        while(MS.getinitFlag()){
            sleep(1);
            while(MS.getinitFlag()){
                MS.settime(MS.gettime()+1);
                sleep(1);
            }
        }
    }
    return;
}
std::int8_t flagCounter(){
    std::int8_t x=0;
    for(int i=0;i<MS.getboardX();i++){
        for(int j=0;j<MS.getboardY();j++){
            for(int k=0;k<MS.getboardZ();k++){
                if(MS.getvisual(i,j,k)==-1&&!MS.getdemined(i,j,k)){
                    x+=1;
                }
            }
        }
    }
    return x;
}

void generateLists(){
    MS.setdangerAllClear();
    MS.setdeminedAllClear();
    MS.setvisualAllClear();
    
    std::vector<std::vector<std::vector<bool>>> ddanger;
    std::vector<std::vector<std::vector<bool>>> ddemined;
    std::vector<std::vector<std::vector<std::int8_t>>> vvisual;

    ddanger.clear();
    ddemined.clear();
    vvisual.clear();

    ddanger.resize(MS.getboardX());
    ddemined.resize(MS.getboardX());
    vvisual.resize(MS.getboardX());

    for(int i=0;i<MS.getboardX();i++){
        ddanger[i].resize(MS.getboardY());
        ddemined[i].resize(MS.getboardY());
        vvisual[i].resize(MS.getboardY());
    }

    for(int i=0;i<MS.getboardX();i++){
        for(int j=0;j<MS.getboardY();j++){
            ddanger[i][j].resize(MS.getboardZ());
            ddemined[i][j].resize(MS.getboardZ());
            vvisual[i][j].resize(MS.getboardZ());
        }
    }

    for(int i=0;i<MS.getboardX();i++){
        for(int j=0;j<MS.getboardY();j++){
            for(int k=0;k<MS.getboardZ();k++){
                ddanger[i][j][k]=false;
                ddemined[i][j][k]=false;
                vvisual[i][j][k]=0;
            }
        }
    }

    MS.setdangerAll(ddanger);
    MS.setdeminedAll(ddemined);
    MS.setvisualAll(vvisual);
}


void modeChange(std::int8_t m){
    switch(m){
        case 0://easy
            MS.setboardX(6);
            MS.setboardY(6);
            MS.setboardZ(6);
            MS.setmines(10);
            break;
        case 1://normal
            MS.setboardX(8);
            MS.setboardY(8);
            MS.setboardZ(8);
            MS.setmines(40);
            break;
        case 2://hard
            MS.setboardX(10);
            MS.setboardY(10);
            MS.setboardZ(10);
            MS.setmines(99);
            break;
        default:
            break;
    }
    
    gameInitialize();
}

cv::Scalar cellColor(unsigned short x){
        unsigned short R=0,G=0,B=0;
        
        if(x%3==0){
            R=0x11;
        }else if(x%3==1){
            R=0x55;
        }else if(x%3==2){
            R=0x99;
        }
        
        float var_g=x/2;
        unsigned short gg=(unsigned short)var_g;

        if(gg%3==0){
            G=0x44;
        }else if(gg%3==1){
            G=0x88;
        }else if(gg%3==2){
            G=0xcc;
        }

        float var_h=x/3;
        unsigned short hh=(unsigned short)var_h;
        if(hh%3==0){
            B=0x77;
        }else if(hh%3==1){
            B=0xbb;
        }else if(hh%3==2){
            B=0xff;
        }

        return cv::Scalar(B,G,R);
}

void gameClearJudge(){
    unsigned short counter=0;
    for(int i=0;i<MS.getboardX();i++){
        for(int j=0;j<MS.getboardY();j++){
            for(int k=0;k<MS.getboardZ();k++){
                if(MS.getdemined(i,j,k)){
                    counter++;
                }
            }
        }
    }

    if(counter==MS.getboardX()*MS.getboardY()*MS.getboardZ()-MS.getmines()){
        MS.setgameclear(true);
        MS.setgameover(true);
        MS.setinitFlag(false);
    }

    if(MS.getgameclear()){
        std::cout<<"game clear"<<std::endl;
    }
}

void gameInitialize(){

    MS.setgameover(false);
    MS.setgameclear(false);
    MS.settime(0);
    MS.setinitFlag(false);

    MS.setmouseDownX(0);
    MS.setmouseDownY(0);
    MS.setmouseUpdateX(0);
    MS.setmouseUpdateY(0);
    MS.setmouseUpX(0);
    MS.setmouseUpY(0);

    MS.setcursor(0,(int)(MS.getboardX()/2));
    MS.setcursor(1,(int)(MS.getboardY()/2));
    MS.setcursor(2,(int)(MS.getboardZ()/2));

    generateLists();
    
    dangerInitialize();

    //------------------------------------
    MS.setcontrollerlistClear();

    std::vector<Controller> clist;
    clist.clear();

    for(int i=0;i<2;i++){
        std::int8_t var_i=2*i-1;
        double var_l=MS.getCANVAS_HEIGHT()/4.0*var_i;
        clist.push_back(Controller(var_l,0.0,0.0,var_i,0,0));
        clist.push_back(Controller(0.0,var_l,0.0,0,var_i,0));
        clist.push_back(Controller(0.0,0.0,var_l,0,0,var_i));
    }

    MS.setcontrollerlist(clist);

    //-------------------------------------
    MS.setmainlistClear();

    std::vector<Cell> mlist;
    mlist.clear();

    for(int i=0;i<MS.getboardX();i++){
        for(int j=0;j<MS.getboardY();j++){
            for(int k=0;k<MS.getboardZ();k++){
                double x,y,z;
                x=(2*i-(MS.getboardX()-1))*MS.getinterval()/2.0;
                y=(2*j-(MS.getboardY()-1))*MS.getinterval()/2.0;
                z=(2*k-(MS.getboardZ()-1))*MS.getinterval()/2.0;
                mlist.push_back(Cell(x,y,z,cv::Scalar(150,150,150),"h",i,j,k));
            }
        }
    }

    MS.setmainlist(mlist);

    gameDisplay();
}

void boardInitialize(){
    for(int i=0;i<MS.getboardX();i++){
        for(int j=0;j<MS.getboardY();j++){
            for(int k=0;k<MS.getboardZ();k++){
                MS.setdanger(i,j,k,false);
                MS.setdemined(i,j,k,false);
                MS.setvisual(i,j,k,0);
            }
        }
    }
}
void dangerInitialize(){
    std::vector<std::int16_t> mineIndex;
    bool newIntFlag=true;
            
    mineIndex.clear();
    while(mineIndex.size()<MS.getmines()){
        int rand=MS.random();
        newIntFlag=true;
        for(int i=0;i<mineIndex.size();i++){
            if(mineIndex[i]==rand){
                newIntFlag=false;
                break;
            }
        }
        if(newIntFlag){
            mineIndex.push_back(rand);
        }
    }

    for(int i=0;i<MS.getmines();i++){
        std::int8_t x=(mineIndex[i]/(MS.getboardY()*MS.getboardZ()))%MS.getboardX();
        std::int8_t y=(mineIndex[i]/MS.getboardZ())%MS.getboardY();
        std::int8_t z=mineIndex[i]%MS.getboardZ();
        MS.setdanger(x,y,z,true);
    }

}

void safechain(std::int8_t x,std::int8_t y,std::int8_t z){
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            for(int k=0;k<3;k++){
                if(!(i==0&&j==0&&k==0)){
                    safechain2(x,y,z,i-1,j-1,k-1);
                }
            }

        }
    }
}

void safechain2(std::int8_t x,std::int8_t y,std::int8_t z,std::int8_t dx,std::int8_t dy,std::int8_t dz){
    if(x>=0&&x<MS.getboardX()
    &&y>=0&&y<MS.getboardY()
    &&z>=0&&z<MS.getboardZ()
    &&x+dx>=0&&x+dx<MS.getboardX()
    &&y+dy>=0&&y+dy<MS.getboardY()
    &&z+dz>=0&&z+dz<MS.getboardZ()
    &&!MS.getdemined(x+dx,y+dy,z+dz)){
        if(safe(x+dx,y+dy,z+dz)){
            MS.setvisual(x+dx,y+dy,z+dz,0);
            MS.setdemined(x+dx,y+dy,z+dz,true);
            (*(MS.getmainlistPtr((x+dx)*MS.getboardY()*MS.getboardZ()+(y+dy)*MS.getboardZ()+(z+dz)))).setcolor(cellColor(MS.getvisual(x+dx,y+dy,z+dz)));
            (*(MS.getmainlistPtr((x+dx)*MS.getboardY()*MS.getboardZ()+(y+dy)*MS.getboardZ()+(z+dz)))).setlabel(std::to_string(MS.getvisual(x+dx,y+dy,z+dz)));
            safechain(x+dx,y+dy,z+dz);
        }else{
            MS.setvisual(x+dx,y+dy,z+dz,count(x+dx,y+dy,z+dz));
            MS.setdemined(x+dx,y+dy,z+dz,true);
            (*(MS.getmainlistPtr((x+dx)*MS.getboardY()*MS.getboardZ()+(y+dy)*MS.getboardZ()+(z+dz)))).setcolor(cellColor(MS.getvisual(x+dx,y+dy,z+dz)));
            (*(MS.getmainlistPtr((x+dx)*MS.getboardY()*MS.getboardZ()+(y+dy)*MS.getboardZ()+(z+dz)))).setlabel(std::to_string(MS.getvisual(x+dx,y+dy,z+dz)));
            return;
        }
    }else{
        return;
    }
}

bool safe(std::int8_t x,std::int8_t y,std::int8_t z){
    bool result=true;
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            for(int k=0;k<3;k++){
                if(!(i==1&&j==1&&k==1)){
                    result=result&&safe2(x,y,z,i-1,j-1,k-1);
                }
            }
        }
    }
    return result;
}

bool safe2(std::int8_t x,std::int8_t y,std::int8_t z,std::int8_t dx,std::int8_t dy,std::int8_t dz){
    if(x+dx<0||x+dx>MS.getboardX()-1
    ||y+dy<0||y+dy>MS.getboardY()-1
    ||z+dz<0||z+dz>MS.getboardZ()-1){
        return true;
    }else{
        if(MS.getdanger(x+dx,y+dy,z+dz)){
            return false;
        }else{
            return true;
        }
    }
}

std::int8_t count(std::int8_t x,std::int8_t y,std::int8_t z){
    std::int8_t sum=0;
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            for(int k=0;k<3;k++){
                if(!(i==1&&j==1&&k==1)){
                    sum=sum+count2(x,y,z,i-1,j-1,k-1);
                }
            }
        }
    }
    return sum;
}

std::int8_t count2(std::int8_t x,std::int8_t y,std::int8_t z,std::int8_t dx,std::int8_t dy,std::int8_t dz){
    if(x+dx<0||x+dx>MS.getboardX()-1
    ||y+dy<0||y+dy>MS.getboardY()-1
    ||z+dz<0||z+dz>MS.getboardZ()-1){
        return 0;
    }else{
        return MS.getdanger(x+dx,y+dy,z+dz)?1:0;
    }
}

void mouse_callback(int event,int x,int y,int flags,void *userdata)
{
    if(event==cv::EVENT_LBUTTONDOWN){
        MS.setmouseDownX(x);
        MS.setmouseDownY(y);
        MS.setmouseEscapeX(x);
        MS.setmouseEscapeY(y);
        MS.setmouseDownFlag(true);
        MS.setleftClickFlag(true);
        MS.setlongPressFlag(false);
     }
    if(event==cv::EVENT_LBUTTONUP){
        MS.setmouseDownFlag(false);
        if(!MS.getlongPressFlag()){
            unsigned short r,g,b;
            r=MS.getcanvas().at<uchar>(y,3*x);
            g=MS.getcanvas().at<uchar>(y,3*x+1);
            b=MS.getcanvas().at<uchar>(y,3*x+2);
           if(r==0xff&&g==0xff&&b==0xff){
                std::int8_t cx,cy,cz;
                cx=MS.getcursor(0);
                cy=MS.getcursor(1);
                cz=MS.getcursor(2);
                if(MS.getvisual(cx,cy,cz)!=-1){
                    MS.setdemined(cx,cy,cz,true);
                    if(MS.getdanger(cx,cy,cz)){//bomb
                        (*(MS.getmainlistPtr(cx*MS.getboardY()*MS.getboardZ()+cy*MS.getboardZ()+cz))).setlabel("b");
                        MS.setgameover(true);
                        MS.setinitFlag(false);
                    }else if(safe(cx,cy,cz)){//no mine around
                        MS.setvisual(cx,cy,cz,0);
                        (*(MS.getmainlistPtr(cx*MS.getboardY()*MS.getboardZ()+cy*MS.getboardZ()+cz))).setlabel("");
                        (*(MS.getmainlistPtr(cx*MS.getboardY()*MS.getboardZ()+cy*MS.getboardZ()+cz))).setcolor(cellColor(MS.getvisual(cx,cy,cz)));
                        safechain(cx,cy,cz);
                    }else{//some mine around
                        MS.setvisual(cx,cy,cz,count(cx,cy,cz));
                        (*(MS.getmainlistPtr(cx*MS.getboardY()*MS.getboardZ()+cy*MS.getboardZ()+cz))).setlabel(std::to_string(MS.getvisual(cx,cy,cz)));
                        (*(MS.getmainlistPtr(cx*MS.getboardY()*MS.getboardZ()+cy*MS.getboardZ()+cz))).setcolor(cellColor(MS.getvisual(cx,cy,cz)));
                    }
                    
                    if(!MS.getgameover()){
                        MS.setinitFlag(true);
                    }                    
                }
            }else{
                if(r==0xff){
                    MS.setcursor(0,(MS.getcursor(0)+1)%MS.getboardX());
                }
                if(r==0x00){
                    MS.setcursor(0,(MS.getcursor(0)-1)%MS.getboardX());
                    while(MS.getcursor(0)<0)MS.setcursor(0,(MS.getcursor(0)+MS.getboardX()));
                }
                if(g==0xff){
                    MS.setcursor(1,(MS.getcursor(1)+1)%MS.getboardY());
                }
                if(g==0x00){
                    MS.setcursor(1,(MS.getcursor(1)-1)%MS.getboardY());
                    while(MS.getcursor(1)<0)MS.setcursor(1,(MS.getcursor(1)+MS.getboardY()));
                }
                if(b==0xff){
                    MS.setcursor(2,(MS.getcursor(2)+1)%MS.getboardZ());
                }
                if(b==0x00){
                    MS.setcursor(2,(MS.getcursor(2)-1)%MS.getboardZ());
                    while(MS.getcursor(2)<0)MS.setcursor(2,(MS.getcursor(2)+MS.getboardZ()));
                }
            } 
        }
    }
    if(event==cv::EVENT_RBUTTONDOWN){
        MS.setmouseDownFlag(true);
        MS.setleftClickFlag(true);
        MS.setlongPressFlag(false);
    }
    if(event==cv::EVENT_RBUTTONUP){
        MS.setmouseDownFlag(false);
        if(!MS.getlongPressFlag()){
            std::int8_t cx,cy,cz;
            cx=MS.getcursor(0);
            cy=MS.getcursor(1);
            cz=MS.getcursor(2);
            if(MS.getvisual(cx,cy,cz)==-1){
                MS.setvisual(cx,cy,cz,0);
                (*(MS.getmainlistPtr(cx*MS.getboardY()*MS.getboardZ()+cy*MS.getboardZ()+cz))).setlabel("");
            }else if(MS.getvisual(cx,cy,cz)==0&&!MS.getdemined(cx,cy,cz)){
                MS.setvisual(cx,cy,cz,-1);
                (*(MS.getmainlistPtr(cx*MS.getboardY()*MS.getboardZ()+cy*MS.getboardZ()+cz))).setlabel("f");
            }
        }
        MS.setlongPressFlag(false);
        if(MS.getgameover()){
            std::cout<<">>>game over<<<"<<std::endl;   
        }
    }
    if(event==cv::EVENT_MOUSEMOVE){
        MS.setmouseUpdateX(x);
        MS.setmouseUpdateY(y);
        
        double xd,yd,ld;
        xd=x-MS.getmouseDownX();
        yd=y-MS.getmouseDownY();
        ld=sqrt(xd*xd+yd*yd);
        if(ld>10){
            MS.setlongPressFlag(true);
        }
        if(MS.getmouseDownFlag()&&MS.getlongPressFlag()){
            double rotZ,rotY;
            rotZ=(MS.getmouseUpdateX()-MS.getmouseEscapeX())/300.0;
            rotY=(MS.getmouseUpdateY()-MS.getmouseEscapeY())/300.0;
            for(int i=0;i<MS.getboardX()*MS.getboardY()*MS.getboardZ();i++){
                (*(MS.getmainlistPtr(i))).rotation(rotZ,rotY);
            }
            for(int i=0;i<6;i++){
                (*(MS.getcontrollerlistPtr(i))).rotation(rotZ,rotY);
            }
        }
        MS.setmouseEscapeX(x);
        MS.setmouseEscapeY(y);
    }
}

void gameDisplay(){
while(1){
    
    cv::Mat img(cv::Size(MS.getCANVAS_WIDTH(),2*MS.getCANVAS_HEIGHT()),CV_8UC3,cv::Scalar(0,0,0));
    
    putText(img,"R:New Game",cv::Point(10,40),1,1.2,cv::Scalar(0xfe,0xfe,0xfe));
    putText(img,"E:Easy ",cv::Point(10,70),1,1.2,cv::Scalar(0xfe,0xfe,0xfe));
    putText(img,"N:Normal",cv::Point(10,100),1,1.2,cv::Scalar(0xfe,0xfe,0xfe)); 
    putText(img,"H:Hard",cv::Point(10,130),1,1.2,cv::Scalar(0xfe,0xfe,0xfe));
    putText(img,"Flag : "+std::to_string(flagCounter()),cv::Point(10,200),1,1.2,cv::Scalar(0xfe,0xfe,0xfe));
    putText(img,"Time : "+std::to_string(MS.gettime()),cv::Point(10,230),1,1.2,cv::Scalar(0xfe,0xfe,0xfe));
    
    std::vector<Cell> sortlist;
    for(int i=0;i<MS.getmainlistAll().size();i++){
        sortlist.push_back((*(MS.getmainlistPtr(i))));
    }
    
    for(int i=0;i<MS.getboardX()*MS.getboardY()*MS.getboardZ();i++){
        for(int j=i+1;j<MS.getboardX()*MS.getboardY()*MS.getboardZ();j++){
            if(sortlist[i].getx()>sortlist[j].getx()){
                Cell t=sortlist[i];
                sortlist[i]=sortlist[j];
                sortlist[j]=t;
            }
        }
    }

    for(int i=0;i<MS.getboardX()*MS.getboardY()*MS.getboardZ();i++){
	    double fy,fz,fr;
        std::int16_t oy,oz;
        std::int8_t size;
        fy=sortlist[i].gety();
        fz=sortlist[i].getz();
        fr=1+sortlist[i].getx()/300;
        oy=MS.getCANVAS_WIDTH()/2;
        oz=MS.getCANVAS_HEIGHT()/2;
        size=MS.getcellsize();
        if(fr>0.0&&fr<2.0
        &&fy*fr+oy>size&&fy*fr+oy<MS.getCANVAS_WIDTH()-size
        &&fz*fr+oz>size&&fz*fr+oz<MS.getCANVAS_HEIGHT()-size){
            std::int8_t vx,vy,vz;
            cv::Scalar cellcolor;
            bool onCursor;

            vx=sortlist[i].getvx();
            vy=sortlist[i].getvy();
            vz=sortlist[i].getvz();
            cellcolor=sortlist[i].getcolor();
            
            onCursor=(MS.getcursor(0)==vx)&&(MS.getcursor(1)==vy)&&(MS.getcursor(2)==vz);

            if(onCursor){
                cellcolor=cv::Scalar(0xff,0xff,0xff);
            }else{
                if(!MS.getdemined(vx,vy,vz)){
                    unsigned short R=fr*10*10+44;
                    unsigned short G=fr*10*10;
                    unsigned short B=fr*10*10+22;

                    cellcolor=cv::Scalar(B,G,R);
                }
            }

            cv::String celltext;
            if(!MS.getdemined(vx,vy,vz)){
                if(MS.getvisual(vx,vy,vz)==-1){
                    celltext="f";
                }else{
                    celltext="";
                }
            }else if(MS.getdemined(vx,vy,vz)){
                if(MS.getvisual(vx,vy,vz)==0){
                    celltext="";
                }else{
                    celltext=std::to_string(MS.getvisual(vx,vy,vz));
                }
            }else{
                celltext="error";
            }

            if(!(MS.getdemined(vx,vy,vz)&&MS.getvisual(vx,vy,vz)==0)||onCursor){
                
                circle(img,cv::Point(fy*fr+oy,fz*fr+oz),size*fr,cellcolor,-1);            
                circle(img,cv::Point(fy*fr+oy,fz*fr+oz),size*fr,cv::Scalar(0xff,0xff,0xff),1);
                
                putText(img,celltext,cv::Point(fy*fr+oy,fz*fr+oz),1,0.8,cv::Scalar(20,20,20));
                
                if(MS.getgameclear()){
                    putText(img,"GAME CLEAR",cv::Point(MS.getCANVAS_WIDTH()*3/8,MS.getCANVAS_HEIGHT()/2),1,3.0,cv::Scalar(140,170,160),2);
                }else if(MS.getgameover()){
                    putText(img,"GAME OVER",cv::Point(MS.getCANVAS_WIDTH()*3/8,MS.getCANVAS_HEIGHT()/2),1,3.0,cv::Scalar(70,100,90),2);
                }
            }
        }
    }

    //controller
    for(int i=0;i<6;i++){
        for(int j=i+1;j<6;j++){
            if((*(MS.getcontrollerlistPtr(i))).getx()>(*(MS.getcontrollerlistPtr(j))).getx()){
               Controller t=(*(MS.getcontrollerlistPtr(i)));
               (*(MS.getcontrollerlistPtr(i)))=(*(MS.getcontrollerlistPtr(j)));
               (*(MS.getcontrollerlistPtr(j)))=t;
            }
        }
    }

    for(int i=0;i<6;i++){
        double cy,cz,cr,size;
        std::int16_t oy,oz;
        unsigned short R,G,B;

        cy=(*(MS.getcontrollerlistPtr(i))).gety();
        cz=(*(MS.getcontrollerlistPtr(i))).getz();
        cr=1+(*(MS.getcontrollerlistPtr(i))).getx()/200;
        oy=MS.getCANVAS_WIDTH()/2;
        oz=MS.getCANVAS_HEIGHT()/2+MS.getCANVAS_HEIGHT();
        size=MS.getcellsize()*cr;

        if((*(MS.getcontrollerlistPtr(i))).getex()==1){
            R=0xff;
        }else if((*(MS.getcontrollerlistPtr(i))).getex()==-1){
            R=0x00;
        }else{
            R=0x88;
        }

        if((*(MS.getcontrollerlistPtr(i))).getey()==1){
            G=0xff;
        }else if((*(MS.getcontrollerlistPtr(i))).getey()==-1){
            G=0x00;
        }else{
            G=0x88;
        }

        if((*(MS.getcontrollerlistPtr(i))).getez()==1){
            B=0xff;
        }else if((*(MS.getcontrollerlistPtr(i))).getez()==-1){
            B=0x00;
        }else{
            B=0x88;
        }

        circle(img,cv::Point(cy*cr+oy,cz*cr+oz),size,cv::Scalar(R,G,B),-1);
        
        if(i==2){
            circle(img,cv::Point(oy,oz),2*MS.getcellsize(),cv::Scalar(255,255,255),-1);
        }
    }

    imshow("3Dminesweeper", img);    
    
    MS.setcanvas(img);

    cv::setMouseCallback("3Dminesweeper",mouse_callback,&img);
    
    img.cv::Mat::release();

    switch(cv::waitKey(33)){
        case 101://easy
            modeChange(0);
            break;
        case 110://normal
            modeChange(1);
            break;
        case 104://hard
            modeChange(2);
            break;
        case 114://reset
            gameInitialize();
            break;
        default:
            break;
    }

}
}

void start(){
 	std::thread th_main(TDMineSweeper::gameInitialize);
    std::thread th_time(TDMineSweeper::timeCounter);
    
    th_main.join();
    th_time.join(); 
}

};

int main( int argc, char** argv )
{ 
    TDMineSweeper::start();
    return 0;
}
