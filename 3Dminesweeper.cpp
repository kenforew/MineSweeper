#include <iostream>
#include <string>
#include <math.h>
#include <random>
#include <vector>

#include <cstdlib>
#include <Windows.h>
#include <unistd.h>

#include <chrono>
#include <thread>

#include <opencv2/opencv.hpp>

using namespace std;

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
    int ex;
    int ey;
    int ez;
public:
    Controller(double xx,double yy,double zz,int exx,int eyy,int ezz):Physics(xx,yy,zz)
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

    int getex(){
        return ex;
    }

    int getey(){
        return ey;
    }

    int getez(){
        return ez;
    }
};

class Cell : public Physics{
private:
    cv::Scalar color;
    string label;
    int vx;
    int vy;
    int vz;
public:   
    Cell(double xx,double yy,double zz,cv::Scalar cc,string ll,int exx,int eyy,int ezz):Physics(xx,yy,zz){
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

    string getlabel(){
        return label;
    }

    int getvx(){
        return vx;
    }

    int getvy(){
        return vy;
    }

    int getvz(){
        return vz;
    }

    void setcolor(cv::Scalar cc){
        color=cc;
    }

    void setlabel(string ll){
        label=ll;
    }
};

class MineSweeper{
private:
    int CANVAS_WIDTH;
    int CANVAS_HEIGHT;

    int mouseDownX;
    int mouseDownY;
    int mouseEscapeX;
    int mouseEscapeY;
    int mouseUpdateX;
    int mouseUpdateY;
    int mouseUpX;
    int mouseUpY;
    bool mouseDownFlag;
    bool leftClickFlag;
    bool longPressFlag;
    bool initFlag;

    bool gameover;
    bool gameclear;

    int boardX;
    int boardY;
    int boardZ;
    int mines;

    int interval;
    int cellsize;

    vector<vector<vector<bool>>> danger;
    vector<vector<vector<bool>>> demined;
    vector<vector<vector<int>>> visual;
    vector<int> cursor;

    vector<Controller> controllerlist;
    vector<Cell> mainlist;

    cv::Mat canvas;
    
    std::default_random_engine generator;

    int time;
        
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

    }
    void setCANVAS_WIDTH(int xx){
        CANVAS_WIDTH=xx;
    }
    void setCANVAS_HEIGHT(int yy){
        CANVAS_HEIGHT=yy;
    }
    void setmouseDownX(int xx){
        mouseDownX=xx;
    }
    void setmouseDownY(int yy){
        mouseDownY=yy;
    }
    void setmouseEscapeX(int xx){
        mouseEscapeX=xx;
    }
    void setmouseEscapeY(int yy){
        mouseEscapeY=yy;
    }
    void setmouseUpdateX(int xx){
        mouseUpdateX=xx;
    }
    void setmouseUpdateY(int yy){
        mouseUpdateY=yy;
    }
    void setmouseUpX(int xx){
        mouseUpX=xx;
    }
    void setmouseUpY(int yy){
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
    void setboardX(int xx){
        boardX=xx;
    }
    void setboardY(int yy){
        boardY=yy;
    }
    void setboardZ(int zz){
        boardZ=zz;
    }
    void setmines(int mm){
        mines=mm;
    }
    void setinterval(int ii){
        interval=ii;
    }
    void setcellsize(int cc){
        cellsize=cc;
    }
    void setdangerAll(vector<vector<vector<bool>>> dd){
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
    void setdanger(int x,int y,int z,bool dd){
        danger[x][y][z]=dd;
    }
    void setdeminedAll(vector<vector<vector<bool>>> dd){
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
    void setdemined(int x,int y,int z,bool dd){
        demined[x][y][z]=dd;
    }
    void setvisualAll(vector<vector<vector<int>>> vv){
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
    void setvisual(int x,int y,int z,int vv){
        visual[x][y][z]=vv;
    }
    void setcursor(int i,int v){
	    cursor[i]=v;
    }
    void setcontrollerlist(vector<Controller> cc){
        for(size_t i=0;i<cc.size();i++){
            controllerlist.push_back(cc[i]);
        }
    }
    void setcontrollerlistClear(){
        controllerlist.clear();
        //cout<<"cl"<<controllerlist.size()<<endl;
        //controllerlist.resize(6);
    }
    void setmainlist(vector<Cell> mm){
        for(size_t i=0;i<mm.size();i++){
            mainlist.push_back(mm[i]);
        }
    }
    void setmainlistClear(){
        mainlist.clear();
        //mainlist.resize(boardX*boardY*boardZ);
    }
    void setcanvas(cv::Mat cc){
        canvas=cc;
    }
    void settime(int tt){
        time=tt;
    }
    int getCANVAS_WIDTH(){
        return CANVAS_WIDTH;
    }
    int getCANVAS_HEIGHT(){
        return CANVAS_HEIGHT;
    }
    int getmouseDownX(){
        return mouseDownX;
    }
    int getmouseDownY(){
        return mouseDownY;
    }
    int getmouseEscapeX(){
        return mouseEscapeX;
    }
    int getmouseEscapeY(){
        return mouseEscapeY;
    }
    int getmouseUpdateX(){
        return mouseUpdateX;
    }
    int getmouseUpdateY(){
        return mouseUpdateY;
    }
    int getmouseUpX(){
        return mouseUpX;
    }
    int getmouseUpY(){
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
    int getboardX(){
        return boardX;
    }
    int getboardY(){
        return boardY;
    }
    int getboardZ(){
        return boardZ;
    }
    int getmines(){
        return mines;
    }
    int getinterval(){
        return interval;
    }
    int getcellsize(){
        return cellsize;
    }
    vector<vector<vector<bool>>> getdangerAll(){
        return danger;
    }
    bool getdanger(int x,int y,int z){
        return danger[x][y][z];
    }
    vector<vector<vector<bool>>> getdeminedAll(){
        return demined;
    }
    bool getdemined(int x,int y,int z){
        return demined[x][y][z];
    }
    vector<vector<vector<int>>> getvisualAll(){
        return visual;
    }
    int getvisual(int x,int y,int z){
        return visual[x][y][z];
    }
    vector<int> getcursorAll(){
        return cursor;
    }
    int getcursor(int i){
        return cursor[i];
    }
    vector<Controller> getcontrollerlistAll(){
        return controllerlist;
    }
    Controller getcontrollerlist(int i){
        return controllerlist[i];
    }
    Controller * getcontrollerlistPtr(int i){
        return &controllerlist[i];
    }
    vector<Cell> getmainlistAll(){
        return mainlist;
    }
    Cell getmainlist(int i){
        return mainlist[i];
    }
    Cell * getmainlistPtr(int i){
        return &mainlist[i];
    }
    cv::Mat getcanvas(){
        return canvas;
    }
    int random(){
        std::uniform_int_distribution<int> distribution(0,boardX*boardY*boardZ-1);
        return distribution(generator);
    }
    int gettime(){
        return time;
    }
};

MineSweeper MS;

void timeCounter();
int flagCounter();
void generateLists();
void modeChange(int);
cv::Scalar cellColor(int);
void gameClearJudge();
void gameInitialize();
void boardInitialize();
void dangerInitialize();
void safechain(int,int,int);
void safechain2(int,int,int,int,int,int);
bool safe(int,int,int);
bool safe2(int,int,int,int,int,int);
int count(int,int,int);
int count2(int,int,int,int,int,int);
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
int flagCounter(){
    int x=0;
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
    
    vector<vector<vector<bool>>> ddanger;
    vector<vector<vector<bool>>> ddemined;
    vector<vector<vector<int>>> vvisual;

    ddanger.clear();
    ddemined.clear();
    vvisual.clear();

    ddanger.resize(MS.getboardX());
    ddemined.resize(MS.getboardX());
    vvisual.resize(MS.getboardX());

    for(size_t i=0;i<MS.getboardX();i++){
        ddanger[i].resize(MS.getboardY());
        ddemined[i].resize(MS.getboardY());
        vvisual[i].resize(MS.getboardY());
    }

    for(size_t i=0;i<MS.getboardX();i++){
        for(size_t j=0;j<MS.getboardY();j++){
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


void modeChange(int m){
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

cv::Scalar cellColor(int x){
        int R=0,G=0,B=0;
        
        if(x%3==0){
            R=0x11;
        }else if(x%3==1){
            R=0x55;
        }else if(x%3==2){
            R=0x99;
        }
        
        float var_g=x/2;
        int gg=(int)var_g;

        if(gg%3==0){
            G=0x44;
        }else if(gg%3==1){
            G=0x88;
        }else if(gg%3==2){
            G=0xcc;
        }

        float var_h=x/3;
        int hh=(int)var_h;
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
    int counter=0;
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
        std::cout<<"game clear"<<endl;
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

    vector<Controller> clist;
    clist.clear();

    for(int i=0;i<2;i++){
        int var_i=2*i-1;
        double var_l=MS.getCANVAS_HEIGHT()/4.0*var_i;
        clist.push_back(Controller(var_l,0.0,0.0,var_i,0,0));
        clist.push_back(Controller(0.0,var_l,0.0,0,var_i,0));
        clist.push_back(Controller(0.0,0.0,var_l,0,0,var_i));
    }

    MS.setcontrollerlist(clist);

    //-------------------------------------
    MS.setmainlistClear();

    vector<Cell> mlist;
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
    vector<int> mineIndex;
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
        int x=(mineIndex[i]/(MS.getboardY()*MS.getboardZ()))%MS.getboardX();
        int y=(mineIndex[i]/MS.getboardZ())%MS.getboardY();
        int z=mineIndex[i]%MS.getboardZ();
        MS.setdanger(x,y,z,true);
    }

}

void safechain(int x,int y,int z){
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

void safechain2(int x,int y,int z,int dx,int dy,int dz){
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
            (*(MS.getmainlistPtr((x+dx)*MS.getboardY()*MS.getboardZ()+(y+dy)*MS.getboardZ()+(z+dz)))).setlabel(to_string(MS.getvisual(x+dx,y+dy,z+dz)));
            safechain(x+dx,y+dy,z+dz);
        }else{
            MS.setvisual(x+dx,y+dy,z+dz,count(x+dx,y+dy,z+dz));
            MS.setdemined(x+dx,y+dy,z+dz,true);
            (*(MS.getmainlistPtr((x+dx)*MS.getboardY()*MS.getboardZ()+(y+dy)*MS.getboardZ()+(z+dz)))).setcolor(cellColor(MS.getvisual(x+dx,y+dy,z+dz)));
            (*(MS.getmainlistPtr((x+dx)*MS.getboardY()*MS.getboardZ()+(y+dy)*MS.getboardZ()+(z+dz)))).setlabel(to_string(MS.getvisual(x+dx,y+dy,z+dz)));
            return;
        }
    }else{
        return;
    }
}

bool safe(int x,int y,int z){
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

bool safe2(int x,int y,int z,int dx,int dy,int dz){
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

int count(int x,int y,int z){
    int sum=0;
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

int count2(int x,int y,int z,int dx,int dy,int dz){
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
            short r,g,b;
            r=MS.getcanvas().at<uchar>(y,3*x);
            g=MS.getcanvas().at<uchar>(y,3*x+1);
            b=MS.getcanvas().at<uchar>(y,3*x+2);
           if(r==0xff&&g==0xff&&b==0xff){
                int cx,cy,cz;
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
                        (*(MS.getmainlistPtr(cx*MS.getboardY()*MS.getboardZ()+cy*MS.getboardZ()+cz))).setlabel(to_string(MS.getvisual(cx,cy,cz)));
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
            int cx,cy,cz;
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
            std::cout<<">>>game over<<<"<<endl;   
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

void gameDisplay()
{
    for(int gumi=0;true;){
    
    cv::Mat img(cv::Size(MS.getCANVAS_WIDTH(),2*MS.getCANVAS_HEIGHT()),CV_8UC3,cv::Scalar(0,0,0));
    
    putText(img,"R:New Game",cv::Point(10,40),1,1.2,cv::Scalar(0xfe,0xfe,0xfe));
    putText(img,"E:Easy ",cv::Point(10,70),1,1.2,cv::Scalar(0xfe,0xfe,0xfe));
    putText(img,"N:Normal",cv::Point(10,100),1,1.2,cv::Scalar(0xfe,0xfe,0xfe)); 
    putText(img,"H:Hard",cv::Point(10,130),1,1.2,cv::Scalar(0xfe,0xfe,0xfe));
    putText(img,"Flag : "+to_string(flagCounter()),cv::Point(10,200),1,1.2,cv::Scalar(0xfe,0xfe,0xfe));
    putText(img,"Time : "+to_string(MS.gettime()),cv::Point(10,230),1,1.2,cv::Scalar(0xfe,0xfe,0xfe));
    
    vector<Cell> sortlist;
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
        int oy,oz,size;
        fy=sortlist[i].gety();
        fz=sortlist[i].getz();
        fr=1+sortlist[i].getx()/300;
        oy=MS.getCANVAS_WIDTH()/2;
        oz=MS.getCANVAS_HEIGHT()/2;
        size=MS.getcellsize();
        if(fr>0.0&&fr<2.0
        &&fy*fr+oy>size&&fy*fr+oy<MS.getCANVAS_WIDTH()-size
        &&fz*fr+oz>size&&fz*fr+oz<MS.getCANVAS_HEIGHT()-size){
            int vx,vy,vz;
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
                    int R=fr*10*10+44;
                    int G=fr*10*10;
                    int B=fr*10*10+22;

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
                    celltext=to_string(MS.getvisual(vx,vy,vz));
                }
            }else{
                celltext="error";
            }

            if(!(MS.getdemined(vx,vy,vz)&&MS.getvisual(vx,vy,vz)==0)||onCursor){
                
                circle(img,cv::Point(fy*fr+oy,fz*fr+oz),size*fr,cellcolor,-1);            
                circle(img,cv::Point(fy*fr+oy,fz*fr+oz),size*fr,cv::Scalar(0xff,0xff,0xff),1);
                
                putText(img,celltext,cv::Point(fy*fr+oy,fz*fr+oz),1,0.8,cv::Scalar(20,20,20));
                
                if(MS.getgameclear()){
                    putText(img,"GAME CLEAR",cv::Point(MS.getCANVAS_WIDTH()/2,MS.getCANVAS_HEIGHT()/2),1,3.0,cv::Scalar(100,100,100));
                }else if(MS.getgameover()){
                    putText(img,"GAME OVER",cv::Point(MS.getCANVAS_WIDTH()/2,MS.getCANVAS_HEIGHT()/2),1,3.0,cv::Scalar(100,100,100));
                }
            }
        }
    }

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
        int oy,oz,R,G,B;

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

    switch(cv::waitKey(100)){
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

int main( int argc, char** argv )
{ 
	std::thread th_main(gameInitialize);
    std::thread th_time(timeCounter);
    
    th_main.join();
    th_time.join(); 
    return 0;
}
