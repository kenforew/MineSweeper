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

namespace TDMinesweeper{

class Base{
protected:
    double x,y,z;
public:
    Base(double xx,double yy,double zz){
        x=xx,y=yy,z=zz;
    }
    ~Base(){}

    double getx(){return x;}
    double gety(){return y;}
    double getz(){return z;}

    void setx(double xx){x=xx;}
    void sety(double yy){y=yy;}
    void setz(double zz){z=zz;}

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

class Controller:public Base{
private:
    std::int16_t ex,ey,ez;
    cv::Scalar color;
public:
    Controller(double xx,double yy,double zz,std::int16_t exx,std::int16_t eyy,std::int16_t ezz):Base(xx,yy,zz)
    {
        x=xx,y=yy,z=zz,ex=exx,ey=eyy,ez=ezz;
    }
    ~Controller(){}
    
    std::int16_t getex(){return ex;}
    std::int16_t getey(){return ey;}
    std::int16_t getez(){return ez;}

    cv::Scalar getcolor(){return color;}

    void setcolor(cv::Scalar cc){color=cc;}
};

class Cell:public Base{
private:
    cv::Scalar color;
    std::string label;
    std::uint16_t vx,vy,vz,neighbor;
    bool danger,demined,flag;
public:
    Cell(double xx,double yy,double zz,std::uint16_t vxx,std::uint16_t vyy,std::uint16_t vzz):Base(xx,yy,zz)
    {
        x=xx,y=yy,z=zz,
        vx=vxx,vy=vyy,vz=vzz;
        neighbor=0;
        danger=0,demined=0,flag=0;
    }
    ~Cell(){}

    std::uint16_t getvx(){return vx;}
    std::uint16_t getvy(){return vy;}
    std::uint16_t getvz(){return vz;}

    cv::Scalar getcolor(){return color;}
    
    std::string getlabel(){return label;}
    
    bool getdanger(){return danger;}
    bool getdemined(){return demined;}
    std::uint16_t getneighbor(){return neighbor;}
    bool getflag(){return flag;}
    
    void setcolor(cv::Scalar cc){color=cc;}
    void setlabel(std::string ll){label=ll;}

    void setdanger(bool dd){danger=dd;}
    void setdemined(bool dd){demined=dd;}
    void setneighbor(std::uint16_t nn){neighbor=nn;}
    void setflag(bool ff){flag=ff;}
};

class GrobalVariables{
private:
    std::int16_t
    CANVAS_WIDTH,CANVAS_HEIGHT,
	mouseDownX,mouseDownY,mouseEscapeX,mouseEscapeY,
    mouseUpdateX,mouseUpdateY,mouseUpX,mouseUpY,
    boardX,boardY,boardZ,mines,interval,cellsize,
    time;
    
    bool
    mouseDownFlag,leftClickFlag,longPressFlag,initFlag,
    gameover,gameclear;
    
    std::vector<std::int16_t> cursor;
    std::vector<Controller> controllerlist;
    std::vector<std::vector<std::vector<Cell>>> celllist;
    cv::Mat canvas;

    std::default_random_engine generator;
    std::random_device rd;

public:
    GrobalVariables(){
        CANVAS_WIDTH=800,CANVAS_HEIGHT=800;

        mouseDownX=0,mouseDownY=0;
        mouseEscapeX=0,mouseEscapeY=0;
        mouseUpdateX=0,mouseUpdateY=0;
        mouseUpX=0,mouseUpY=0;
        boardX=6,boardY=6,boardZ=6,mines=10;
        interval=CANVAS_HEIGHT/10;
        cellsize=interval/2.2;
        time=0;

        mouseDownFlag=0;
        leftClickFlag=0;
        longPressFlag=0;
        initFlag=0;

        gameover=0;
        gameclear=0;

        cursor={3,3,3};
    }
    void setCW(std::int16_t xx){CANVAS_WIDTH=xx;}
    void setCH(std::int16_t yy){CANVAS_HEIGHT=yy;}
    void setmouseDownX(std::int16_t xx){mouseDownX=xx;}
    void setmouseDownY(std::int16_t yy){mouseDownY=yy;}
    void setmouseEscapeX(std::int16_t xx){mouseEscapeX=xx;}
    void setmouseEscapeY(std::int16_t yy){mouseEscapeY=yy;}
    void setmouseUpdateX(std::int16_t xx){mouseUpdateX=xx;}
    void setmouseUpdateY(std::int16_t yy){mouseUpdateY=yy;}
    void setmouseUpX(std::int16_t xx){mouseUpX=xx;}
    void setmouseUpY(std::int16_t yy){mouseUpY=yy;}
    
    void setboardX(std::int16_t xx){boardX=xx;}
    void setboardY(std::int16_t yy){boardY=yy;}
    void setboardZ(std::int16_t zz){boardZ=zz;}
    void setmines(std::int16_t mm){mines=mm;}
    void setinterval(std::int16_t ii){interval=ii;}
    void setcellsize(std::int16_t cc){cellsize=cc;}

    void settime(std::int16_t tt){time=tt;}
    
    void setmouseDownFlag(bool mm){mouseDownFlag=mm;}
    void setleftClickFlag(bool ll){leftClickFlag=ll;}
    void setlongPressFlag(bool ll){longPressFlag=ll;}
    void setinitFlag(bool ii){initFlag=ii;}
    
    void setgameover(bool gg){gameover=gg;};
    void setgameclear(bool gg){gameclear=gg;};

    void setcursor(std::int16_t i,std::int16_t v){
        cursor[i]=v;
    }

    void setcanvas(cv::Mat cc){canvas=cc;}
    
    std::int16_t getCW(){return CANVAS_WIDTH;}
    std::int16_t getCH(){return CANVAS_HEIGHT;}
    std::int16_t getmouseDownX(){return mouseDownX;}
    std::int16_t getmouseDownY(){return mouseDownY;}
    std::int16_t getmouseEscapeX(){return mouseEscapeX;}
    std::int16_t getmouseEscapeY(){return mouseEscapeY;}
    std::int16_t getmouseUpdateX(){return mouseUpdateX;}
    std::int16_t getmouseUpdateY(){return mouseUpdateY;}
    std::int16_t getmouseUpX(){return mouseUpX;}
    std::int16_t getmouseUpY(){return mouseUpY;}
    
    std::int16_t gettime(){return time;}

    std::int16_t random(){
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> distrib(0,boardX*boardY*boardZ-1);
        return distrib(gen);
    }

    std::int16_t getboardX(){return boardX;}
    std::int16_t getboardY(){return boardY;}
    std::int16_t getboardZ(){return boardZ;}
    std::int16_t getmines(){return mines;}
    std::int16_t getinterval(){return interval;}
    std::int16_t getcellsize(){return cellsize;}

    Controller * getcontrollerPtr(std::int16_t i){
        return &(controllerlist[i]);
    }
    
    Cell * getcellPtr(std::int16_t xx,std::int16_t yy,std::int16_t zz){
        return &(celllist[xx][yy][zz]);
    }

    bool getmouseDownFlag(){return mouseDownFlag;}
    bool getleftClickFlag(){return leftClickFlag;}
    bool getlongPressFlag(){return longPressFlag;}
    bool getinitFlag(){return initFlag;}
    bool getgameover(){return gameover;}
    bool getgameclear(){return gameclear;}
    
    std::int16_t getcursor(std::uint16_t ii){return cursor[ii];}

    cv::Mat getcanvas(){
        return canvas;
    }

    void generateController(){
        controllerlist.clear();
        for(auto i=0;i<2;++i){
            std::int16_t sign=2*i-1;
            double branch=sign*100.0;
            controllerlist.push_back(Controller(branch,0.0,0.0,sign,0,0));
            controllerlist.push_back(Controller(0.0,branch,0.0,0,sign,0));
            controllerlist.push_back(Controller(0.0,0.0,branch,0,0,sign));
        }
    }

    void generateCell(){
        celllist.clear();
        for(auto i=0;i<boardX;++i){
            celllist.push_back(std::vector<std::vector<Cell>>());
            for(auto j=0;j<boardY;++j){
                celllist[i].push_back(std::vector<Cell>());
                for(auto k=0;k<boardZ;++k){
                    double x,y,z;
                    x=interval/2*(i-(double)(boardX-1)/2);
                    y=interval/2*(j-(double)(boardY-1)/2);
                    z=interval/2*(k-(double)(boardZ-1)/2);
                    celllist[i][j].push_back(Cell(x,y,z,i,j,k));
                }			
            }
        }
    } 
};


GrobalVariables GV;

void controllerInitialize();
void cellInitialize();
void timeCounter();
std::int16_t flagCounter();
void modeChange(std::uint16_t);
cv::Scalar cellColor(std::uint16_t);
void gameClearJudge();
void gameInitialize();
void dangerInitialize();
void safechain(std::int16_t,std::int16_t,std::int16_t);
void safechain2(std::int16_t,std::int16_t,std::int16_t,std::int16_t,std::int16_t,std::int16_t);
bool safe(std::int16_t,std::int16_t,std::int16_t);
bool safe2(std::int16_t,std::int16_t,std::int16_t,std::int16_t,std::int16_t,std::int16_t);
std::uint16_t count(std::int16_t,std::int16_t,std::int16_t);
std::uint16_t count2(std::int16_t,std::int16_t,std::int16_t,std::int16_t,std::int16_t,std::int16_t);
void gameDisplay();

void controllerInitialize(){
    GV.generateController();
    for(auto i=0;i<6;++i){
        Controller object=(*GV.getcontrollerPtr(i));

        auto R=0,G=0,B=0;

        if(object.getex()==1){
            R=0xff;
        }else if(object.getex()==-1){
            R=0x00;
        }else{
            R=0x88;
        }
        
        if(object.getey()==1){
            G=0xff;
        }else if(object.getey()==-1){
            G=0x00;
        }else{
            G=0x88;
        }
        
        if(object.getez()==1){
            B=0xff;
        }else if(object.getez()==-1){
            B=0x00;
        }else{
            B=0x88;
        }

        (*GV.getcontrollerPtr(i)).setcolor(cv::Scalar(B,G,R));
    }
}

void cellInitialize(){
    GV.generateCell();
    dangerInitialize();
    for(auto i=0;i<GV.getboardX();++i){
        for(auto j=0;j<GV.getboardY();++j){
            for(auto k=0;k<GV.getboardZ();++k){
                std::uint16_t neighbors=count(i,j,k);
                (*GV.getcellPtr(i,j,k)).setneighbor(neighbors);
                (*GV.getcellPtr(i,j,k)).setcolor(cellColor(neighbors));
                (*GV.getcellPtr(i,j,k)).setlabel(
                    (*GV.getcellPtr(i,j,k)).getdanger()
                    ?(std::string)"b"
                    :std::to_string(neighbors)
                );
            }
        }
    }
}

void timeCounter(){	
    while(1){
        while(GV.getinitFlag()){
            sleep(1);
            while(GV.getinitFlag()){
                GV.settime(GV.gettime()+1);
                sleep(1);
            }
        }
    }
    return;
}

std::int16_t flagCounter(){
    std::int16_t x=0;
    for(auto i=0;i<GV.getboardX();++i){
        for(auto j=0;j<GV.getboardY();++j){
            for(auto k=0;k<GV.getboardZ();++k){
                if((*GV.getcellPtr(i,j,k)).getflag()&&!(*GV.getcellPtr(i,j,k)).getdemined()){
                    x++;
                }
            }
        }
    }
    return x;
}

void modeChange(std::uint16_t m){
    switch(m){
        case 0:
            GV.setboardX(6);
            GV.setboardY(6);
            GV.setboardZ(6);
            GV.setmines(10);
            break;
        case 1:
            GV.setboardX(8);
            GV.setboardY(8);
            GV.setboardZ(8);
            GV.setmines(40);
            break;
        case 2:
            GV.setboardX(10);
            GV.setboardY(10);
            GV.setboardZ(10);
            GV.setmines(99);
            break;
        default:
            break;
    }

    gameInitialize();
}

cv::Scalar cellColor(std::uint16_t x){
    uchar R=0,G=0,B=0;

    if(x%3==0){
        R=0x11;
    }else if(x%3==1){
        R=0x55;
    }else if(x%3==2){
        R=0x99;
    }
    
    float var_g=x/2;
    std::uint16_t gg=(std::uint16_t)var_g;

    if(gg%3==0){
        G=0x44;
    }else if(gg%3==1){
        G=0x88;
    }else if(gg%3==2){
        G=0xcc;
    }

    float var_h=x/3;
    std::uint16_t hh=(std::uint16_t)var_h;

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
    std::uint16_t counter=0;
    for(auto i=0;i<GV.getboardX();++i){
        for(auto j=0;j<GV.getboardY();++j){
            for(auto k=0;k<GV.getboardZ();++k){
                if((*GV.getcellPtr(i,j,k)).getdemined()){
                    counter++;
                }
            }
        }
    }

    if(counter==GV.getboardX()*GV.getboardY()*GV.getboardZ()-GV.getmines()){
        GV.setgameclear(1);
        GV.setgameover(1);
        GV.setinitFlag(0);
    }

    if(GV.getgameclear()){
        std::cout<<"game clear"<<std::endl;
    }
}

void gameInitialize(){
    GV.setgameover(0);
    GV.setgameclear(0);
    GV.settime(0);
    GV.setinitFlag(0);

    GV.setmouseDownX(0);
    GV.setmouseDownY(0);
    GV.setmouseUpdateX(0);
    GV.setmouseUpdateY(0);
    GV.setmouseUpX(0);
    GV.setmouseUpY(0);

    GV.setcursor(0,(std::int16_t)(GV.getboardX()/2));
    GV.setcursor(1,(std::int16_t)(GV.getboardY()/2));
    GV.setcursor(2,(std::int16_t)(GV.getboardZ()/2));

    controllerInitialize();
    
    cellInitialize();   

    gameDisplay();
}

void dangerInitialize(){
    std::vector<std::int16_t> mineIndex;
    bool newIntFlag=1;

    while(mineIndex.size()<GV.getmines()){
        int rand=GV.random();
        newIntFlag=1;
        for(auto i=0;i<mineIndex.size();++i){
            if(mineIndex[i]==rand){
                newIntFlag=0;
                break;
            }
        }
        if(newIntFlag){
            mineIndex.push_back(rand);
        }
    }

    for(auto i=0;i<GV.getmines();++i){
        std::int16_t x=(mineIndex[i]/(GV.getboardY()*GV.getboardZ()))%GV.getboardX();
        std::int16_t y=(mineIndex[i]/GV.getboardZ())%GV.getboardY();
        std::int16_t z=(mineIndex[i])%GV.getboardZ();
        (*GV.getcellPtr(x,y,z)).setdanger(1);
    }
}

void safechain(std::int16_t x,std::int16_t y,std::int16_t z){
    for(auto i=0;i<3;++i){
        for(auto j=0;j<3;++j){
            for(auto k=0;k<3;++k){
                if(!(i==1&&j==1&&k==1)){
                    safechain2(x,y,z,i-1,j-1,k-1);
                }
            }
        }
    }
}

void safechain2(std::int16_t x,std::int16_t y,std::int16_t z,std::int16_t dx,std::int16_t dy,std::int16_t dz){
    if(x>=0&&x<GV.getboardX()
    &&y>=0&&y<GV.getboardY()
    &&z>=0&&z<GV.getboardZ()
    &&x+dx>=0&&x+dx<GV.getboardX()
    &&y+dy>=0&&y+dy<GV.getboardY()
    &&z+dz>=0&&z+dz<GV.getboardZ()
    &&!(*GV.getcellPtr(x+dx,y+dy,z+dz)).getdemined()){
        (*GV.getcellPtr(x+dx,y+dy,z+dz)).setdemined(1);
        if(safe(x+dx,y+dy,z+dz)){
            safechain(x+dx,y+dy,z+dz);
        }else{
            return;
        }
    }else{
        return;
    }
}

bool safe(std::int16_t x,std::int16_t y,std::int16_t z){
    bool result=1;
    for(auto i=0;i<3;++i){
        for(auto j=0;j<3;++j){
            for(auto k=0;k<3;++k){
                if(!(i==1&&j==1&&k==1)){
                    result=result&&safe2(x,y,z,i-1,j-1,k-1);
                }
            }
        }
    }
    return result;
}

bool safe2(std::int16_t x,std::int16_t y,std::int16_t z,std::int16_t dx,std::int16_t dy,std::int16_t dz){
    if(x+dx<0||x+dx>GV.getboardX()-1
    ||y+dy<0||y+dy>GV.getboardY()-1
    ||z+dz<0||z+dz>GV.getboardZ()-1){
        return 1;
    }else{
        if((*GV.getcellPtr(x+dx,y+dy,z+dz)).getdanger()){
            return 0;
        }else{
            return 1;
        }
    }
}

std::uint16_t count(std::int16_t x,std::int16_t y,std::int16_t z){
    std::uint16_t sum=0;
    for(auto i=0;i<3;++i){
        for(auto j=0;j<3;++j){
            for(auto k=0;k<3;++k){
                if(!(i==1&&j==1&&k==1)){
                    sum+=count2(x,y,z,i-1,j-1,k-1);
                }
            }
        }
    }
    return sum;
}

std::uint16_t count2(std::int16_t x,std::int16_t y,std::int16_t z,std::int16_t dx,std::int16_t dy,std::int16_t dz){
    if(x+dx<0||x+dx>GV.getboardX()-1
    ||y+dy<0||y+dy>GV.getboardY()-1
    ||z+dz<0||z+dz>GV.getboardZ()-1){
        return 0;
    }else{
        return (*GV.getcellPtr(x+dx,y+dy,z+dz)).getdanger()?1:0;
    }
}

void mouse_callback(int event,int x,int y,int flags,void *userdata){
    if(event==cv::EVENT_LBUTTONDOWN){
        GV.setmouseDownX(x);
        GV.setmouseDownY(y);
        GV.setmouseEscapeX(x);
        GV.setmouseEscapeY(y);
        GV.setmouseDownFlag(1);
        GV.setleftClickFlag(1);
        GV.setlongPressFlag(0);
    }
    if(event==cv::EVENT_LBUTTONUP){
        GV.setmouseDownFlag(0);
        if(!GV.getlongPressFlag()){
            uchar r,g,b;
            r=GV.getcanvas().at<uchar>(y,3*x+2);
            g=GV.getcanvas().at<uchar>(y,3*x+1);
            b=GV.getcanvas().at<uchar>(y,3*x);
            if(r==0xff&&g==0xff&&b==0xff){
                std::int16_t cx,cy,cz;
                cx=GV.getcursor(0);
                cy=GV.getcursor(1);
                cz=GV.getcursor(2);
                Cell object=(*GV.getcellPtr(cx,cy,cz));
                if(!object.getdemined()&&!object.getflag()){
                    (*GV.getcellPtr(cx,cy,cz)).setdemined(1);
                    if(object.getdanger()){//bomb
                        GV.setgameover(1);
                        GV.setinitFlag(0);
                    }else if(safe(cx,cy,cz)){// no mine around
                        safechain(cx,cy,cz);
                    }

                    if(!GV.getgameover()){
                        GV.setinitFlag(1);
                    }
                }
            }else if(!(r==0&&g==0&&b==0)){
                if(r==0xff){
                    GV.setcursor(0,(GV.getcursor(0)+1)%GV.getboardX());
                }
                if(r==0x00){
                    GV.setcursor(0,GV.getcursor(0)-1);
                    if(GV.getcursor(0)<0)GV.setcursor(0,(GV.getcursor(0)+GV.getboardX()));
                }
                if(g==0xff){
                    GV.setcursor(1,(GV.getcursor(1)+1)%GV.getboardY());
                }
                if(g==0x00){
                    GV.setcursor(1,GV.getcursor(1)-1);
                    if(GV.getcursor(1)<0)GV.setcursor(1,(GV.getcursor(1)+GV.getboardY()));
                }
                if(b==0xff){
                    GV.setcursor(2,(GV.getcursor(2)+1)%GV.getboardZ());
                }
                if(b==0x00){
                    GV.setcursor(2,GV.getcursor(2)-1);
                    if(GV.getcursor(2)<0)GV.setcursor(2,(GV.getcursor(2)+GV.getboardZ()));
                }
            }
        }
    }
    if(event==cv::EVENT_RBUTTONDOWN){
        GV.setmouseDownFlag(1);
        GV.setleftClickFlag(1);
        GV.setlongPressFlag(0);
    }
    if(event==cv::EVENT_RBUTTONUP){
        GV.setmouseDownFlag(0);
        if(!GV.getlongPressFlag()){
            std::int16_t cx,cy,cz;
            cx=GV.getcursor(0);
            cy=GV.getcursor(1);
            cz=GV.getcursor(2);
            Cell object(*GV.getcellPtr(cx,cy,cz));
            if(!object.getdemined()&&object.getflag()){
                (*GV.getcellPtr(cx,cy,cz)).setflag(0);
            }else if(!object.getdemined()&&!object.getflag()){
                (*GV.getcellPtr(cx,cy,cz)).setflag(1);
            }
        }
    	GV.setlongPressFlag(0);
        if(GV.getgameover()){
            std::cout<<">>>game over<<<"<<std::endl;
        }
    }
    if(event==cv::EVENT_MOUSEMOVE){
        GV.setmouseUpdateX(x);
        GV.setmouseUpdateY(y);

        double xd,yd,ld;
        xd=x-GV.getmouseDownX();
        yd=y-GV.getmouseDownY();
        ld=sqrt(xd*xd+yd*yd);
        if(ld>10){
            GV.setlongPressFlag(1);
        }
        if(GV.getmouseDownFlag()&&GV.getlongPressFlag()){
            double rotZ,rotY;
            rotZ=(GV.getmouseUpdateX()-GV.getmouseEscapeX())/300.0;
            rotY=(GV.getmouseUpdateY()-GV.getmouseEscapeY())/300.0;
            for(auto i=0;i<GV.getboardX();++i){
                for(auto j=0;j<GV.getboardY();++j){
                    for(auto k=0;k<GV.getboardZ();++k){
                        (*GV.getcellPtr(i,j,k)).rotation(rotZ,rotY);
                    }
                }
            }
            for(auto i=0;i<6;++i){
                (*GV.getcontrollerPtr(i)).rotation(rotZ,rotY);
            }
        }
        GV.setmouseEscapeX(x);
        GV.setmouseEscapeY(y);
    }
}

void gameDisplay(){
while(1){
    cv::Mat img(cv::Size((int)GV.getCW(),(int)GV.getCH()),CV_8UC3,cv::Scalar(0,0,0));

    cv::Scalar textcolor=cv::Scalar(0xfe,0xfe,0xfe);
    putText(img,"R:New Game",cv::Point(10,40),1,1.2,textcolor);
    putText(img,"E:Easy",cv::Point(10,70),1,1.2,textcolor);
    putText(img,"N:Normal",cv::Point(10,100),1,1.2,textcolor);
    putText(img,"H:Hard",cv::Point(10,130),1,1.2,textcolor);
    putText(img,"Flag : "+std::to_string(flagCounter()),cv::Point(10,200),1,1.2,textcolor);
    putText(img,"Time : "+std::to_string(GV.gettime()),cv::Point(10,230),1,1.2,textcolor);

    std::vector<Cell> sortlist;
    for(auto i=0;i<GV.getboardX();++i){
        for(auto j=0;j<GV.getboardY();++j){
            for(auto k=0;k<GV.getboardZ();++k){
                sortlist.push_back(*GV.getcellPtr(i,j,k));
            }
        }
    }

    std::int16_t cube=GV.getboardX()*GV.getboardY()*GV.getboardZ();
    for(auto i=0;i<cube;++i){
        for(auto j=i+1;j<cube;++j){
            if(sortlist[i].getx()>sortlist[j].getx()){
                Cell t=sortlist[i];
                sortlist[i]=sortlist[j];
                sortlist[j]=t;
            }
        }
    }
    

    for(auto i=0;i<cube;++i){
        double fy,fz,fr;
        std::int16_t oy,oz,size;
        fy=sortlist[i].gety();
        fz=sortlist[i].getz();
        fr=1+sortlist[i].getx()/300;
        oy=GV.getCW()/2;
        oz=GV.getCH()/4;
        size=GV.getcellsize()/2;
        if(fy*fr+oy>size&&fy*fr+oy<GV.getCW()-size
        &&fz*fr+oz>size&&fz*fr+oz<GV.getCH()-size){
            std::int16_t vx,vy,vz;
            cv::Scalar cellcolor;
            bool onCursor;

            vx=sortlist[i].getvx();
            vy=sortlist[i].getvy();
            vz=sortlist[i].getvz();
            cellcolor=sortlist[i].getcolor();
            Cell object=*GV.getcellPtr(vx,vy,vz);

            onCursor=(GV.getcursor(0)==vx)&&(GV.getcursor(1)==vy)&&(GV.getcursor(2)==vz);

            if(onCursor){
                cellcolor=cv::Scalar(0xff,0xff,0xff);
            }else{
                if(!object.getdemined()){
                    uchar R=fr*10*10+44;
                    uchar G=fr*10*10;
                    uchar B=fr*10*10+22;
                    cellcolor=cv::Scalar(B,G,R);
                }
            }

            cv::String celltext;
            if(!object.getdemined()){
                if(object.getflag()){
                    celltext="f";
                }else{
                    celltext="";
                }
            }else{
                celltext=object.getlabel();
            }

            if(!(object.getdemined()&&object.getneighbor()==0)||onCursor){
                circle(img,cv::Point(fy*fr+oy,fz*fr+oz),size*fr,cellcolor,-1);
                circle(img,cv::Point(fy*fr+oy,fz*fr+oz),size*fr,cv::Scalar(0xee,0xee,0xee),1);
                if(celltext!="0"){
                    putText(img,celltext,cv::Point(fy*fr+oy,fz*fr+oz),1,0.8,cv::Scalar(20,20,20));
                }
                if(GV.getgameclear()){
                    putText(img,"GAME CLEAR",cv::Point(GV.getCW()*3/8,GV.getCH()/2),1,3.0,cv::Scalar(140,170,160),2);
                }else if(GV.getgameover()){
                    putText(img,"GAME OVER",cv::Point(GV.getCW()*3/8,GV.getCH()/2),1,3.0,cv::Scalar(70,100,90),2);
                }
            }
        }
    }

    //controller
    std::vector<Controller> ssortlist;
    for(auto i=0;i<6;++i){
        ssortlist.push_back(*(GV.getcontrollerPtr(i)));
    }

    for(auto i=0;i<6;++i){
        for(auto j=i+1;j<6;++j){
            if(ssortlist[i].getx()>ssortlist[j].getx()){
                Controller t=ssortlist[i];
                ssortlist[i]=ssortlist[j];
                ssortlist[j]=t;
            }
        }
    }

    for(auto i=0;i<6;++i){
        double cy,cz,cr,size;
        std::int16_t oy,oz;

        Controller object=ssortlist[i];
        cy=object.gety();
        cz=object.getz();
        cr=1+object.getx()/200;
        oy=GV.getCW()/2;
        oz=GV.getCH()*3/4;
        size=GV.getcellsize()/2*cr;

        circle(img,cv::Point(cy*cr+oy,cz*cr+oz),size,object.getcolor(),-1);

        if(i==2){
            circle(img,cv::Point(oy,oz),2*GV.getcellsize()/2,cv::Scalar(0xff,0xff,0xff),-1);
        }
    }

    cv::imshow("3Dminesweeper",img);

    GV.setcanvas(img);

    cv::setMouseCallback("3Dminesweeper",mouse_callback,&img);

    img.cv::Mat::release();

    switch(cv::waitKey(33)){
        case 101:
            modeChange(0);
            break;
        case 110:
            modeChange(1);
            break;
        case 104:
            modeChange(2);
            break;
        case 114:
            gameInitialize();
            break;
        default:
            break;
    }
}
}

void start(){
    std::thread th_main(gameInitialize);
    std::thread th_time(timeCounter);

    th_main.join();
    th_time.join();
}

};

int main(int argc,char** argv){
    TDMinesweeper::start();
    return 0;
}