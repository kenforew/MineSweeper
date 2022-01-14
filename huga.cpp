#include <conio.h>
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

#include "3Dphysics.cpp"

//using namespace std;

class GrobalVariables{
private:
    int CANVAS_WIDTH;
    int CANVAS_HEIGHT;
    int boardX,boardY,boardZ;
    int spf;
    int delay;
    bool frameFlag;
    bool shiftKeyFlag;
public:
    GrobalVariables(){
        CANVAS_WIDTH=800;
        CANVAS_HEIGHT=800;

        boardX=5;
        boardY=5;
        boardZ=11;

        spf=33;
        delay=500;

        frameFlag=false;
        shiftKeyFlag=false; 
    }
    ~GrobalVariables(){}

    int getCW(){
        return CANVAS_WIDTH;
    }
    int getCH(){
        return CANVAS_HEIGHT;
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
    int getspf(){
        return spf;
    }
    int getdelay(){
        return delay;
    }
    bool getframeFlag(){
        return frameFlag;
    }
    bool getshiftKeyFlag(){
        return shiftKeyFlag;
    }

    void setframeFlag(bool ff){
        frameFlag=ff;
    }
    void setshiftKeyFlag(bool ss){
        shiftKeyFlag=ss;
    }
};

GrobalVariables GV;

class Puyo{
protected:
    unsigned short x,y,z;
    std::vector<unsigned short> color;
    bool rotating;
public:
    Puyo(unsigned short xx,unsigned short yy,unsigned short zz,std::vector<unsigned short> cc,bool rr){
        x=xx;y=yy;z=zz;color=cc;
        rotating=rr;
    }
    ~Puyo(){}

    void setx(unsigned short xx){
        x=xx;
    }
    void sety(unsigned short yy){
        y=yy;
    }
    void setz(unsigned short zz){
        z=zz;
    }
    void setcolorAll(std::vector<unsigned short> cc){
        for(unsigned short i=0;i<3;i++){
            color[i]=cc[i];
        }
    }
    void setcolor(unsigned short rgb,unsigned short cc){
        color[rgb]=cc;
    }

    unsigned short getx(){
        return x;
    }
    unsigned short gety(){
        return y;
    }
    unsigned short getz(){
        return z;
    }
    std::vector<unsigned short> getcolorAll(){
        return color;
    } 
    unsigned short getcolor(unsigned short rgb){
        return color[rgb];
    }
};

class MultiPuyo{
protected:
    unsigned short x,y,z;
    short rotX,rotY,rotZ;
    std::vector<Puyo> puyolist;
    std::vector<short> bind;
public:
    MultiPuyo(short rx,short ry,short rz,std::vector<Puyo> pp){
        rotX=rx;rotY=ry;rotZ=rz;
        puyolist=pp;
        x=pp[0].getx();
        y=pp[0].gety();
        z=pp[0].getz();
        bind={1,0,0};
    }
    ~MultiPuyo(){}

    void setx(unsigned short xx){
        x=xx;
    }
    void sety(unsigned short yy){
        y=yy;
    }
    void setz(unsigned short zz){
        z=zz;
    }
    void setrotX(unsigned short xx){
        rotX=xx;

    }
    void setrotY(unsigned short yy){
        rotY=yy;
    }
    void setrotZ(unsigned short zz){
        rotZ=zz;
    }
    void setpuyolistAll(std::vector<Puyo> pp){
        for(unsigned short i=0;i<pp.size();i++){
            puyolist[i]=pp[i];
        }
    }
    void setpuyolist(unsigned short i,Puyo pp){
        puyolist[i]=pp;
    }

    unsigned short getx(){
        return x;
    }
    unsigned short gety(){
        return y;
    }
    unsigned short getz(){
        return z;
    }
    unsigned short getrotX(){
        return rotX;
    }
    unsigned short getrotY(){
        return rotY;
    }
    unsigned short getrotZ(){
        return rotZ;
    }
    std::vector<Puyo> getpuyolistAll(){
        return puyolist;
    }
    Puyo getpuyolist(unsigned short i){
        return puyolist[i];
    }
    
    void rotateX(){
        int yy,zz,sgn;
        if(GV.getshiftKeyFlag()){
            sgn=1;
        }else{
            sgn=-1;
        }
        yy=-bind[2]*sgn;
        zz=bind[1]*sgn;
        bind[1]=yy;
        bind[2]=zz;
        //puyolist[1].setx(x+bind[0]);
        puyolist[1].sety(y+bind[1]);
        puyolist[1].setz(z+bind[2]);
    }

    void rotateY(){
        int zz,xx,sgn;
        if(GV.getshiftKeyFlag()){
            sgn=1;
        }else{
            sgn=-1;
        }
        zz=-bind[0]*sgn;
        xx=bind[2]*sgn;
        bind[2]=zz;
        bind[0]=xx;

        puyolist[1].setz(z+bind[2]);
        puyolist[1].setx(x+bind[0]);
    }

    void rotateZ(){
        int xx,yy,sgn;
        if(GV.getshiftKeyFlag()){
            sgn=1;
        }else{
            sgn=-1;
        }
        xx=-bind[1]*sgn;
        yy=bind[0]*sgn;
        bind[0]=xx;
        bind[1]=yy;

        puyolist[1].setx(x+bind[0]);
        puyolist[1].sety(y+bind[1]);
    }
    
};

void frameManager();
void title();
void gameInitialize();
void gamedisplay();
void keyEvent();

void frameManager(){
while(1){
    Sleep(GV.getspf());
    GV.setframeFlag(true);
}
    return;
}

void title(){
    cv::Mat img(cv::Size(GV.getCW(),GV.getCH()),CV_8UC3,cv::Scalar(0,0,0));
    
    putText(img,"PUYOPUYO",cv::Point(GV.getCW()/2,GV.getCH()/2),1,2.0,cv::Scalar(0xff,0xff,0xff));
    putText(img,"Press Any Key",cv::Point(GV.getCW()/2,GV.getCH()/2+70),1,1.2,cv::Scalar(0xff,0xff,0xff));
    cv::imshow("3Dpuyopuyo",img);
    
    cv::waitKey(0);
    img.release(); 
    return;
}

void gameInitialize(){

    gamedisplay();
    return;
}

MultiPuyo test=MultiPuyo(0,0,0,{Puyo(2,2,10,{0xff,0,0xff},true),Puyo(2,3,10,{0xff,0xff,0},true)});

void gamedisplay(){
    GV.setframeFlag(false);
while(1){
    cv::Mat img(cv::Size(GV.getCW(),GV.getCH()),CV_8UC3,cv::Scalar(0,0,0));


    circle(img,cv::Point(100,100),20,cv::Scalar(test.getpuyolist(0).getcolor(0),test.getpuyolist(0).getcolor(1),test.getpuyolist(0).getcolor(2)),-1);

    cv::imshow("3Dpuyopuyo",img);
    img.release();
    cv::waitKey(GV.getspf());
    GV.setframeFlag(0);
}
    return;
}

void keyEvent(){
    const unsigned short MSB=0x8000;
while(1){
    if(GetAsyncKeyState(VK_LEFT)&MSB){
        std::cout<<"left"<<std::endl;
        int xx=test.getx()+1;
        //if(xx<0)xx=0;
        if(xx>GV.getboardX())xx=GV.getboardX();
        test.setx(xx);
        Sleep(GV.getdelay());
    }
    if(GetAsyncKeyState(VK_RIGHT)&MSB){
        std::cout<<"right"<<std::endl;
        int xx=test.getx()-1;
        if(xx<0)xx=0;
        //if(xx>GV.getboardX())xx=GV.getboardX();
        test.setx(xx); 
        Sleep(GV.getdelay());
    }
    if(GetAsyncKeyState(VK_UP)&MSB){
        std::cout<<"up"<<std::endl;
        int yy=test.gety()+1;
        //if(yy<0)yy=0;
        if(yy>GV.getboardY())yy=GV.getboardY();
        test.setx(yy); 
        Sleep(GV.getdelay());
    }
    if(GetAsyncKeyState(VK_DOWN)&MSB){
        std::cout<<"down"<<std::endl;
        int yy=test.gety()-1;
        if(yy<0)yy=0;
        //if(yy>GV.getboardY())yy=GV.getboardY();
        test.setx(yy); 
        Sleep(GV.getdelay());
    }
    if(GetAsyncKeyState(VK_SHIFT)&MSB){
        GV.setshiftKeyFlag(1);
    }else{
        GV.setshiftKeyFlag(0);
    }
    Sleep(GV.getspf());

}
    return;
}

int main(){
	std::cout<<"hello world."<<std::endl;

	title();
    
    std::thread th_main(gameInitialize);
    std::thread th_frame(frameManager);
    std::thread th_keyEvent(keyEvent);

    th_main.join();
    th_frame.join();
    th_keyEvent.join();

    return 0;
}