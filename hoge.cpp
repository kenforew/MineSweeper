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

#define CANVAS_WIDTH 800
#define CANVAS_HEIGHT 800

#define PI 3.1415926535

double cursorX=CANVAS_WIDTH/4;
double cursorY=CANVAS_HEIGHT/4;
int cameraX=CANVAS_WIDTH/2;
int caneraY=CANVAS_HEIGHT/2;

cv::Mat canvas;

class Stone{
private:
    double x,y,r,m,vx,vy;
public:
    Stone(double xx,double yy,double rr,double mm,double vxx,double vyy){
        x=xx;y=yy;r=rr;m=mm;vx=vxx;vy=vyy;
    }
    ~Stone(){

    }
    double getx(){
        return x;
    }
    double gety(){
        return y;
    }
    double getvx(){
        return vx;
    }
    double getvy(){
        return vy;
    }
    void setx(double xx){
        x=xx;
    }
    void sety(double yy){
        y=yy;
    }
    void setvx(double vxx){
        vx=vxx;
    }
    void setvy(double vyy){
        vy=vyy;
    }
};

class Gravity{
private:
    double x,y,vx,vy,G;
public:
    Gravity(double xx,double yy,double vxx,double vyy,double gg){
        x=xx;y=yy;vx=vxx;vy=vyy;G=gg;
    }
    ~Gravity(){

    }
    double getx(){
        return x;
    }
    double gety(){
        return y;
    }
    double getvx(){
        return vx;
    }
    double getvy(){
        return vy;
    }
    double getG(){
        return G;
    }
    void setx(double xx){
        x=xx;
    }
    void sety(double yy){
        y=yy;
    }
    void setvx(double vxx){
        vx=vxx;
    }
    void setvy(double vyy){
        vy=vyy;
    }
    void setG(double gg){
        G=gg;
    }
    void draw(){
        for(int i=0;i<3;i++){
            for(int j=1;j<5;j++){
                circle(canvas,cv::Point(x,y),G*j*j/(2*i+17),cv::Scalar(0xff,i*100+40,0));
            }
        }
    }
};

vector<Gravity> gravitylist;

void gravitydisplay(){

gravitylist.push_back(Gravity(200,300,0,0,200));
gravitylist.push_back(Gravity(400,400,0,0,100));
for(int hoge=0;true;){
    cv::Mat img(cv::Size(CANVAS_WIDTH,CANVAS_HEIGHT),CV_8UC3,cv::Scalar(0,0,0));
    canvas=img;

    double fx=0,fy=0;
    for(int i=0;i<gravitylist.size();i++){
        double gx,gy,cx,cy,l,G;
        gx=gravitylist[i].getx();
        gy=gravitylist[i].gety();
        double newy=gy+10;
        if(newy>CANVAS_HEIGHT)newy=newy-CANVAS_HEIGHT;
        gravitylist[i].sety(newy);
        G=gravitylist[i].getG();
        cx=cursorX;
        cy=cursorY;
        l=sqrt((gx-cx)*(gx-cx)+(gy-cy)*(gy-cy));

        fx+=G*(gx-cx)/(l*l*l);
        fy+=G*(gy-cy)/(l*l*l);
    }

    for(int i=0;i<gravitylist.size();i++){
        gravitylist[i].draw();
    }

    for(int i=0;i<gravitylist.size();i++){
        for(int j=0;j<gravitylist.size();j++){
            if(i!=j){
                double gxi,gyi,Gi,gxj,gyj,Gj;
                gxi=gravitylist[i].getx();
                gyi=gravitylist[i].gety();
                Gi=gravitylist[i].getG();
                gxj=gravitylist[j].getx();
                gyj=gravitylist[j].gety();
                Gj=gravitylist[j].getG();
            }    
        }
    }
    double d=sqrt(fx*fx+fy*fy);
    d=d*100;
    cout<<"fx,fy,d : "<<fx<<" "<<fy<<" "<<d<<endl;
    circle(img,cv::Point(cursorX,cursorY),10,cv::Scalar(((int)d*10)%256,0xff,0));

    cv::imshow("gravity",img);
    img.cv::Mat::release();


    //cv::waitKey(17);
    /*
    switch(cv::waitKey(0)){
        case 97:
            cursorX=cursorX-1/(d*d);
            if(cursorX<0)cursorX=cursorX+CANVAS_WIDTH;
            break;
        case 100:
            cursorX=cursorX+1/(d*d);
            if(cursorX>CANVAS_WIDTH)cursorX=cursorX-CANVAS_WIDTH;
            break;
        case 119://w
            cursorY=cursorY-1/(d*d);
            if(cursorY<0)cursorY+=CANVAS_HEIGHT;
            break;
        case 115://s
            cursorY=cursorY+1/(d*d);
            if(cursorY>CANVAS_HEIGHT)cursorY=cursorY-CANVAS_HEIGHT;
            break;

    }
    */
   std::this_thread::sleep_for(std::chrono::milliseconds(100));

}
}

void line(double x1,double y1,double x2,double y2){
    double r=sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    if(r>3){
        double hx,hy,vx,vy;
        hx=x2-x1;
        hy=y2-y1;
        vx=-hy;
        vy=hx;
        line(x1,y1,x1+hx/3,y1+hy/3);
        line(x1+hx/3,y1+hy/3,x1+hx/3+vx/3,y1+hy/3+vy/3);
        line(x1+hx/3+vx/3,y1+hy/3+vy/3,x2-hx/3+vx/3,y2-hy/3+vy/3);
        line(x2-hx/3+vx/3,y2-hy/3+vy/3,x2-hx/3,y2-hy/3);
        line(x2-hx/3,y2-hy/3,x2,y2);
    }else{
        if(((x1<0||x1>CANVAS_WIDTH)|(y1<0||y1>CANVAS_HEIGHT))
        &&((x2<0||x2>CANVAS_WIDTH)||(y2<0||y2>CANVAS_HEIGHT))){
        
        }else{
            cv::line(canvas,cv::Point(x1,y1),cv::Point(x2,y2),cv::Scalar(0xff,0xff,0xff));
        }
    }
}

class Maze{
private:
    double x,y,l,th,zoom;
public:
    Maze(double xx,double yy,double ll,double tt,double zz){
        x=xx;y=yy;l=ll;th=tt;zoom=zz;
    }
    ~Maze(){

    }
    double getx(){
        return x;
    }
    double gety(){
        return y;
    }
    double getl(){
        return l;
    }
    double getth(){
        return th;
    }
    double getzoom(){
        return zoom;
    }
    double getcornerx(int i){
        return x+l/2*zoom*cos(th-PI/2*i);
    }
    double getcornery(int i){
        return y+l/2*zoom*sin(th-PI/2*i);
    }
    void setx(double xx){
        x=xx;
    }
    void sety(double yy){
        y=yy;
    }
    void setl(double ll){
        l=ll;
    }
    void setth(double tt){
        th=tt;
    }
    void setzoom(double zz){
        zoom=zz;
    }
    void draw(){
        //cout<<"maze"<<endl;
        line(x+l/2*zoom*cos(th),y+l/2*zoom*sin(th),x+l/2*zoom*cos(th+PI/2),y+l/2*zoom*sin(th+PI/2));
        line(x+l/2*zoom*cos(th+PI/2),y+l/2*zoom*sin(th+PI/2),x+l/2*zoom*cos(th+PI),y+l/2*zoom*sin(th+PI));
        line(x+l/2*zoom*cos(th+PI),y+l/2*zoom*sin(th+PI),x+l/2*zoom*cos(th+3*PI/2),y+l/2*zoom*sin(th+3*PI/2));
        line(x+l/2*zoom*cos(th+3*PI/2),y+l/2*zoom*sin(th+3*PI/2),x+l/2*zoom*cos(th),y+l/2*zoom*sin(th));
    }
};

vector<Maze> mazelist;

void zoomInitialize(double *x,double *y,double *r){
    //double xx;
    
    for(int i=0;i<mazelist.size();i++){
        double mx,my,mcx,mcy,cx,cy,a;
        mx=mazelist[i].getx();
        my=mazelist[i].gety();
        cx=cursorX;
        cy=cursorY;
        if(cy==my){
            double s,t;
            vector<int> indexlist={0,1,2,3};
            
            for(int j=0;j<4;j++){
                s=mazelist[i].getcornerx(j);
                for(int k=j+1;k<4;k++){
                    t=mazelist[i].getcornerx(k);
                    if(s<t){
                        int u=indexlist[j];
                        indexlist[j]=indexlist[k];
                        indexlist[k]=u;
                    }
                }
            }

            if(cx-mx>0){
                mcx=mazelist[i].getcornerx(indexlist[0]);
                mcy=cy;
            }else{
                mcx=mazelist[i].getcornerx(indexlist[3]);
                mcy=cy;
            }

            *x=mcx;
            *y=mcy;
        }else if(cx==mx){
            double s,t;
            vector<int>indexlist{0,1,2,3};

            for(int j=0;j<4;j++){
                s=mazelist[i].getcornery(j);
                for(int k=0;k<4;k++){
                    t=mazelist[i].getcornery(k);
                    if(s<t){
                        int u=indexlist[j];
                        indexlist[j]=indexlist[k];
                        indexlist[k]=u;
                    }
                }
            }

            if(cy-my>0){
                mcx=cx;
                mcy=mazelist[i].getcornery(indexlist[0]);
            }else{
                mcx=cx;
                mcy=mazelist[i].getcornery(indexlist[3]);
            }

            *x=mcx;
            *y=mcy;        
        }else{
            a=(cy-my)/(cx-mx);
            double s,t;
            vector<int> indexlist={0,1,2,3};
            
            for(int j=0;j<4;j++){
                s=1/a*mazelist[i].getcornerx(j)+mazelist[i].getcornery(j);
                for(int k=j+1;k<4;k++){
                    t=1/a*mazelist[i].getcornerx(k)+mazelist[i].getcornery(k);
                    if(s<t){
                        int u=indexlist[j];
                        indexlist[j]=indexlist[k];
                        indexlist[k]=u;
                    }
                }
            }
            
            //cout<<"indexlist : "<<indexlist[0]<<" "<<indexlist[3]<<endl;
            if((cx-mx>=0&&cy-my>=0)||(cx-mx<0&&cy-my>=0)){//a<0&& || a>=0
                mcx=mazelist[i].getcornerx(indexlist[0]);
                mcy=mazelist[i].getcornery(indexlist[0]);
            }else{
                mcx=mazelist[i].getcornerx(indexlist[3]);
                mcy=mazelist[i].getcornery(indexlist[3]);
            }

            *x=a/(a*a+1)*(a*mx+1/a*mcx-(my-mcy));
            *y=a/(a*a+1)*(1/a*my+a*mcy-(mx-mcx));
        }

        //if(*r<10){
        *r=(100/((*x-cx)*(*x-cx)+(*y-cy)*(*y-cy)));
        //}else{
        //    *r=10;
        //}
    }
}

double zoomX,zoomY,testr;

void test(){
for(int hoge=0;true;){
    cv::Mat img(cv::Size(CANVAS_WIDTH,CANVAS_HEIGHT),CV_8UC3,cv::Scalar(0,0,0));
    canvas=img;

    zoomInitialize(&zoomX,&zoomY,&testr);
    cout<<"testr : "<<testr<<endl;
    
    if(testr>100){
        testr=100;
    }

    cv::circle(img,cv::Point(zoomX,zoomY),2,cv::Scalar(0,0xff,0),1);
    
    double mx,my,ml,mth,mzoom,fx,fy,fl;

    for(int i=0;i<mazelist.size();i++){
        mx=mazelist[i].getx();
        my=mazelist[i].gety();
        ml=mazelist[i].getl();
        mth=mazelist[i].getth();
        mzoom=mazelist[i].getzoom();

        fx=mx+(mx-zoomX)*testr;
        fy=my+(my-zoomY)*testr;
        fl=ml*(1+testr);

        mazelist[i].setx(fx);
        mazelist[i].sety(fy);
        mazelist[i].setl(fl);

        mazelist[i].draw();

        mazelist[i].setx(mx);
        mazelist[i].sety(my);
        mazelist[i].setl(ml);
    }

    double cx,cy;
    cx=cursorX+(cursorX-zoomX)*testr;
    cy=cursorY+(cursorY-zoomY)*testr;
    cv::circle(img,cv::Point(cx,cy),10,cv::Scalar(0xff,0,0xff),-1);

    cv::imshow("test",img);
    img.cv::Mat::release();
    double rr=testr;
    if(testr<0.1){
        rr=0.1;
    }else if(testr>1000){
        rr=1000;
    }else{
        rr=testr;
    }
    switch(cv::waitKey(0)){
        case 97:
            cursorX=cursorX-1/rr;
            if(cursorX<0)cursorX+=CANVAS_WIDTH;
            break;
        case 100:
            cursorX=cursorX+1/rr;
            if(cursorX>CANVAS_WIDTH)cursorX=cursorX-CANVAS_WIDTH;
            break;
        case 119://w
            cursorY=cursorY-1/rr;
            if(cursorY<0)cursorY+=CANVAS_HEIGHT;
            break;
        case 115://s
            cursorY=cursorY+1/rr;
            if(cursorY>CANVAS_HEIGHT)cursorY=cursorY-CANVAS_HEIGHT;
            break;
        
    }
}
}

void ThreadA(){
for(int hoge=0;true;){
    cout<<"thread A"<<endl;
    sleep(1);
}
}
void ThreadB(){
for(int hoge=0;true;){
    cout<<"thread B"<<endl;
    cv::Mat img(cv::Size(CANVAS_WIDTH,CANVAS_HEIGHT),CV_8UC3,cv::Scalar(0,0,0));
    cv::imshow("testimg",img);
    img.cv::Mat::release();
    cv::waitKey(0);
}
}

int main(){
    mazelist.push_back(Maze(CANVAS_WIDTH/2,CANVAS_HEIGHT/2,600,PI/4,1));
    //mazelist.push_back(Maze(200,480,100,PI/4,1));
    
    //test();
    //
    std::thread th_a(ThreadA);
    std::thread th_b(ThreadB);

    th_a.join();
    th_b.join();

    //gravitydisplay();
    return 0;
}