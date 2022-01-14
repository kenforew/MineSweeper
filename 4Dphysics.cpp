#include <conio.h>
#include <iostream>
#include <string>
#include <typeinfo>
#include <math.h>
#include <random>
#include <vector>
#include <algorithm>

#include <cstdlib>
#include <Windows.h>
#include <unistd.h>

#include <chrono>
#include <thread>

#include <vec_op.hpp>

#include <opencv2/opencv.hpp>

using namespace std;

class GrobalVariables{
private:
    int CANVAS_WIDTH;
    int CANVAS_HEIGHT;

    bool frameFlag;

    int spf;
public:
    GrobalVariables(){
        CANVAS_WIDTH=800;
        CANVAS_HEIGHT=400;

        frameFlag=false;
        spf=33;
    }
    ~GrobalVariables(){

    }
    int getCW(){
        return CANVAS_WIDTH;
    }
    int getCH(){
        return CANVAS_HEIGHT;
    }
    bool getframeFlag(){
        return frameFlag;
    }
    int getspf(){
        return spf;
    }
};

GrobalVariables GV;
class Camera{
private:
    double w,x,y,z,sw,sx,sy,sz,speed;
    vector<int> gravity;
public:
    Camera(){
        gravity={0,0,0,-1};
        w=0;x=0;y=0;z=-1;
        sw=0;sx=1.0;sy=0;sz=0;
        speed=2.0;
    }
    ~Camera(){}
    
    void setw(double ww){
        w=ww;
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
    void setsw(double ww){
        sw=ww;
    }
    void setsx(double xx){
        sx=xx;
    }
    void setsy(double yy){
        sy=yy;
    }
    void setsz(double zz){
        sz=zz;
    }
    void setspeed(double ss){
        speed=ss;
    }

    double getw(){
        return w;
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
    vector<double> getX(){
        return {w,x,y,z};
    }
    
    double getsw(){
        return sw;
    }
    double getsx(){
        return sx;
    }
    double getsy(){
        return sy;
    }
    double getsz(){
        return sz;
    }
    vector<double> getSX(){
        return {sw,sx,sy,sz};
    }
    
    double getspeed(){
        return speed;
    }
};

Camera camera;

class HyperCube{
private:
    vector<vector<double>> x,I;
    vector<double>ulist,xlist,ylist,zlist;
    vector<double>g,v,w;
    double m,e,sfc,dfc;
public:
    HyperCube(vector<vector<double>> xx,vector<double> vv,vector<double> ww,double mm,double ee,double ssfc,double ddfc){
        copy(xx.begin(),xx.end(),x);
        copy(vv.begin(),vv.end(),v);
        copy(ww.begin(),ww.end(),w);
        for(int i=0;i<4;i++){
            double gg=0;
            for(int j=0;j<4;j++){
                gg+=x[j][i];
            }
            g.push_back(gg/4);
            ulist.push_back(xx[i][0]);
            xlist.push_back(xx[i][1]);
            ylist.push_back(xx[i][2]);
            zlist.push_back(xx[i][3]);
        }
        m=mm;e=ee;sfc=ssfc;dfc=ddfc;
    }
    ~HyperCube(){}

    vector<vector<double>> getPos(){
        return x;
    }
    vector<double>getulist(){
        return ulist;
    }
    vector<double>getxlist(){
        return xlist;
    }
    vector<double>getylist(){
        return ylist;
    }
    vector<double>getzlist(){
        return zlist;
    }

    vector<double>getG(){
        return g;
    }
    vector<double>getV(){
        return v;
    }
    vector<double>getW(){
        return w;
    }

    double getM(){
        return m;
    }
    double getE(){
        return e;
    }
    double getsfc(){
        return sfc;
    }
    double getdfc(){
        return dfc;
    }
    void setG(vector<double>gg){
        g=gg;
    }
    void setV(vector<double>vv){
        v=vv;
    }
    void setW(vector<double>ww){
        w=ww;
    }

    void update(){
        g=g+v;
    }

    void shiftColPos(vector<double>shift){
        
    }
};

void gameInitialize(){


    gameDisplay();
}

void gameDisplay(){
//    GV.setframeFlag(false);
HyperCube hc=HyperCube(
    {
        {1,1,1,1},{1,1,1,-1},{1,1,-1,1},{1,1,-1,-1},
        {1,-1,1,1},{1,-1,1,-1},{1,-1,-1,1},{1,-1,-1,-1},
        {-1,1,1,1},{-1,1,1,-1},{-1,1,-1,1},{-1,1,-1,-1},
        {-1,-1,1,1},{-1,-1,1,-1},{-1,-1,-1,1},{-1,-1,-1,-1}
    },{0,0,0,0},{0,0,0,0},1,1,1,1);

while(1){
    cv::Mat img(cv::Size(GV.getCW(),GV.getCH()),CV_8UC3,cv::Scalar(0,0,0));

    //4D->3D
    //


    //3D->2D
    vector<double>a=camera.getSX(),b=camera.getX();
    
    vector<cv::Point>plist;
    bool drawflag=false;
    bool inframe=true;
    
    for(int i=0;i<16;i++){
        vector<double>vv=hc.getPos()[i],O_yz;
        double t=vec_op::dot(a,vv-b);
        O_yz=t*a+b;

        double l=vec_op::norm(vv-O_yz);

        if(l!=0){
            double costh,th;
            vector<double>X2={0.0,0.0,0.0},G,Y2;
            G={0,0,0,-1};//camera.getG();
            Y2=vec_op::cross(G,a);
            double ly=vec_op::norm(Y2);
            costh=vec_op::dot(Y2,vv-O_yz);
            th=acos(costh);
            X2[0]=t;
            int sgn=1;
            int Ge=G[0]+G[1]+G[2];

            sgn=vec_op::dot(G,vv)>vec_op::dot(G,O_yz)?Ge:-Ge;

            X2[1]=l*cos(th);
            X2[2]=l*sgn*sin(th);

            plist.push_back(
                cv::Point(
                    GV.getCW()/2+X2[1]*(1000/sqrt((t*t+1))),
                    GV.getCH()/2-X2[2]*(1000/sqrt((t*t+1)))
                )
            );

            if(t>1){
                drawflag=drawflag||false;
            }else{
                drawflag=drawflag||true;
            }
        }else{
            plist.push_back(
                cv::Point(
                    GV.getCW()/2,GV.getCH()/2
                )
            );
        }
    }

    if(!drawflag){
        polylines(img,plist,true,cv::Scalar(0xff,0xff,0xff),1,cv::LINE_AA);
    }
    
    
}
}

int main(){
    std::thread th_main(gameInitialize);
    //std::thread th_time(timeCounter);

    th_main.join();
    //th_time.join();
    //
    return 0;
}