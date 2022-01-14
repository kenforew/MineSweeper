//#ifndef 3D_PHYSICS
//#define 3D_PHYSICS

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

#include "vec_op.hpp"
#include "mat_op.hpp"
#include "phy_op.hpp"
#include "const.hpp"

#include <opencv2/opencv.hpp>


namespace Math{
    const double PI=3.1415926535;
};

class GrobalVariables{
private:
    int CANVAS_WIDTH;
    int CANVAS_HEIGHT;
    
    int cursorX;
    int cursorY;

    int mouseUpdateX;
    int mouseUpdateY;

    bool frameFlag;

    double GX,GY,GZ;

    int spf;

    bool test;

public:
    GrobalVariables(){
        CANVAS_WIDTH=800;
        CANVAS_HEIGHT=800;

        cursorX=0;
        cursorY=0;

        mouseUpdateX=0;
        mouseUpdateY=0;

        GX=0;GY=0;GZ=-0.5;

        spf=333;

        test=false;
    }
    ~GrobalVariables(){

    }
    void setcursorX(int xx){
        cursorX=xx;
    }
    void setcursorY(int yy){
        cursorY=yy;
    }
    void setmouseUpdateX(int xx){
        mouseUpdateX=xx;
    }
    void setmouseUpdateY(int yy){
        mouseUpdateY=yy;
    }
    void setframeFlag(bool ff){
        frameFlag=ff;
}
    int getCW(){
        return CANVAS_WIDTH;
    }
    int getCH(){
        return CANVAS_HEIGHT;
    }
    int getcursorX(){
        return cursorX;
    }
    int getcursorY(){
        return cursorY;
    }
    int getmouseUpdateX(){
        return mouseUpdateX;
    }
    int getmouseUpdateY(){
        return mouseUpdateY;
    }
    bool getframeFlag(){
        return frameFlag;
    }
    double getGX(){
        return GX;
    }
    double getGY(){
        return GY;
    }
    double getGZ(){
        return GZ;
    }
    std::vector<double> getG(){
        return {GX,GY,GZ};
    }
    int getspf(){
        return spf;
    }

    bool gettest(){
        return test;
    }
    void settest(bool tt){
        test=tt;
    }
};

GrobalVariables GV;

class Camera{
private:
    double x,y,z,sx,sy,sz;
    std::vector<int> gravity;
    double speed;
public:
    Camera(){
        gravity={0,0,-1};
        x=-10*gravity[0];y=-10*gravity[1];z=-10*gravity[2];
        sx=1.0;sy=0.0;sz=0.0;
        speed=2.0;
    }
    ~Camera(){}

    void setx(double xx){
        x=xx;
    }
    void sety(double yy){
        y=yy;
    }
    void setz(double zz){
        z=zz;
    }
    void setsx(double sxx){
        sx=sxx;
    }
    void setsy(double syy){
        sy=syy;
    }
    void setsz(double szz){
        sz=szz;
    }
    void setgravity(int i,double gg){
        gravity[i]=gg;
    }
    void setspeed(double ss){
        speed=ss;
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
    double getsx(){
        return sx;
    }
    double getsy(){
        return sy;
    }
    double getsz(){
        return sz;
    }
    std::vector<double> getsv(){
        return {sx,sy,sz};
    }
    double getgravity(int i){
        return gravity[i];
    }
    double getspeed(){
        return speed;
    }
};

Camera camera;

class Triangle{
private:
    std::vector<std::vector<double>> points;//{{x1,y1,z1},{x2,y2,z2}, ... };
public:
    Triangle(std::vector<std::vector<double>> pp){
        if(pp.size()==3){
            points=pp;
        }else{
            std::cout<<"wrong input."<<std::endl;
        }
    }
    ~Triangle(){}

    std::vector<std::vector<double>> getpoints(){
        return points;
    }
};
class Tetrahedron{
private:
    std::vector<std::vector<double>> corner,I,aI,Ig,aIg;
    std::vector<double> xlist,ylist,zlist,g,v,w;
    double m,e,sfc,dfc;
    bool stop;
public:
    Tetrahedron(std::vector<std::vector<double>> cc,std::vector<double> vv,std::vector<double> ww,double mm,double ee,double ssfc,double ddfc){
        double x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4;
        double xg,yg,zg;
        x1=cc[0][0];y1=cc[0][1];z1=cc[0][2];
        x2=cc[1][0];y2=cc[1][1];z2=cc[1][2];
        x3=cc[2][0];y3=cc[2][1];z3=cc[2][2];
        x4=cc[3][0];y4=cc[3][1];z4=cc[3][2];
	
        xg=(x1+x2+x3+x4)/4;
        yg=(y1+y2+y3+y4)/4;
        zg=(z1+z2+z3+z4)/4;
        
        g={xg,yg,zg};
        
        v=vv;
        w=ww;
	
        x1=x1-xg;x2=x2-xg;x3=x3-xg;x4=x4-xg;
        y1=y1-yg;y2=y2-yg;y3=y3-yg;y4=y4-yg;
        z1=z1-zg;z2=z2-zg;z3=z3-zg;z4=z4-zg;

        xlist={x1,x2,x3,x4};
        ylist={y1,y2,y3,y4};
        zlist={z1,z2,z3,z4};

        corner={
            {x1,y1,z1},
            {x2,y2,z2},
            {x3,y3,z3},
            {x4,y4,z4},
        };

        m=mm;e=ee;sfc=ssfc;dfc=ddfc;

        std::vector<std::vector<double>>II={{0,0,0},{0,0,0},{0,0,0}};
        for(int i=0;i<4;i++){
            for(int j=i+1;j<4;j++){
                for(int k=j+1;k<4;k++){
                    II=II+phy_op::I(corner[i],corner[j],corner[k],m);
                }
            }
        }
        I=II;
        
        //cout<<"this is Inertia. : "<<endl;
        //for(int i=0;i<3;i++){
        //    for(int j=0;j<3;j++){
        //        cout<<i<<", "<<j<<" : "<<I[i][j]<<" ";
        //    }
        //    cout<<endl;
        //}

        aI=mat_op::inverse(I);
        //cout<<"this is inverse Inertia. : "<<endl;
        //for(int i=0;i<3;i++){
        //    for(int j=0;j<3;j++){
        //        cout<<i<<", "<<j<<" : "<<aI[i][j]<<" ";
        //    }
        //    cout<<endl;
        //}

        Ig=I+m*mat_op::dot(mat_op::cross(g),mat_op::cross(g));
        aIg=mat_op::inverse(Ig);

        stop=false;

    }
    ~Tetrahedron(){}

    std::vector<double> getxlist(){
        return xlist;
    }
    std::vector<double> getylist(){
        return ylist;
    }
    std::vector<double> getzlist(){
        return zlist;
    }
    std::vector<std::vector<double>> getcorner(){
        return corner;
    }

    std::vector<double> getG(){
        return g;
    }
    std::vector<double> getV(){
        return v;
    }
    std::vector<double> getW(){
        return w;
    }
    double getM(){
        return m;
    }
    double getE(){
        return e;
    }
    std::vector<std::vector<double>> getI(){
        return I;
    }
    std::vector<std::vector<double>> getaI(){
        return aI;
    }
    std::vector<std::vector<double>> getIg(){
        return Ig;
    }
    std::vector<std::vector<double>> getaIg(){
        return aIg;
    }
    double getsfc(){
        return sfc;
    }
    double getdfc(){
        return sfc;
    }
    bool getstop(){
        return stop;
    }

    void setG(std::vector<double> gg){
        g=gg;
    }
    void setV(std::vector<double> vv){
        v=vv;
    }
    void setW(std::vector<double> ww){
        w=ww;
    }
    void setstop(bool ss){
        stop=ss;
        v=0.0*v;
        w=0.0*w;
    }
    
    void update();
    void shiftColPos(std::vector<double>);
    bool AABB(Tetrahedron *);
    double distDots(std::vector<double>,std::vector<double>);
    double distLine(std::vector<double>,std::vector<double>,std::vector<double>);
    double distPlane(std::vector<double>,std::vector<double>,std::vector<double>,std::vector<double>);
    bool is_inTriangle(std::vector<double>,std::vector<double>,std::vector<double>,std::vector<double>);
    bool is_inSameSide(std::vector<double>,std::vector<double>,std::vector<double>,std::vector<double>,std::vector<double>);
    std::vector<double> getNVecLine(std::vector<double>,std::vector<double>,std::vector<double>);
    std::vector<double> getNVecPlane(std::vector<double>,std::vector<double>,std::vector<double>,std::vector<double>);
    std::vector<double> GJK_EPA_force(Tetrahedron *);
    void Collision(Tetrahedron *);
    void addJoint(std::vector<double>);
    void removeJoint(int);

};

void Tetrahedron::VWUpdate(double ratio){
    v=v+ratio*GV.getG();
}
void Tetrahedron::PosUpdate(double ratio){
    g=g+ratio*v;
}
void Tetrahedron::RotUpdate(){
    double xx1,yy1,zz1,th;
    th=vec_op::norm(w);
    if(w[0]<0)th=-th;
    if(w[1]<0)th=-th;
    if(w[2]<0)th=-th;

    if(th!=0){
    std::vector<double> nn=(1/th)*w;//{wx/th,wy/th,wz/th};

    th=th/180;
    std::vector<std::vector<double>> RotMat=
    {
        {
            cos(th)+nn[0]*nn[0]*(1-cos(th)),
            nn[0]*nn[1]*(1-cos(th))-nn[2]*sin(th),
            nn[2]*nn[0]*(1-cos(th))+nn[1]*sin(th)
        },
        {
            nn[0]*nn[1]*(1-cos(th))+nn[2]*sin(th),
            cos(th)+nn[1]*nn[1]*(1-cos(th)),
            nn[1]*nn[2]*(1-cos(th))-nn[0]*sin(th)
        },
        {
            nn[2]*nn[0]*(1-cos(th))-nn[1]*sin(th),
            nn[1]*nn[2]*(1-cos(th))+nn[0]*sin(th),
            cos(th)+nn[2]*nn[2]*(1-cos(th))
        }
    };
    xlist.clear();
    ylist.clear();
    zlist.clear();

    std::vector<std::vector<double>> escape=corner;

    corner.clear();

    for(int i=0;i<4;i++){
        std::vector<double>vv;
        vv=vec_op::mat(RotMat,escape[i]);
        //xx1=0;yy1=0;zz1=0;
        //for(int j=0;j<3;j++){
        //    xx1+=RotMat[0][j]*escape[i][j];
        //    yy1+=RotMat[1][j]*escape[i][j];
        //    zz1+=RotMat[2][j]*escape[i][j];
        //}
        xx1=vv[0];yy1=vv[1];zz1=vv[2];
        //x1=xx1;y1=yy1;z1=zz1;
        xlist.push_back(xx1);
        ylist.push_back(yy1);
        zlist.push_back(zz1);
        corner.push_back({xx1,yy1,zz1});            
    }
    }
}

void Tetrahedron::shiftColPos(std::vector<double> shift){
    g=g+shift;
}

bool Tetrahedron::AABB(Tetrahedron *B){
    double minAX,minAY,minAZ,maxAX,maxAY,maxAZ;
    minAX=*min_element(xlist.begin(),xlist.end())+getG()[0];
    maxAX=*max_element(xlist.begin(),xlist.end())+getG()[0];
    minAY=*min_element(ylist.begin(),ylist.end())+getG()[1];
    maxAY=*max_element(ylist.begin(),ylist.end())+getG()[1];
    minAZ=*min_element(zlist.begin(),zlist.end())+getG()[2];
    maxAZ=*max_element(zlist.begin(),zlist.end())+getG()[2];

    double minBX,minBY,minBZ,maxBX,maxBY,maxBZ;
    minBX=*min_element(xlist.begin(),xlist.end())+(*B).getG()[0];
    maxBX=*max_element(xlist.begin(),xlist.end())+(*B).getG()[0];
    minBY=*min_element(ylist.begin(),ylist.end())+(*B).getG()[1];
    maxBY=*max_element(ylist.begin(),ylist.end())+(*B).getG()[1];
    minBZ=*min_element(zlist.begin(),zlist.end())+(*B).getG()[2];
    maxBZ=*max_element(zlist.begin(),zlist.end())+(*B).getG()[2];

    bool flag=true;
    if(
        maxBX<minAX||maxAX<minBX||
        maxBY<minAY||maxAY<minBY||
        maxBZ<minAZ||maxAZ<minBZ
    ){
        flag=flag&&false;
    }

    return flag;
}

double Tetrahedron::distDots(std::vector<double>p1,std::vector<double>p2){
    return vec_op::norm(p2-p1);
}
double Tetrahedron::distLine(std::vector<double>p1,std::vector<double>p2,std::vector<double>q){
    double a;
    std::vector<double>dv=p2-p1;
    a=vec_op::dot(dv,dv);

    if(a==0){
        return distDots(p1,q);
    }

    double b,t;//,x,y,z;
    std::vector<double>foot;
    b=vec_op::dot(dv,p1)-vec_op::dot(dv,q);
    t=-b/a;
    if(t<0.0)t=0.0;
    if(t>1.0)t=1.0;
    foot=p1+t*dv;
    return vec_op::norm(q-foot);
}

double Tetrahedron::distPlane(std::vector<double>p1,std::vector<double>p2,std::vector<double>p3,std::vector<double>q){
	
    std::vector<double>Avec=p2-p1,Bvec=p3-p1;//{p3[0]-p1[0],p3[1]-p1[1],p3[2]-p1[2]};

    double det=vec_op::dot(Avec,Avec)*vec_op::dot(Bvec,Bvec)-pow(vec_op::dot(Avec,Bvec),2);
    if(det==0){
        return distLine(p1,p2,q);
    }
    double s,t;//,x,y,z;
    std::vector<double>dv;
    s=(vec_op::dot(Bvec,Bvec)*vec_op::dot(q-p1,Avec)-vec_op::dot(Avec,Bvec)*vec_op::dot(q-p1,Bvec))/det;
    t=(-vec_op::dot(Avec,Bvec)*vec_op::dot(q-p1,Avec)+vec_op::dot(Avec,Avec)*vec_op::dot(q-p1,Bvec))/det;
    
    if(s+t>1&&s>0&&t>0){
        return distLine(p2,p3,q);
    }else if(s+t<1&&s>0&&t<0){
        return distLine(p1,p2,q);
    }else if(s+t<1&&s<0&&t>0){
        return distLine(p1,p3,q);
    }else if(s<0&&t<0){
        return distDots(p1,q);
    }else if(s+t>1&&t<0){
        return distDots(p2,q);
    }else if(s+t>1&&s<0){
        return distDots(p3,q);
    }else{
        dv=p1+s*Avec+t*Bvec;

        return distDots(q,dv);
    }
}

bool Tetrahedron::is_inTriangle(std::vector<double>p1,std::vector<double>p2,std::vector<double>p3,std::vector<double>q){
    std::vector<double>Avec=p2-p1,Bvec=p3-p1;//{p3[0]-p1[0],p3[1]-p1[1],p3[2]-p1[2]};

    double det=vec_op::dot(Avec,Avec)*vec_op::dot(Bvec,Bvec)-pow(vec_op::dot(Avec,Bvec),2);
    if(det==0){
        return false;
    }

    double s,t;//,x,y,z;
    s=(vec_op::dot(Bvec,Bvec)*vec_op::dot(q-p1,Avec)-vec_op::dot(Avec,Bvec)*vec_op::dot(q-p1,Bvec))/det;
    t=(-vec_op::dot(Avec,Bvec)*vec_op::dot(q-p1,Avec)+vec_op::dot(Avec,Avec)*vec_op::dot(q-p1,Bvec))/det;
    
    if(s+t<1&&s>0&&t>0){
        return true;
    }else{
        return false;
    }
}

bool Tetrahedron::is_inSameSide(std::vector<double>p1,std::vector<double>p2,std::vector<double>p3,std::vector<double>q1,std::vector<double>q2){
    std::vector<double>cp=vec_op::cross(p2-p1,p3-p1);
    
    if(vec_op::dot(q1-p1,cp)*vec_op::dot(q2-p1,cp)>0){
        return true;
    }else{
        return false;
    }
    //is q1,q2 in the same side of triangle(p1,p2,p3)?
}

std::vector<double> Tetrahedron::getNVecLine(std::vector<double>p1,std::vector<double>p2,std::vector<double>q){
    std::vector<double>dv=p2-p1;
    double a=vec_op::dot(dv,dv);
    
    if(a==0){
        return p1-q;//{p1[0]-q[0],p1[1]-q[1],p1[2]-q[2]};
    }

    double b,t;
    std::vector<double>ans;
    b=vec_op::dot(dv,p1-q);
    t=-b/a;
    if(t<0.0)t=0.0;
    if(t>1.0)t=1.0;
    ans=p1+t*dv-q;
    return ans;
}

std::vector<double> Tetrahedron::getNVecPlane(std::vector<double>p1,std::vector<double>p2,std::vector<double>p3,std::vector<double>q){
    std::vector<double>Avec=p2-p1,Bvec=p3-p1;//{p3[0]-p1[0],p3[1]-p1[1],p3[2]-p1[2]};

    double det=vec_op::dot(Avec,Avec)*vec_op::dot(Bvec,Bvec)-pow(vec_op::dot(Avec,Bvec),2);
    if(det==0){
        return getNVecLine(p1,p2,q);
    }
    double s,t;
    //std::vector<double>ans;
    s=(vec_op::dot(Bvec,Bvec)*vec_op::dot(q-p1,Avec)-vec_op::dot(Avec,Bvec)*vec_op::dot(q-p1,Bvec))/det;
    t=(-vec_op::dot(Avec,Bvec)*vec_op::dot(q-p1,Avec)+vec_op::dot(Avec,Avec)*vec_op::dot(q-p1,Bvec))/det;
    //std::cout<<"getNVecPlane"<<endl;
    if(s+t>1&&s>0&&t>0){
        //std::cout<<"s+t>1 s>0 t>0"<<endl;
        return getNVecLine(p2,p3,q);
    }else if(s+t<1&&s>0&&t<0){
        //std::cout<<"s+t<1 s>0 t<0"<<endl;
        return getNVecLine(p1,p2,q);
    }else if(s+t<1&&s<0&&t>0){
        //std::cout<<"s+t<1 s<0 t>0"<<endl;
        return getNVecLine(p1,p3,q);
    }else if(s<0&&t<0){
        //std::cout<<"s<0 t<0"<<endl;
        return p1;
    }else if(s+t>1&&t<0){
        //std::cout<<"s+t>1 t<0"<<endl;
        return p2;
    }else if(s+t>1&&s<0){
        //std::cout<<"s+t>1 s<0"<<endl;
        return p3;
    }else{
        //std::cout<<"s+t<1 s>0 t>0"<<endl;
        return p1+s*Avec+t*Bvec-q;
    }
}

std::vector<double> Tetrahedron::GJK_EPA_force(Tetrahedron *B){
    std::cout<<"GJK EPA force is called."<<std::endl;

    bool goEPAFlag=false;

    std::vector<double> 
    AtoO=(-1.0)*getG(),//{-xg,-yg,-zg},
    xnew={0.0,0.0,0.0};
    double da,db,dda,ddb;
    int ai,bi,ta,tb;
    std::vector<std::vector<double>>tetrahedron;
    ta=getcorner().size();
    tb=(*B).getcorner().size();
    for(int hoge=0;hoge<2;hoge++){
        dda=vec_op::dot(getcorner()[0]+getG(),AtoO);//(xlist[0]+getG()[0])*AtoO[0]+(ylist[0]+getG()[1])*AtoO[1]+(zlist[0]+getG()[2])*AtoO[2];
        ai=0;
        for(int i=0;i<ta;i++){
            da=vec_op::dot(getcorner()[i]+getG(),AtoO);//(xlist[i]+getG()[0])*AtoO[0]+(ylist[i]+getG()[1])*AtoO[1]+(zlist[i]+getG()[2])*AtoO[2];   
            if(da>dda){
                dda=da;
                ai=i;
            }
        }
        ddb=vec_op::dot((*B).getcorner()[0]+(*B).getG(),AtoO);//+((*B).getylist()[0]+getG()[1])*AtoO[1]+((*B).getzlist()[0]+getG()[2])*AtoO[2];
        bi=0;
        for(int i=0;i<tb;i++){
            db=vec_op::dot((*B).getcorner()[i]+(*B).getG(),AtoO);//+((*B).getylist()[i]+getG()[1])*AtoO[1]+((*B).getzlist()[i]+getG()[2])*AtoO[2];   
            if(db<ddb){
                ddb=db;
                bi=i;
            }
        }
        xnew=getcorner()[ai]+getG()-((*B).getcorner()[bi]+(*B).getG());
        //xnew={
        //    -((xlist[ai]+getG()[0])-((*B).getxlist()[bi]+(*B).getG()[0])),
        //    -((ylist[ai]+getG()[1])-((*B).getylist()[bi]+(*B).getG()[1])),
        //    -((zlist[ai]+getG()[2])-((*B).getzlist()[bi]+(*B).getG()[2]))
        //};

        tetrahedron.push_back(xnew);

        AtoO=(-1.0)*xnew;
        std::cout<<"AtoO : "<<std::endl;
        for(int i=0;i<3;i++){
            std::cout<<i<<" th element : "<<AtoO[i]<<std::endl;
        }
        //AtoO[0]=xnew[0];
        //AtoO[1]=xnew[1];
        //AtoO[2]=xnew[2];
    }

    std::cout<<"line segment"<<std::endl;
    for(int i=0;i<2;i++){
        std::cout<<"x,y,z : "<<tetrahedron[i][0]<<" "<<tetrahedron[i][1]<<" "<<tetrahedron[i][2]<<std::endl;
    }

    std::vector<double>second=getNVecLine(tetrahedron[0],tetrahedron[1],{0,0,0});
    std::cout<<"AtoO std::vector : "<<std::endl;
    for(int i=0;i<3;i++){
        std::cout<<i<<" th element : "<<-second[i]<<std::endl;
    }
    AtoO=(-1.0)*second;
    //AtoO[0]=-second[0];
    //AtoO[1]=-second[1];
    //AtoO[2]=-second[2];

    dda=vec_op::dot(getcorner()[0]+getG(),AtoO);//+(ylist[0]+getG()[1])*AtoO[1]+(zlist[0]+getG()[2])*AtoO[2];
    ai=0;
    for(int i=0;i<ta;i++){
        da=vec_op::dot(getcorner()[i]+getG(),AtoO);//[0]+(ylist[i]+getG()[1])*AtoO[1]+(zlist[i]+getG()[2])*AtoO[2];   
        if(da>dda){
            dda=da;
            ai=i;
        }
    }
    ddb=vec_op::dot((*B).getcorner()[0]+(*B).getG(),AtoO);//[0]+((*B).getylist()[0]+getG()[1])*AtoO[1]+((*B).getzlist()[0]+getG()[2])*AtoO[2];
    bi=0;
    for(int i=0;i<tb;i++){
        db=vec_op::dot((*B).getcorner()[i]+(*B).getG(),AtoO);//[0]+((*B).getylist()[i]+getG()[1])*AtoO[1]+((*B).getzlist()[i]+getG()[2])*AtoO[2];   
        if(db<ddb){
            ddb=db;
            bi=i;
        }
    }

    xnew=getcorner()[ai]+getG()-((*B).getcorner()[bi]+(*B).getG());
    //xnew={
    //    -((xlist[ai]+getG()[0])-((*B).getxlist()[bi]+(*B).getG()[0])),
    //    -((ylist[ai]+getG()[1])-((*B).getylist()[bi]+(*B).getG()[1])),
    //    -((zlist[ai]+getG()[2])-((*B).getzlist()[bi]+(*B).getG()[2]))
    //};

    tetrahedron.push_back(xnew);

    std::cout<<"this is triangle."<<std::endl;
    for(int i=0;i<3;i++){
        std::cout<<"x,y,z : "<<tetrahedron[i][0]<<" "<<tetrahedron[i][1]<<" "<<tetrahedron[i][2]<<std::endl;
    }

    std::vector<double>third=getNVecPlane(tetrahedron[0],tetrahedron[1],tetrahedron[2],{0,0,0});

    AtoO=(-1.0)*third;
    std::cout<<"AtoO std::vector : "<<std::endl;
    for(int i=0;i<3;i++){
        std::cout<<i<<" th element : "<<AtoO[i]<<std::endl;
    }
    //AtoO[0]=-third[0];
    //AtoO[1]=-third[1];
    //AtoO[2]=-third[2];
    
    dda=vec_op::dot(getcorner()[0]+getG(),AtoO);//[0]+(ylist[0]+getG()[1])*AtoO[1]+(zlist[0]+getG()[2])*AtoO[2];
    ai=0;
    for(int i=0;i<ta;i++){
        da=vec_op::dot(getcorner()[i]+getG(),AtoO);//[0]+(ylist[i]+getG()[1])*AtoO[1]+(zlist[i]+getG()[2])*AtoO[2];   
        if(da>dda){
            dda=da;
            ai=i;
        }
    }
    ddb=vec_op::dot((*B).getcorner()[0]+(*B).getG(),AtoO);//[0]+((*B).getylist()[0]+getG()[1])*AtoO[1]+((*B).getzlist()[0]+getG()[2])*AtoO[2];
    bi=0;
    for(int i=0;i<tb;i++){
        db=vec_op::dot((*B).getcorner()[i]+(*B).getG(),AtoO);//[0]+((*B).getylist()[i]+getG()[1])*AtoO[1]+((*B).getzlist()[i]+getG()[2])*AtoO[2];   
        if(db<ddb){
            ddb=db;
            bi=i;
        }
    }

    xnew=getcorner()[ai]+getG()-((*B).getcorner()[bi]+(*B).getG());
    //xnew={
    //    -((xlist[ai]+getG()[0])-((*B).getxlist()[bi]+(*B).getG()[0])),
    //    -((ylist[ai]+getG()[1])-((*B).getylist()[bi]+(*B).getG()[1])),
    //    -((zlist[ai]+getG()[2])-((*B).getzlist()[bi]+(*B).getG()[2]))
    //};

    tetrahedron.push_back(xnew);

    std::cout<<"this is tetrahedron."<<std::endl;
    for(int i=0;i<4;i++){
        std::cout<<"x,y,z : "<<tetrahedron[i][0]<<" "<<tetrahedron[i][1]<<" "<<tetrahedron[i][2]<<std::endl;
    }

    bool flag=true;
    short sgn=-1;

    int ccc=10;
    while(ccc>0){
        std::cout<<"hoge"<<std::endl;
        ccc--;
        flag=true;
        for(int i=0;i<4;i++){
            flag=flag&&is_inSameSide(tetrahedron[(0+i)%4],tetrahedron[(1+i)%4],tetrahedron[(2+i)%4],tetrahedron[(3+i)%4],{0,0,0});
            std::cout<<"flag : "<<flag<<std::endl;
        }

        if(flag){
            std::cout<<"in tetrahedron"<<std::endl;
            goEPAFlag=true;
            break;
        }else{
            std::vector<std::vector<double>>vlist;
            for(int i=0;i<4;i++){
                vlist.push_back(getNVecPlane(tetrahedron[(0+i)%4],tetrahedron[(1+i)%4],tetrahedron[(2+i)%4],{0,0,0}));
            }
            double minlen=vec_op::norm(vlist[0]);
            double vlistindex=0;
            std::cout<<"nearest coordinate : "<<std::endl;
            std::cout<<"0 th node : "<<vlist[0][0]<<" "<<vlist[0][1]<<" "<<vlist[0][2]<<std::endl;
            
            for(int i=1;i<vlist.size();i++){
                std::cout<<i<<" th node : "<<vlist[i][0]<<" "<<vlist[i][1]<<" "<<vlist[i][2]<<std::endl;
                if(minlen>vec_op::norm(vlist[i])){
                    vlistindex=i;
                }
            }

            AtoO=(-1.0)*vlist[vlistindex];

            std::cout<<"AtoO std::vector : "<<std::endl;
            for(int i=0;i<3;i++){
                std::cout<<AtoO[i]<<std::endl;
            }
            //AtoO[0]=-vlist[vlistindex][0];
            //AtoO[1]=-vlist[vlistindex][1];
            //AtoO[2]=-vlist[vlistindex][2];
        
            dda=vec_op::dot(getcorner()[0]+getG(),AtoO);//[0]+(ylist[0]+getG()[1])*AtoO[1]+(zlist[0]+getG()[2])*AtoO[2];
            ai=0;
            for(int i=0;i<ta;i++){
                da=vec_op::dot(getcorner()[i]+getG(),AtoO);//[0]+(ylist[i]+getG()[1])*AtoO[1]+(zlist[i]+getG()[2])*AtoO[2];   
                if(da>dda){
                    dda=da;
                    ai=i;
                }
            }
            ddb=vec_op::dot((*B).getcorner()[0]+(*B).getG(),AtoO);//[0]+((*B).getylist()[0]+getG()[1])*AtoO[1]+((*B).getzlist()[0]+getG()[2])*AtoO[2];
            bi=0;
            for(int i=0;i<tb;i++){
                db=vec_op::dot((*B).getcorner()[i]+(*B).getG(),AtoO);//[0]+((*B).getylist()[i]+getG()[1])*AtoO[1]+((*B).getzlist()[i]+getG()[2])*AtoO[2];   
                if(db<ddb){
                    ddb=db;
                    bi=i;
                }
            }

            xnew=getcorner()[ai]+getG()-((*B).getcorner()[bi]+(*B).getG());
            xnew=(-1.0)*xnew;
            tetrahedron.push_back(xnew);

            std::vector<double> tetrahedronIP;
            for(int j=0;j<tetrahedron.size();j++){
                tetrahedronIP.push_back(vec_op::dot(tetrahedron[j],AtoO));
            }
            
            int index=0;
            double maxip=tetrahedronIP[0];
            for(int j=0;j<tetrahedronIP.size();j++){
                if(maxip<tetrahedronIP[j]){
                    maxip=tetrahedronIP[j];
                    index=j;
                }
            }

            tetrahedron.erase(tetrahedron.begin()+index);

            if(
                abs(vec_op::dot(
                    tetrahedron[3]-tetrahedron[0],
                    vec_op::cross(
                        tetrahedron[1]-tetrahedron[0],
                        tetrahedron[2]-tetrahedron[0]
                    )
                ))<3
            ){
                std::cout<<"convergent"<<std::endl;
                goEPAFlag=false;
                break;
            }

            std::cout<<"tetrahedron : "<<std::endl;
            for(int i=0;i<4;i++){
                std::cout<<i<<" th x,y,z : "<<tetrahedron[i][0]<<" "<<tetrahedron[i][1]<<" "<<tetrahedron[i][2]<<std::endl;
            }


        }
    }

std::vector<double>force={0.0,0.0,0.0},escape={0.0,0.0,0.0};
std::cout<<"EPA start."<<std::endl;

bool goVWUpdateFlag=false;

std::vector<std::vector<int>> triangleList={{0,1,2},{0,1,3},{0,2,3},{1,2,3}};
if(goEPAFlag){
    std::cout<<"in EPA loop"<<std::endl;
    for(int i=0;i<4;i++){
        std::cout<<tetrahedron[i][0]<<" "<<tetrahedron[i][1]<<" "<<tetrahedron[i][2]<<std::endl;
    }
    while(1){
        std::vector<double>distPlaneList;
        std::vector<std::vector<double>> nearestCoordinateList;

        int t=tetrahedron.size();

        for(int i=0;i<triangleList.size();i++){
            //std::cout<<i<<" th triangleList : "<<endl;
            //for(int j=0;j<3;j++){
            //    std::cout<<triangleList[i][j]<<endl;
            //}
            distPlaneList.push_back(
                distPlane(
                    tetrahedron[triangleList[i][0]],
                    tetrahedron[triangleList[i][1]],
                    tetrahedron[triangleList[i][2]],
                    {0,0,0}
                ));
            nearestCoordinateList.push_back(
                getNVecPlane(
                    tetrahedron[triangleList[i][0]],
                    tetrahedron[triangleList[i][1]],
                    tetrahedron[triangleList[i][2]],
                    {0,0,0}
                ));
        }

        std::cout<<"distPlaneList : "<<std::endl;
        for(int i=0;i<t;i++){
            std::cout<<i<<" th : "<<distPlaneList[i]<<std::endl;
        }
        std::cout<<"nearestCoordinateList : "<<std::endl;
        for(int i=0;i<t;i++){
            std::cout<<i<<" th : "<<nearestCoordinateList[i][0]<<" "<<nearestCoordinateList[i][1]<<" "<<nearestCoordinateList[i][2]<<std::endl;
        }

        double mindist=distPlaneList[0];
        int index=0;
        for(int i=0;i<distPlaneList.size();i++){
            if(mindist>distPlaneList[i]){
                mindist=distPlaneList[i];
                index=i;
            }
        }

        AtoO=nearestCoordinateList[index];
        std::cout<<"AtoO : "<<std::endl;
        for(int i=0;i<3;i++){
            std::cout<<i<<" th element : "<<AtoO[i]<<std::endl;
        }
        
        //AtoO[0]=nearestCoordinateList[index][0];
        //AtoO[1]=nearestCoordinateList[index][1];
        //AtoO[2]=nearestCoordinateList[index][2];

        //if q is on line or plane. IP is 0.
        //if(vec_op::norm(AtoO)==0){
        if(0){
            std::cout<<"AtoO is 0"<<std::endl;
            for(int i=0;i<t;i++){
                if(
                    vec_op::dot(tetrahedron[(0+i)%t],vec_op::cross(tetrahedron[(1+i)%t],tetrahedron[(2+i)%t]))==0
                ){//3 nodes
                    std::cout<<"qis on line or plane. new AtoO is generated."<<std::endl;
                    force={0.0,0.0,0.0};
                    std::vector<double>hoge=getNVecPlane(
                        tetrahedron[(0+i)%t],
                        tetrahedron[(1+i)%t],
                        tetrahedron[(2+i)%t],
                        tetrahedron[(3+i)%t]);
                    AtoO=tetrahedron[(3+i)%t]-hoge;
                    //AtoO[0]=tetrahedron[(3+i)%t][0]-hoge[0];
                    //AtoO[1]=tetrahedron[(3+i)%t][1]-hoge[1]; 
                    //AtoO[2]=tetrahedron[(3+i)%t][2]-hoge[2];
                    break;
                }
            }
        }
        
        dda=vec_op::dot(getcorner()[0]+getG(),AtoO);//[0]+(ylist[0]+getG()[1])*AtoO[1]+(zlist[0]+getG()[2])*AtoO[2];
        ai=0;
        for(int i=0;i<ta;i++){
            da=vec_op::dot(getcorner()[i]+getG(),AtoO);//[0]+(ylist[i]+getG()[1])*AtoO[1]+(zlist[i]+getG()[2])*AtoO[2];   
            if(da>dda){
                dda=da;
                ai=i;
            }
        }
        ddb=vec_op::dot((*B).getcorner()[0]+(*B).getG(),AtoO);//[0]+((*B).getylist()[0]+getG()[1])*AtoO[1]+((*B).getzlist()[0]+getG()[2])*AtoO[2];
        bi=0;
        for(int i=0;i<tb;i++){
            db=vec_op::dot((*B).getcorner()[i]+(*B).getG(),AtoO);//[0]+((*B).getylist()[i]+getG()[1])*AtoO[1]+((*B).getzlist()[i]+getG()[2])*AtoO[2];   
            if(db<ddb){
                ddb=db;
                bi=i;
            }
        }

        xnew=getcorner()[ai]+getG()-((*B).getcorner()[bi]+(*B).getG());

        short insertindex=0;

        bool samenodeFlag=false;
        for(int i=0;i<t;i++){
            if(tetrahedron[i%t]==xnew){
                insertindex=(i+1)%t;
                force=(-1.0)*AtoO;
                std::cout<<"same node is chosen."<<std::endl;
                goVWUpdateFlag=true;
                samenodeFlag=true;
                break;
            }
        }
        if(samenodeFlag){
            break;
        }

        int indexi=0,indexj=0;
        for(int i=0;i<t;i++){//index triangleList
            for(int j=0;j<t;j++){//nodes that is not on the trianle
                bool flag=true;
                for(int k=0;k<t;k++){
                    if(triangleList[i][k]==j){
                        flag=false;
                        break;
                    }
                }
                if(flag&&!is_inSameSide(tetrahedron[triangleList[i][0]],tetrahedron[triangleList[i][1]],tetrahedron[triangleList[i][2]],xnew,{0,0,0})){
                    //insertindex=(i+1)%t;
                    indexi=i;
                    indexj=j;
                    force=(-1.0)*AtoO;
                    std::cout<<"new triangle is generated."<<std::endl;
                    break;    
                }
            }
        }

        std::cout<<"test1 indexi : "<<indexi<<std::endl;
        for(int i=0;i<3;i++){
            triangleList.push_back({triangleList[indexi][i%3],triangleList[indexi][(i+1)%3],t});
        }
        std::cout<<"test2"<<std::endl;
        triangleList.erase(triangleList.begin()+indexi);
        std::cout<<"triangleList : "<<std::endl;
        for(int i=0;i<triangleList.size();i++){
            std::cout<<triangleList[i][0]<<" "<<triangleList[i][1]<<" "<<triangleList[i][2]<<std::endl;
        }
        //tetrahedron.insert(tetrahedron.begin()+insertindex,xnew);
        tetrahedron.push_back(xnew);

        t=tetrahedron.size();

        std::cout<<"this is tetrahedron : "<<std::endl;
        for(int i=0;i<t;i++){
            std::cout<<tetrahedron[i][0]<<" "<<tetrahedron[i][1]<<" "<<tetrahedron[i][2]<<std::endl;
        }

        if(t>10||distDots(escape,xnew)<1){
            force=(-1.0)*AtoO;
            std::cout<<t<<" convergent"<<std::endl;
            goVWUpdateFlag=true;
            break;
        }
        escape=xnew;
    }
}
std::cout<<"GJK EPA force is over. force:"<<force[0]<<" "<<force[1]<<" "<<force[2]<<std::endl;
return force;

}

void Tetrahedron::Collision(Tetrahedron *B){
    std::vector<double>collision,nearestDot,force={0.0,0.0,0.0};
    
    for(int bbb=0;bbb<1;bbb++){
        force=GJK_EPA_force(B);
        std::cout<<"Collision is detected. force : "<<force[0]<<" "<<force[1]<<" "<<force[2]<<std::endl;
        if(vec_op::norm(force)==0.0){
            std::cout<<"force is 0. A pos is "<<getG()[0]<<" "<<getG()[1]<<" "<<getG()[2]<<std::endl;
            return;
        }
        
        if(!getstop()){
            shiftColPos(1.1*force);
        }

        if(!(*B).getstop()){
            (*B).shiftColPos(-1.1*force);
        }

        short ta=xlist.size();
        short tb=(*B).getxlist().size();

        double len,minlen=distPlane(
            getcorner()[0]+getG(),
            getcorner()[1]+getG(),
            getcorner()[2]+getG(),
            (*B).getcorner()[0]+(*B).getG()
        );
        short ia=0,ib=0;
        char x='a';

        for(int i=0;i<ta;i++){
            for(int j=0;j<tb;j++){
                len=distPlane(
                    getcorner()[(0+i)%ta]+getG(),
                    getcorner()[(1+i)%ta]+getG(),
                    getcorner()[(2+i)%ta]+getG(),
                    (*B).getcorner()[j]+(*B).getG()
                );
                if(len<minlen){
                    minlen=len;
                    ia=i;
                    ib=j;
                    x='a';
                }
            }
        }
    
        for(int i=0;i<tb;i++){
            for(int j=0;j<ta;j++){
                len=distPlane(
                    (*B).getcorner()[(0+i)%tb]+(*B).getG(),
                    (*B).getcorner()[(1+i)%tb]+(*B).getG(),
                    (*B).getcorner()[(2+i)%tb]+(*B).getG(),
                    getcorner()[j]+getG()
                );
                if(len<minlen){
                    minlen=len;
                    ia=i;
                    ib=j;
                    x='b';
                }
            }
        }

        if(x=='a'){
            collision=getNVecPlane(
                getcorner()[(0+ia)%ta]+getG(),
                getcorner()[(1+ia)%ta]+getG(),
                getcorner()[(2+ia)%ta]+getG(),
                (*B).getcorner()[ib]+(*B).getG()
            );
            nearestDot=(*B).getcorner()[ib]+(*B).getG();
        }else if(x=='b'){
            collision=getNVecPlane(
                (*B).getcorner()[(0+ib)%tb]+(*B).getG(),
                (*B).getcorner()[(1+ib)%tb]+(*B).getG(),
                (*B).getcorner()[(2+ib)%tb]+(*B).getG(),
                getcorner()[ia]+getG()
            );
            nearestDot=getcorner()[ia]+getG();
        }
    
        if(!getstop()){
            shiftColPos(collision);
        }
        if(!(*B).getstop()){
            (*B).shiftColPos((-1.0)*collision);
        }

        double Q,a,b;
        std::vector<double>
        ra=nearestDot+collision-getG(),
        rb=nearestDot+collision-(*B).getG(),
        nn=(1/vec_op::norm(force))*force,
        //hh 
        v_ab=((getV()-vec_op::cross(ra,getW()))-((*B).getV()-vec_op::cross(rb,(*B).getW())));
    
        if(
            !getstop()
            &&vec_op::norm(getV())<3.0
            &&vec_op::norm(getW())<3.0
        ){
            std::cout<<"A anteika ----------------------"<<std::endl;
            setstop(1);
        }
        if(
            !(*B).getstop()
            &&vec_op::norm((*B).getV())<3.0
            &&vec_op::norm((*B).getW())<3.0
        ){
            std::cout<<"B anteika -----------------------"<<std::endl;
            (*B).setstop(1);
        }

        //a=vec_op::dot(ra,ra)-pow(vec_op::dot(nn,ra),2);
        //b=vec_op::dot(rb,rb)-pow(vec_op::dot(nn,rb),2);
        a=vec_op::dot(vec_op::cross(vec_op::mat(getaIg(),vec_op::cross(ra,nn)),ra),nn);
        b=vec_op::dot(vec_op::cross(vec_op::mat((*B).getaIg(),vec_op::cross(rb,nn)),rb),nn);

        Q=-(1.0+getE()*(*B).getE())*vec_op::dot(v_ab,nn)/(1.0/getM()+1.0/(*B).getM()+a+b);
    
        std::cout<<std::endl<<"before : "<<std::endl;
        std::cout<<"A V / B V : "<<std::endl;
        for(int i=0;i<3;i++){
            std::cout<<getV()[i]<<" "<<(*B).getV()[i]<<std::endl;
        }
        std::cout<<"A W / B W : "<<std::endl;
        for(int i=0;i<3;i++){
            std::cout<<getW()[i]<<" "<<(*B).getW()[i]<<std::endl;
        }
        std::cout<<"Q : "<<Q<<std::endl;
        std::cout<<"n std::vector : "<<std::endl;
        for(int i=0;i<3;i++){
            std::cout<<nn[i]<<std::endl;
        }

        if(!getstop()){//remember ... I tensor
            setV(getV()+(Q/getM())*nn);
            setW(getW()+(Q)*(vec_op::mat(getaIg(),vec_op::cross(ra,nn))));
        }
    
        if(!(*B).getstop()){//remember ... I tensor
            (*B).setV((*B).getV()+(-Q/(*B).getM())*nn);
            (*B).setW((*B).getW()+(-Q)*(vec_op::mat((*B).getaIg(),vec_op::cross(rb,nn))));
        }

        std::cout<<std::endl<<"after : "<<std::endl;
        std::cout<<"A Pos / B pos : "<<std::endl;
        for(int i=0;i<3;i++){
            std::cout<<getG()[i]<<" "<<(*B).getG()[i]<<std::endl;
        }
        std::cout<<"A V / B V : "<<std::endl;
        for(int i=0;i<3;i++){
            std::cout<<getV()[i]<<" "<<(*B).getV()[i]<<std::endl;
        }
        std::cout<<"A W / B W : "<<std::endl;
        for(int i=0;i<3;i++){
            std::cout<<getW()[i]<<" "<<(*B).getW()[i]<<std::endl;
        }


    }
    std::cout<<"GJK is over."<<std::endl;
}

void Tetrahedron::addJoint(std::vector<double> ff){
    joint=1;
    fix.push_back(ff);
    If=I;
}

void Tetrahedron::removeJoint(int i){
    joint=0;
    fix.removeAt(i);
}

void timeCounter();
void frameManager();
void modeChange(int);
void gameInitialize();
void gameDisplay();

void timeCounter(){

return;
}

void frameManager(){
while(true){
//this_thread::sleep_for(chrono::milliseconds(GV.getspf()));
Sleep(GV.getspf());
GV.setframeFlag(true);
//std::cout<<"set flag true..."<<endl;
}
return;
}

void modeChange(int m){
switch(m){
    case 0://easy
        break;
    case 1://normal
        break;
    case 2://hard
        break;
}

gameInitialize();
return;
}
void gameInitialize(){



    gameDisplay();
    return;
}

void keyEvent(){
const unsigned short MSB=0x8000;
while(1){
        if(GetAsyncKeyState(VK_UP)&MSB){
            std::cout<<"key up"<<std::endl;
            camera.setx(camera.getx()+camera.getspeed()*camera.getsx());
            camera.sety(camera.gety()+camera.getspeed()*camera.getsy());
            camera.setz(camera.getz()+camera.getspeed()*camera.getsz());
        }
        if(GetAsyncKeyState(VK_DOWN)&MSB){
            std::cout<<"key down"<<std::endl;
            camera.setx(camera.getx()-camera.getspeed()*camera.getsx());
            camera.sety(camera.gety()-camera.getspeed()*camera.getsy());
            camera.setz(camera.getz()-camera.getspeed()*camera.getsz());

        }
        if(GetAsyncKeyState(VK_LEFT)&MSB){
            std::cout<<"key left"<<std::endl;
            double gx,gy,gz,cx,cy,cz;
            gx=camera.getgravity(0);
            gy=camera.getgravity(1);
            gz=camera.getgravity(2);
            cx=camera.getsx();
            cy=camera.getsy();
            cz=camera.getsz();

            camera.setx(camera.getx()+camera.getspeed()*(gz*cy-gy*cz));
            camera.sety(camera.gety()+camera.getspeed()*(gx*cz-gz*cx));
            camera.setz(camera.getz()+camera.getspeed()*(gy*cx-gx*cy));
        }
        if(GetAsyncKeyState(VK_RIGHT)&MSB){
            std::cout<<"key right"<<std::endl;
            GV.setcursorX(GV.getcursorX()+10);
            while(GV.getcursorX()>GV.getCW())GV.setcursorX(GV.getcursorX()-GV.getCW());
            double gx,gy,gz,cx,cy,cz;
            gx=camera.getgravity(0);
            gy=camera.getgravity(1);
            gz=camera.getgravity(2);
            cx=camera.getsx();
            cy=camera.getsy();
            cz=camera.getsz();

            camera.setx(camera.getx()+camera.getspeed()*(gy*cz-gz*cy));
            camera.sety(camera.gety()+camera.getspeed()*(gz*cx-gx*cz));
            camera.setz(camera.getz()+camera.getspeed()*(gx*cy-gy*cx));

        }
        if(GetAsyncKeyState(VK_SHIFT)&MSB){
            camera.setspeed(10.0);
        }else{
            camera.setspeed(2.0);
        }
        if(GetAsyncKeyState(VK_HOME)&MSB){
            camera.setsx(1);
            camera.setsy(0);
            camera.setsz(0);
        }
        if(GetAsyncKeyState(VK_TAB)&MSB){
            GV.settest(1);
        }
        Sleep(GV.getspf());
 
}
}

void mouseCallback(int event,int x,int y,int flag,void *userdata){
    if(event==cv::EVENT_MOUSEMOVE){
        int preX,preY;
        preX=GV.getmouseUpdateX();
        preY=GV.getmouseUpdateY();
        double x2,y2,z2,x3,y3,z3,rot1,rot2;
        x2=camera.getsx();
        y2=camera.getsy();
        z2=camera.getsz();
        
        rot1=(double)((double)x-(double)preX)*(double)Math::PI/(double)180.0;
        rot2=(double)((double)y-(double)preY)*(double)Math::PI/(double)360.0;

        x3=x2*cos(rot1)-y2*sin(rot1);
        y3=x2*sin(rot1)+y2*cos(rot1);

        x2=x3;
        y2=y3;

        double root=sqrt(camera.getsx()*camera.getsx()+camera.getsy()*camera.getsy());
        std::vector<double>nn={camera.getsy()/root,-camera.getsx()/root,0};

        x3=
             (cos(rot2)+nn[0]*nn[0]*(1-cos(rot2)))*x2
            +(nn[0]*nn[1]*(1-cos(rot2))-nn[2]*sin(rot2))*y2
            +(nn[2]*nn[0]*(1-cos(rot2))+nn[1]*sin(rot2))*z2;
        y3=
             (nn[0]*nn[1]*(1-cos(rot2))+nn[2]*sin(rot2))*x2
            +(cos(rot2)+nn[1]*nn[1]*(1-cos(rot2)))*y2
            +(nn[1]*nn[2]*(1-cos(rot2))-nn[0]*sin(rot2))*z2;
        z3=
             (nn[2]*nn[0]*(1-cos(rot2))-nn[1]*sin(rot2))*x2
            +(nn[1]*nn[2]*(1-cos(rot2))+nn[0]*sin(rot2))*y2
            +(cos(rot2)+nn[2]*nn[2]*(1-cos(rot2)))*z2;

        std::cout<<"sx,sy,sz : "<<x3<<" "<<y3<<" "<<z3<<std::endl;
        camera.setsx(x3);
        camera.setsy(y3);
        camera.setsz(z3);

        GV.setmouseUpdateX(x);
        GV.setmouseUpdateY(y);
    }
}

std::vector<Triangle> trianglelist;

void gameDisplay(){
    GV.setframeFlag(false);

    Tetrahedron testA=Tetrahedron({{100,100,100},{100,200,100},{100,100,200},{200,100,100}},{0.0,0.0,0.0},{0.0,0.0,0.0},1.0,1.0,1.0,1.0);
    Tetrahedron floor=Tetrahedron({{0,0,-200},{0,300,-200},{300,0,-200},{0,0,-300}},{0,0,0},{0,0,0},1.0,1.0,1.0,1.0);
    floor.setstop(1);

    std::vector<Tetrahedron> testlist={testA,floor};
while(1){
    cv::Mat img(cv::Size(GV.getCW(),GV.getCH()),CV_8UC3,cv::Scalar(0,0,0));

    std::vector<double> a={0.0,0.0,0.0};
    std::vector<double> b={0.0,0.0,0.0};

    a[0]=camera.getsx();
    a[1]=camera.getsy();
    a[2]=camera.getsz();
    b[0]=camera.getx();
    b[1]=camera.gety();
    b[2]=camera.getz();
    
    std::vector<Triangle> list;
    //list.clear();
    double scale=300.0;
    int rr=0;
    for(int i=-rr;i<=rr;i++){
        for(int j=-rr;j<=rr;j++){
            for(int k=-rr;k<=rr;k++){
                for(int l=0;l<trianglelist.size();l++){
                    double x1,y1,z1,x2,y2,z2,x3,y3,z3;
                    x1=trianglelist[l].getpoints()[0][0]+i*scale;
                    y1=trianglelist[l].getpoints()[0][1]+j*scale;
                    z1=trianglelist[l].getpoints()[0][2]+k*scale;
                    x2=trianglelist[l].getpoints()[1][0]+i*scale;
                    y2=trianglelist[l].getpoints()[1][1]+j*scale;
                    z2=trianglelist[l].getpoints()[1][2]+k*scale;
                    x3=trianglelist[l].getpoints()[2][0]+i*scale;
                    y3=trianglelist[l].getpoints()[2][1]+j*scale;
                    z3=trianglelist[l].getpoints()[2][2]+k*scale;
                       
                    //list.push_back(Triangle({{x1,y1,z1},{x2,y2,z2},{x3,y3,z3}}));
                    list.push_back(Triangle({{0,0,0},{0,10,0},{0,0,10}}));
                }
            }
        }
    }

    //std::cout<<"list size : "<<list.size()<<"get std::vector : "<<list[0].getpoints()[0][0]<<endl;
    
    for(int i=0;i<list.size();i++){
        std::vector<cv::Point> plist;
        bool drawflag=false;
        bool inframe=true;
        int colortank;
        for(int j=0;j<4;j++){
            double x,y,z,t;
            std::vector<double> O_yz={0.0,0.0,0.0};
            x=list[i].getpoints()[j%3][0];
            y=list[i].getpoints()[j%3][1];
            z=list[i].getpoints()[j%3][2];
            t=a[0]*(x-b[0])+a[1]*(y-b[1])+a[2]*(z-b[2]);

            O_yz[0]=a[0]*t+b[0];
            O_yz[1]=a[1]*t+b[1];
            O_yz[2]=a[2]*t+b[2];
            
            double l;
            l=sqrt((x-O_yz[0])*(x-O_yz[0])+(y-O_yz[1])*(y-O_yz[1])+(z-O_yz[2])*(z-O_yz[2]));
            
            if(l!=0){
                double costh,th,x2,y2,z2;
                int gx,gy,gz;
                gx=camera.getgravity(0);
                gy=camera.getgravity(1);
                gz=camera.getgravity(2);
                std::vector<double> ydash={0.0,0.0,0.0};
                ydash[0]=gy*a[2]-gz*a[1];
                ydash[1]=gz*a[0]-gx*a[2];
                ydash[2]=gx*a[1]-gy*a[0];
                double ly;
                ly=sqrt(ydash[0]*ydash[0]+ydash[1]*ydash[1]+ydash[2]*ydash[2]);
                costh=((ydash[0]*(x-O_yz[0])+ydash[1]*(y-O_yz[1])+ydash[2]*(z-O_yz[2]))/(l*ly));

                th=acos(costh);
                x2=t;
                int sgn=1;
                int Ge=gx+gy+gz;
        
                sgn=gx*x+gy*y+gz*z>gx*O_yz[0]+gy*O_yz[1]+gz*O_yz[2]?Ge:-Ge;
		
                y2=l*cos(th);
                z2=l*sgn*sin(th);

                plist.push_back(
                    cv::Point(
                        GV.getCW()/2+y2*(1000/sqrt((t*t+1))),
                        GV.getCH()/2-z2*(1000/sqrt((t*t+1)))
                    ));
                
                if(t>1){
                    drawflag=drawflag||false;
                }else{
                    drawflag=drawflag||true;
                    //inframe=inframe||false;
                }
            }else{
                plist.push_back(
                    cv::Point(
                        GV.getCW()/2,GV.getCH()/2
                    )
                );
            }
            colortank=(int)(t/10);
        }
    
        cv::Scalar test;
        test=cv::Scalar(0xff-colortank%256,0,0xff-colortank%256);
	
        if(!drawflag&&(0<=colortank)&&(colortank<=255)){
            //int lineType=cv::LINE_8;
            //cv::Point rook_points[1][3];
            //rook_points[0][0]=plist[0];
            //rook_points[0][1]=plist[1];
            //rook_points[0][2]=plist[2];
            //const cv::Point* ppt[1]={rook_points[0]};
            //int npt[]={3};
            //fillPoly(img,ppt,npt,1,cv::Scalar(0xff-colortank%256,0xff-colortank%256,0xff-colortank%256,0.5),lineType);
            polylines(img,plist,true,test,1,cv::LINE_AA);
        }
    }

    for(int hoge=0;hoge<testlist.size();hoge++){
    for(int i=0;i<4;i++){
        std::vector<cv::Point>pplist;
        bool drawflag=false;
        bool inframe=true;
        int colortank;
        for(int j=0;j<4;j++){
            double x,y,z,t;
            std::vector<double> O_yz={0.0,0.0,0.0};
            x=testlist[hoge].getcorner()[(i+j%3)%4][0]+testlist[hoge].getG()[0];
            y=testlist[hoge].getcorner()[(i+j%3)%4][1]+testlist[hoge].getG()[1];
            z=testlist[hoge].getcorner()[(i+j%3)%4][2]+testlist[hoge].getG()[2];
            t=a[0]*(x-b[0])+a[1]*(y-b[1])+a[2]*(z-b[2]);

            O_yz[0]=a[0]*t+b[0];
            O_yz[1]=a[1]*t+b[1];
            O_yz[2]=a[2]*t+b[2];
            
            double l;
            l=sqrt((x-O_yz[0])*(x-O_yz[0])+(y-O_yz[1])*(y-O_yz[1])+(z-O_yz[2])*(z-O_yz[2]));
            
            if(l!=0){
                double costh,th,x2,y2,z2;
                int gx,gy,gz;
                gx=camera.getgravity(0);
                gy=camera.getgravity(1);
                gz=camera.getgravity(2);
                std::vector<double> ydash={0.0,0.0,0.0};
                ydash[0]=gy*a[2]-gz*a[1];
                ydash[1]=gz*a[0]-gx*a[2];
                ydash[2]=gx*a[1]-gy*a[0];
                double ly;
                ly=sqrt(ydash[0]*ydash[0]+ydash[1]*ydash[1]+ydash[2]*ydash[2]);
                costh=((ydash[0]*(x-O_yz[0])+ydash[1]*(y-O_yz[1])+ydash[2]*(z-O_yz[2]))/(l*ly));

                th=acos(costh);
                x2=t;
                int sgn=1;
                int Ge=gx+gy+gz;
        
                sgn=gx*x+gy*y+gz*z>gx*O_yz[0]+gy*O_yz[1]+gz*O_yz[2]?Ge:-Ge;
		
                y2=l*cos(th);
                z2=l*sgn*sin(th);

                pplist.push_back(
                    cv::Point(
                        GV.getCW()/2+y2*(1000/sqrt((t*t+1))),
                        GV.getCH()/2-z2*(1000/sqrt((t*t+1)))
                    ));
                
                if(t>1){
                    drawflag=drawflag||false;
                }else{
                    drawflag=drawflag||true;
                }
            }else{
                pplist.push_back(
                    cv::Point(
                        GV.getCW()/2,GV.getCH()/2
                    )
                );
            }
            colortank=(int)(t/10);
        }
    
        if(!drawflag&&(0<=colortank)&&(colortank<=255)){
            polylines(img,pplist,true,cv::Scalar(0xff,0xff,0xff),1,cv::LINE_AA);
        }
    }
    }

    for(int j=0;j<4;j++){
    for(int i=0;i<4;i++){
            double x,y,z,t;
            std::vector<double> 
            O_yz={0.0,0.0,0.0},
            vv=testA.getcorner()[i]+testA.getG()-(floor.getcorner()[i]+floor.getG());
            t=vec_op::dot(a,vv-b);
            //x=(testA.getcorner()[i][0]+testA.getG()[0])-(floor.getcorner()[j][0]+floor.getG()[0]);
            //y=(testA.getcorner()[i][1]+testA.getG()[1])-(floor.getcorner()[j][1]+floor.getG()[1]);
            //z=(testA.getcorner()[i][2]+testA.getG()[2])-(floor.getcorner()[j][2]+floor.getG()[2]);
            //t=a[0]*(x-b[0])+a[1]*(y-b[1])+a[2]*(z-b[2]);

            //O_yz[0]=a[0]*t+b[0];
            //O_yz[1]=a[1]*t+b[1];
            //O_yz[2]=a[2]*t+b[2];
            O_yz=t*a+b;
            double l=vec_op::norm(vv-O_yz);
            //l=sqrt((x-O_yz[0])*(x-O_yz[0])+(y-O_yz[1])*(y-O_yz[1])+(z-O_yz[2])*(z-O_yz[2]));
            
            if(l!=0){
                double costh,th,x2,y2,z2;
                int gx,gy,gz;
                gx=camera.getgravity(0);
                gy=camera.getgravity(1);
                gz=camera.getgravity(2);
                std::vector<double> ydash={0.0,0.0,0.0};
                ydash[0]=gy*a[2]-gz*a[1];
                ydash[1]=gz*a[0]-gx*a[2];
                ydash[2]=gx*a[1]-gy*a[0];
                double ly;
                ly=sqrt(ydash[0]*ydash[0]+ydash[1]*ydash[1]+ydash[2]*ydash[2]);
                costh=((ydash[0]*(x-O_yz[0])+ydash[1]*(y-O_yz[1])+ydash[2]*(z-O_yz[2]))/(l*ly));

                th=acos(costh);
                x2=t;
                int sgn=1;
                int Ge=gx+gy+gz;
        
                sgn=gx*x+gy*y+gz*z>gx*O_yz[0]+gy*O_yz[1]+gz*O_yz[2]?Ge:-Ge;
		
                y2=l*cos(th);
                z2=l*sgn*sin(th);

                //pplist.push_back(
                //    cv::Point(
                //        GV.getCW()/2+y2*(1000/sqrt((t*t+1))),
                //        GV.getCH()/2-z2*(1000/sqrt((t*t+1)))
                //    ));
                
                if(t>1){
                    //drawflag=drawflag||false;
                    cv::circle(img,cv::Point(
                        GV.getCW()/2+y2*(1000/sqrt(t*t+1)),
                        GV.getCH()/2-z2*(1000/sqrt(t*t+1))
                        ),10,cv::Scalar(0xff,0xff,0xff));
                }
            }
    }
    }


    for(int i=0;i<testlist.size();i++){
    
        if(!testlist[i].getstop()){
            testlist[i].update();
        }
        for(int i=0;i<testlist.size();i++){
            for(int j=i+1;j<testlist.size();j++){
                if(testlist[i].AABB(&(testlist[j]))){
                    std::cout<<"---------------------- AABB "<<i<<" vs "<<j<<"---------------------------"<<std::endl;
                    testlist[i].Collision(&(testlist[j]));
                }
            }
        }
    }

    if(GV.gettest()){
        Sleep(1000000);
    }
    cv::imshow("3DPhysics",img);
    img.release();
    
    cv::setMouseCallback("3DPhysics",mouseCallback);
    
    while(!GV.getframeFlag()){
        cv::waitKey(GV.getspf());        
    }

    GV.setframeFlag(false);
}
    return;
}

int main(int argc,char** argv){
    //trianglelist.push_back(Triangle({{0,20,20},{0,0,20},{0,20,0}}));
    //trianglelist.push_back(Triangle({{200,-20,-20},{200,-200,-20},{200,-20,-200}}));
    
    trianglelist.push_back(Triangle({{200,0,0},{200,100,0},{100,100,0}}));
    trianglelist.push_back(Triangle({{200,0,0},{200,100,0},{200,100,100}}));
    trianglelist.push_back(Triangle({{200,100,0},{100,100,0},{200,100,100}}));
    trianglelist.push_back(Triangle({{200,0,0},{200,100,100},{100,100,0}}));

    trianglelist.push_back(Triangle({{100,0,0},{100,100,0},{0,100,0}}));
    trianglelist.push_back(Triangle({{0,0,0},{100,0,0},{0,100,0}}));
    
    trianglelist.push_back(Triangle({{100,100,100},{100,100,0},{0,100,100}}));
    trianglelist.push_back(Triangle({{0,100,0},{100,100,0},{0,100,100}}));


    //trianglelist.push_back(Triangle({{-300,10,20},{-100,100,100},{-200,-200,-100}}));   
    //trianglelist.push_back(Triangle({{-100,20,10},{-100,10,10},{-100,10,10}}));

    std::thread th_main(gameInitialize);
    std::thread th_time(timeCounter);
    std::thread th_frame(frameManager);
    std::thread th_keyEvent(keyEvent);

    th_main.join();
    th_time.join();
    th_frame.join();
    th_keyEvent.join();
    return 0;
}

//#endif