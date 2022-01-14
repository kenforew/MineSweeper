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

#include "vec_op.hpp"
#include "const.hpp"

#include <opencv2/opencv.hpp>

//using namespace std;

class GrobalVariables{
private:
    std::uint16_t CANVAS_WIDTH;
    std::uint16_t CANVAS_HEIGHT;
    
    double GX;
    double GY;
    double VMax;

    bool frameFlag;

    std::uint16_t spf;
public:
    GrobalVariables(){
        CANVAS_WIDTH=800;
        CANVAS_HEIGHT=400;

        GX=0;
        GY=0.5;
        VMax=10;

        spf=100;
    }
    ~GrobalVariables(){

    }
    void setframeFlag(bool ff){
        frameFlag=ff;
    }
    std::uint16_t getCW(){
        return CANVAS_WIDTH;
    }
    std::uint16_t getCH(){
        return CANVAS_HEIGHT;
    }
    double getGX(){
        return GX;
    }
    double getGY(){
        return GY;
    }
    std::vector<double>getG(){
        return {GX,GY};
    }
    double getVMax(){
        return VMax;
    }
    bool getframeFlag(){
        return frameFlag;
    }
    std::uint16_t getspf(){
        return spf;
    }
};

GrobalVariables GV;

class Triangle{
private:
    std::vector<std::vector<double>> corner;//相対座標
    std::vector<double> xlist,ylist,v,g,fix;
    //double x1,y1,x2,y2,x3,y3;//相対座標
    //一般座標 x=x1+xg,y=y1+yg
    double m,e,sfc,dfc,w,I,Ig,If;
    //static friction coefficient // dynamic friction coefficient
    bool stop,joint;
public:
    Triangle(std::vector<std::vector<double>>cc,double mm,double ee,double ssfc,double ddfc,double ww,std::vector<double>vv){
        double xg,yg;
        //x1=cc[0][0];y1=cc[0][1];x2=cc[1][0];y2=cc[1][1];x3=cc[2][0];y3=cc[2][1];
        
        //xg=(x1+x2+x3)/3;yg=(y1+y2+y3)/3;
        //g={xg,yg};
        
        //x1=x1-xg;x2=x2-xg;x3=x3-xg;
        //y1=y1-yg;y2=y2-yg;y3=y3-yg;
        
        //xlist={x1,x2,x3};

        //ylist={y1,y2,y3};

        //corner={
        //    {x1,y1},
        //    {x2,y2},
        //    {x3,y3}
        //};

        //m=mm;e=ee,sfc=ssfc,dfc=ddfc,w=ww;v=vv;
        
        //I=m/18.0*(
        //    (x1*x1)+(y1*y1)+(x2*x2)+(y2*y2)+(x3*x3)+(y3*y3)
        //    -(x1*x2)-(y1*y2)-(x2*x3)-(y2*y3)-(x3*x1)-(y3*y1)
        //);
        
        xg=0;yg=0;
        for(int i=0;i<cc.size();i++){
            xg+=cc[i][0];
            yg+=cc[i][1];
        }
        xg=xg/cc.size();
        yg=yg/cc.size();

        g={xg,yg};
        
        xlist.clear();ylist.clear();corner.clear();
        for(int i=0;i<cc.size();i++){
            xlist.push_back(cc[i][0]-xg);
            ylist.push_back(cc[i][1]-yg);
            corner.push_back(cc[i]-g);
        }

        m=mm;e=ee,sfc=ssfc,dfc=ddfc,w=ww;v=vv;
        
        I=0;
        for(int i=0;i<corner.size();i++){
            I=I+vec_op::dot(corner[i],corner[i])-vec_op::dot(corner[i],corner[(i+1)%corner.size()]);
        }
        I=I*m/18.0;

        stop=false;
        
        joint=false;
    }

    Triangle(std::vector<std::vector<double>>cc,double mm,double ee,double ssfc,double ddfc,double ww,std::vector<double> vv,std::vector<double>jj){
        //double x1,y1,x2,y2,x3,y3,
        double xg,yg;
        //x1=cc[0][0];y1=cc[0][1];x2=cc[1][0];y2=cc[1][1];x3=cc[2][0];y3=cc[2][1];
        
        //xg=(x1+x2+x3)/3;yg=(y1+y2+y3)/3;
        xg=0;yg=0;
        for(int i=0;i<cc.size();i++){
            xg+=cc[i][0];
            yg+=cc[i][1];
        }
        xg=xg/cc.size();
        yg=yg/cc.size();

        g={xg,yg};
        
        //x1=x1-xg;x2=x2-xg;x3=x3-xg;
        //y1=y1-yg;y2=y2-yg;y3=y3-yg;
        
        //xlist={x1,x2,x3};

        //ylist={y1,y2,y3};

        xlist.clear();ylist.clear();corner.clear();
        for(int i=0;i<cc.size();i++){
            xlist.push_back(cc[i][0]-xg);
            ylist.push_back(cc[i][1]-yg);
            corner.push_back(cc[i]-g);
        }
        //corner={
        //    {x1,y1},
        //    {x2,y2},
        //    {x3,y3}
        //}/;

        m=mm;e=ee,sfc=ssfc,dfc=ddfc,w=ww;v=vv;
        
        //I=m/18.0*(
        //    (x1*x1)+(y1*y1)+(x2*x2)+(y2*y2)+(x3*x3)+(y3*y3)
        //    -(x1*x2)-(y1*y2)-(x2*x3)-(y2*y3)-(x3*x1)-(y3*y1)
        //);
        I=0;
        for(int i=0;i<corner.size();i++){
            I=I+vec_op::dot(corner[i],corner[i])-vec_op::dot(corner[i],corner[(i+1)%corner.size()]);
        }
        I=I*m/18.0;

        stop=false;

        joint=true;
        fix=jj;

        If=I;
    }

    ~Triangle(){}
    
    std::vector<double> getxlist(){
        return xlist;
    }
    std::vector<double> getylist(){
        return ylist;
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
    double getW(){
        return w;
    }
    double getM(){
        return m;
    }
    double getE(){
        return e;
    }
    double getI(){
        return I;
    }
    double getsfc(){
        return sfc;
    }
    double getdfc(){
        return dfc;
    }
    bool getstop(){
        return stop;
    }
    bool getjoint(){
        return joint;
    }
    std::vector<double> getfix(){
        return fix;
    }

    void setG(std::vector<double> gg){
        g=gg;
    }
    void setV(std::vector<double> vv){
        v=vv;
    }
    void setW(double ww){
        w=ww;
    }
    void setstop(bool ss){
        stop=ss;
        v=(0.0)*v;
        w=0;
    }
    void setjoint(bool jj){
        joint=jj;
    }
    void setfix(std::vector<double>ff){
        fix=ff;
    }
    
    void VWUpdate(double);
    void PosUpdate(double);
    void RotUpdate(double);

    void update();

    void JointWUpdate(double);
    void JointPosUpdate(double);
    void JointRotUpdate(double);
    
    void Jointupdate();

    void shiftColPos(std::vector<double>);
    bool AABB(Triangle *);
    double distDots(std::vector<double>,std::vector<double>);
    double distLine(std::vector<double>,std::vector<double>,std::vector<double>);    
    std::vector<double> getNVector(std::vector<double>,std::vector<double>,std::vector<double>);
    std::vector<double> GJK_EPA_force(Triangle *);
    void Collision(Triangle *);
    void addJoint(std::vector<double>);
    void removeJoint();

};

void Triangle::VWUpdate(double ratio){
    v=v+ratio*GV.getG();
}
void Triangle::PosUpdate(double ratio){
    g=g+ratio*v;
}

void Triangle::RotUpdate(double ratio){
    double rr=1.0;
    std::vector<double> xy1,xy2,xy3;
    xy1=vec_op::rot_2(ratio*w/rr,corner[0]);
    xy2=vec_op::rot_2(ratio*w/rr,corner[1]);
    xy3=vec_op::rot_2(ratio*w/rr,corner[2]);

    xlist={xy1[0],xy2[0],xy3[0]};
    ylist={xy1[1],xy2[1],xy3[1]};

    corner={
        xy1,xy2,xy3
    };
}

void Triangle::update(){
    VWUpdate(1.0);
    PosUpdate(1.0);
    RotUpdate(1.0);
}

void Triangle::JointWUpdate(double ratio){
    w=w+vec_op::cross_2(g-fix,GV.getG())*m/If;
}

void Triangle::JointPosUpdate(double ratio){
    //g=g+(ratio*w)*(g-fix);
}

void Triangle::JointRotUpdate(double ratio){
    double rr=1.0;
    std::vector<double> xy1,xy2,xy3;
    xy1=vec_op::rot_2(ratio*w/rr,(corner[0]+g-fix));
    xy2=vec_op::rot_2(ratio*w/rr,(corner[1]+g-fix));
    xy3=vec_op::rot_2(ratio*w/rr,(corner[2]+g-fix));


    //std::cout<<"xy1 : "<<xy1[0]<<" "<<xy1[1]<<std::endl;
    //std::cout<<"xy2 : "<<xy2[0]<<" "<<xy2[1]<<std::endl;
    //std::cout<<"xy3 : "<<xy3[0]<<" "<<xy3[1]<<std::endl;
    //std::cout<<"fix : "<<fix[0]<<" "<<fix[1]<<std::endl;

    g=((1.0/3.0)*(xy1+xy2+xy3))+fix;
    //std::cout<<"g : "<<g[0]<<" "<<g[1]<<std::endl;

    corner={
        xy1+fix-g,xy2+fix-g,xy3+fix-g
    };

    //for(int i=0;i<corner.size();i++){
    //    std::cout<<i<<" th corner : "<<corner[i][0]<<" "<<corner[i][1]<<std::endl;
    //}

    xlist={corner[0][0],corner[1][0],corner[2][0]};
    ylist={corner[0][1],corner[1][1],corner[2][1]};
}

void Triangle::Jointupdate(){
    JointWUpdate(1.0);
    JointPosUpdate(1.0);
    JointRotUpdate(1.0);
}


void Triangle::shiftColPos(std::vector<double> shift){
    g=g+shift;
}

bool Triangle::AABB(Triangle *B){
    double minAX,minAY,maxAX,maxAY;
    minAX=*min_element(xlist.begin(),xlist.end())+getG()[0];
    maxAX=*max_element(xlist.begin(),xlist.end())+getG()[0];
    minAY=*min_element(ylist.begin(),ylist.end())+getG()[1];
    maxAY=*max_element(ylist.begin(),ylist.end())+getG()[1];

    double minBX,minBY,maxBX,maxBY;
    minBX=*min_element((*B).xlist.begin(),(*B).xlist.end())+(*B).getG()[0];
    maxBX=*max_element((*B).xlist.begin(),(*B).xlist.end())+(*B).getG()[0];
    minBY=*min_element((*B).ylist.begin(),(*B).ylist.end())+(*B).getG()[1];
    maxBY=*max_element((*B).ylist.begin(),(*B).ylist.end())+(*B).getG()[1];

    bool flag=true;
    if(maxBX<minAX||maxAX<minBX||
    maxBY<minAY||maxAY<minBY){
        flag=false;
    }

    return flag;
}
double Triangle::distDots(std::vector<double> p1,std::vector<double> p2){
    return vec_op::norm(p2-p1);
}
double Triangle::distLine(std::vector<double> p1,std::vector<double> p2,std::vector<double> q){
    double a;
    std::vector<double> dv=p2-p1;
    a=vec_op::dot(dv,dv);

    if(a==0){
        return vec_op::norm(p1-q);
    }
    double b,t;
    b=vec_op::dot(dv,p1)-vec_op::dot(dv,q);
    t=-b/a;
    if(t<0.0)t=0.0;
    if(t>1.0)t=1.0;
    std::vector<double> ans=p1+t*dv;

    return vec_op::norm(ans-q);
}
std::vector<double> Triangle::getNVector(std::vector<double> p1,std::vector<double> p2,std::vector<double> q){
    double a;
    std::vector<double> dv=p2-p1;
    a=vec_op::dot(dv,dv);

    if(a==0){
        return p1-q;
    }
    double b,t;
    b=vec_op::dot(dv,p1)-vec_op::dot(dv,q);
    t=-b/a;
    if(t<0.0)t=0.0;
    if(t>1.0)t=1.0;
    std::vector<double> ans=p1+t*dv;
    return ans-q;
}

std::vector<double> Triangle::GJK_EPA_force(Triangle *B){
    std::cout<<"GJK EPA force is called."<<std::endl;
    bool goEPAFlag=false;

    std::vector<double> AtoO=(-1.0)*g,xnew={0.0,0.0};//01
    double da,db,dda,ddb,r=100;
    int ai,bi,ta,tb;
    std::vector<std::vector<double>> triangle;
    ta=getxlist().size();
    tb=(*B).getxlist().size();

    //EPA properties 
    //vector<double>force={0.0,0.0};
    //vector<double>escape={0.0,0.0};
    //cout<<"EPA start. init Triangle : "<<triangle[0][0]<<" "<<triangle[0][1]<<" "<<triangle[1][0]<<" "<<triangle[1][1]<<" "<<triangle[2][0]<<" "<<triangle[2][1]<<endl;
    //bool goVWUpdateFlag=false;

    //V,W Update properties
    //vector<double> collision;
    //vector<double> nearestDot;
            

    for(int hoge=0;hoge<2;hoge++){
        dda=vec_op::dot(getcorner()[0]+getG(),AtoO);
        ai=0;
        for(int i=0;i<ta;i++){
            //da=(xlist[i]+getG()[0])*AtoO[0]+(ylist[i]+getG()[1])*AtoO[1];
            da=vec_op::dot(getcorner()[i]+getG(),AtoO);
            //std::cout<<i<<" th da : "<<da<<std::endl;
            if(da>dda){
                dda=da;
                ai=i;
            }
        }
        ddb=vec_op::dot((*B).getcorner()[0]+(*B).getG(),AtoO);
        bi=0;
        for(int i=0;i<tb;i++){
            //db=((*B).getxlist()[i]+(*B).getG()[0])*AtoO[0]+((*B).getylist()[i]+(*B).getG()[1])*AtoO[1];
            db=vec_op::dot((*B).getcorner()[i]+(*B).getG(),AtoO);
            //std::cout<<i<<" th db : "<<db<<std::endl;
            if(db<ddb){
                ddb=db;
                bi=i;
            }
        }

        //minimize
        xnew=(-1.0)*(getcorner()[ai]+getG())+((*B).getcorner()[bi]+(*B).getG());
        triangle.push_back(xnew);
        AtoO=xnew;
    }

    //ここまでで最初の二点を vector<double> triangle に格納

    
    std::cout<<"line segment two nodes : ("<<triangle[0][0]<<" "<<triangle[0][1]<<") ("<<triangle[1][0]<<" "<<triangle[1][1]<<")"<<std::endl;

    //triangle : vector from (x,y) to (0,0) coordinate:(x,y)<=>vector:(-x,-y)
    std::vector<double> second=getNVector((-1.0)*triangle[0],(-1.0)*triangle[1],{0.0,0.0});//04
    //原点から線分へ垂線を下ろす　交点の座標取得
    //AtoO[0]=-second[0];
    //AtoO[1]=-second[1];//05
    AtoO=(-1.0)*second;
    std::cout<<"n vector from IS to O : "<<AtoO[0]<<" "<<AtoO[1]<<std::endl;
    //交点から原点に向かうベクトルを作成
    //
    //dda=(xlist[0]+getG()[0])*AtoO[0]+(ylist[0]+getG()[1])*AtoO[1];
    dda=vec_op::dot(getcorner()[0]+getG(),AtoO);
    ai=0;
    for(int i=0;i<ta;i++){
        //da=(xlist[i]+getG()[0])*AtoO[0]+(ylist[i]+getG()[1])*AtoO[1];
        da=vec_op::dot(getcorner()[i]+getG(),AtoO);
        //std::cout<<i<<" th da : "<<da<<std::endl;
        //std::cout<<i<<" th x,y : "<<getcorner()[i][0]<<" "<<getcorner()[i][1]<<std::endl;
        if(da>dda){
            dda=da;
            ai=i;
        }
    }
    ddb=vec_op::dot((*B).getcorner()[0]+(*B).getG(),AtoO);
    bi=0;
    for(int i=0;i<tb;i++){
        //db=((*B).getxlist()[i]+(*B).getG()[0])*AtoO[0]+((*B).getylist()[i]+(*B).getG()[1])*AtoO[1];
        db=vec_op::dot((*B).getcorner()[i]+(*B).getG(),AtoO);
        //std::cout<<i<<" th db : "<<db<<std::endl;
        if(db<ddb){
            ddb=db;
            bi=i;
        }
    }

    //minimize
    xnew=(-1.0)*(getcorner()[ai]+getG())+((*B).getcorner()[bi]+(*B).getG());
    //std::cout<<"third node : "<<xnew[0]<<" "<<xnew[1]<<std::endl;
    //cout<<"thrid node index ai,bi,dda,ddb : "<<ai<<" "<<bi<<" "<<dda<<" "<<ddb<<endl;
    triangle.push_back(xnew);//07 triangle is generated.

    //std::cout<<"this is triangle : ("<<triangle[0][0]<<" "<<triangle[0][1]<<" "<<triangle[1][0]<<" "<<triangle[1][1]<<" "<<triangle[2][0]<<" "<<triangle[2][1]<<std::endl;
    
    bool flag=true;
    std::int8_t sgn=-1;

    std::uint8_t ccc=10;
    while(ccc>0){
        //std::cout<<"hoge"<<std::endl;
        ccc--;
        std::vector<double>
            v0=triangle[0]-triangle[1],//{-triangle[1][0]+triangle[0][0],-triangle[1][1]+triangle[0][1]},
            v1=triangle[1]-triangle[2],//{-triangle[2][0]+triangle[1][0],-triangle[2][1]+triangle[1][1]},
            v2=triangle[2]-triangle[0];//{-triangle[0][0]+triangle[2][0],-triangle[0][1]+triangle[2][1]};

            //三角形が作られない場合つまり外積0のときは原点を含まないことが確定?
        //if(v0[0]*v1[1]-v1[0]*v0[1]==0){
            //cout<<"on one line"<<endl;
            //return false;
        //}

        //if((v1[0])*(v2[1])-(v2[0])*(v1[1])<0){
        if(vec_op::cross_2(v1,v2)<0){
        //    std::cout<<"sgn changed."<<std::endl;
            sgn*=(-1);
        }
        
        //std::cout<<"this is triangle // inner judge : "<<std::endl;
        //for(int i=0;i<3;i++){
        //    std::cout<<triangle[i][0]<<" "<<triangle[i][1]<<std::endl;
        //}
        //std::cout<<"v0 : "<<v0[0]<<" "<<v0[1]<<std::endl;
        //std::cout<<"v1 : "<<v1[0]<<" "<<v1[1]<<std::endl;
        //std::cout<<"v2 : "<<v2[0]<<" "<<v2[1]<<std::endl;

        flag=true;
        if(sgn*vec_op::cross_2(triangle[0],v0)<=0.0){
        //    std::cout<<"1st false flag"<<std::endl;
            flag=flag&&false;
        }
        if(sgn*vec_op::cross_2(triangle[1],v1)<=0.0){
        //    std::cout<<"2nd false flag"<<std::endl;
            flag=flag&&false;
        }
        if(sgn*vec_op::cross_2(triangle[2],v2)<=0.0){
        //    std::cout<<"3rd false flag"<<std::endl;
            flag=flag&&false;
        }
        //std::cout<<"flag is "<<flag<<std::endl;

        if(flag){
        //    std::cout<<"in triangle"<<std::endl;
            goEPAFlag=true;
            break;
        }else{
            std::vector<double>
                v01=getNVector((-1.0)*triangle[1],(-1.0)*triangle[0],{0.0,0.0}),
                v12=getNVector((-1.0)*triangle[2],(-1.0)*triangle[1],{0.0,0.0}),
                v20=getNVector((-1.0)*triangle[0],(-1.0)*triangle[2],{0.0,0.0});
            
            std::cout<<"nearest vector from O to LS v01,v12,v20 : "<<v01[0]<<" "<<v01[1]<<" "<<v12[0]<<" "<<v12[1]<<" "<<v20[0]<<" "<<v20[1]<<std::endl;
            //三角形の各辺に対する原点からの最近接ベクトル

            std::vector<std::vector<double>> vlist={v01,v12,v20};
            //double minlen=sqrt(v01[0]*v01[0]+v01[1]*v01[1]);
            double minlen=vec_op::norm(v01);
            double vlistindex=0;
            for(int i=1;i<vlist.size();i++){
                //if(minlen>sqrt(vlist[i][0]*vlist[i][0]+vlist[i][1]*vlist[i][1])){
                if(minlen>vec_op::norm(vlist[i])){
                    vlistindex=i;
                }
            }
            //うち最近接のものの引数vlistindex vlistに対応

            //AtoO[0]=-vlist[vlistindex][0];
            //AtoO[1]=-vlist[vlistindex][1];//-? +? (-)=>(+)
            AtoO=(-1.0)*vlist[vlistindex];
            std::cout<<"shortest vector from O to LS : "<<AtoO[0]<<" "<<AtoO[1]<<std::endl;
        
            //dda=(xlist[0]+getG()[0])*AtoO[0]+(ylist[1]+getG()[1])*AtoO[1];
            dda=vec_op::dot(getcorner()[0]+getG(),AtoO);
            ai=0;
            //maximize dda
            for(int i=0;i<ta;i++){
                //da=(xlist[i]+getG()[0])*AtoO[0]+(ylist[i]+getG()[1])*AtoO[1];
                da=vec_op::dot(getcorner()[i]+getG(),AtoO);
                //cout<<i<<" th da : "<<da<<endl;
                if(da>dda){
                    //cout<<"da>dda"<<endl;
                    dda=da;
                    ai=i;
                }
            }
        
            //ddb=((*B).getxlist()[0]+(*B).getG()[0])*AtoO[0]+((*B).getylist()[0]+(*B).getG()[1])*AtoO[1];
            ddb=vec_op::dot((*B).getcorner()[0]+(*B).getG(),AtoO);
            bi=0;
            //minimize ddb
            for(int i=0;i<tb;i++){
                //db=((*B).getxlist()[i]+(*B).getG()[0])*AtoO[0]+((*B).getylist()[i]+(*B).getG()[1])*AtoO[1];
                db=vec_op::dot((*B).getcorner()[i]+(*B).getG(),AtoO);    //cout<<i<<" th db : "<<db<<endl;
                if(db<ddb){
                    ddb=db;
                    bi=i;
                }
            }

            //cout<<"AU-B corner A(x,y),B(x,y) : "<<xlist[ai]<<" "<<ylist[ai]<<" "<<B.getxlist()[bi]<<" "<<B.getylist()[bi]<<endl;
            xnew=(-1.0)*(getcorner()[ai]+getG())+((*B).getcorner()[bi]+(*B).getG());
            std::cout<<"4 th node : "<<xnew[0]<<" "<<xnew[1]<<std::endl;
            //cout<<"index ai,bi,da,db: "<<ai<<" "<<bi<<" "<<dda<<" "<<ddb<<endl;
            triangle.push_back(xnew);//4th node is added.

            //remove largest support f.
            std::vector<double> triangleIP;
            for(int j=0;j<triangle.size();j++){
                triangleIP.push_back(triangle[j][0]*AtoO[0]+triangle[j][1]*AtoO[1]);
            }
            //cout<<"inner products : ";
            //for(int i=0;i<triangleIP.size();i++){
            //    cout<<i<<" th ip : "<<triangleIP[i]<<" triangle : "<<triangle[i][0]<<" "<<triangle[i][1]<<endl;
            //}
            
            //maximize (-triangle) <=> minimize triangle

            int index=0;
            double maxip=(triangleIP[0]);
            for(int j=0;j<triangleIP.size();j++){
                //cout<<j<<" th inner product : "<<triangleIP[j]<<endl;
                if(maxip<(triangleIP[j])){
                    //cout<<"max,j th : "<<minip<<" "<<triangleIP[j]<<endl;
                    maxip=triangleIP[j];
                    index=j;
                }
            }
            //cout<<"removed element index, triangle, ip: "<<index<<" "<<triangle[index][0]<<" "<<triangle[index][1]<<" "<<triangleIP[index]<<endl;
            
            triangle.erase(triangle.begin()+index);

            if(distDots(triangle[0],triangle[1])<3
                ||distDots(triangle[1],triangle[2])<3
                ||distDots(triangle[2],triangle[0])<3
            ){
                //cout<<"convergent // return false"<<endl;
                //return false;
                goEPAFlag=false;
                break;
            }
            
            std::cout<<"all nodes of triangle ... "<<std::endl;
            for(int i=0;i<triangle.size();i++){
                std::cout<<i<<" th node : "<<triangle[i][0]<<" "<<triangle[i][1]<<std::endl;
            }
        }            
    }

    //EPA algorithm -------------------------------------------------
    std::vector<double>force={0.0,0.0},escape={0.0,0.0};
    std::cout<<"EPA start. init Triangle : "<<triangle[0][0]<<" "<<triangle[0][1]<<" "<<triangle[1][0]<<" "<<triangle[1][1]<<" "<<triangle[2][0]<<" "<<triangle[2][1]<<std::endl;
    bool goVWUpdateFlag=false;
    std::cout<<"goEPAFlag : "<<goEPAFlag<<std::endl;

    if(goEPAFlag){
        std::cout<<"enter EPA"<<std::endl;
    while(1){
        std::vector<double> distLineList;
        std::vector<std::vector<double>> nearestCoordinateList;

        int t=triangle.size();
        
        for(int i=0;i<t;i++){
            distLineList.push_back(
                distLine(
                    (-1.0)*triangle[i],
                    (-1.0)*triangle[(i+1)%t],
                    {0.0,0.0}));
            nearestCoordinateList.push_back(
                getNVector(
                    (-1.0)*triangle[i],
                    (-1.0)*triangle[(i+1)%t],
                    {0.0,0.0}));
        }

        std::cout<<"distLineList : "<<std::endl;
        for(int i=0;i<distLineList.size();i++){
            std::cout<<distLineList[i]<<std::endl;
        }
        std::cout<<"nearestCoordinateList"<<std::endl;
        for(int i=0;i<nearestCoordinateList.size();i++){
            std::cout<<i<<" th nc : "<<nearestCoordinateList[i][0]<<" "<<nearestCoordinateList[i][1]<<std::endl;
        }

        double mindist=distLineList[0];
        int index=0;
        for(int i=0;i<distLineList.size();i++){

            if(mindist>distLineList[i]){
                mindist=distLineList[i];
                index=i;
            }
        }
        std::cout<<"index , nearest dist , coordinate : "<<index<<", "<<distLineList[index]<<", "<<nearestCoordinateList[index][0]<<" "<<nearestCoordinateList[index][1]<<std::endl;

        //AtoO[0]=nearestCoordinateList[index][0];
        //AtoO[1]=nearestCoordinateList[index][1];
        AtoO=nearestCoordinateList[index];

        //if dist(A~O) is 0 // dist(AtoO={0,0}==0)
        //generate a new AtoO vector
        if(vec_op::norm(AtoO)==0.0){
            for(int i=0;i<t;i++){
                if(triangle[i][0]*(triangle[(i+1)%t][1]-triangle[i][1])-triangle[i][1]*(triangle[(i+1)%t][0]-triangle[i][0])==0){
                    AtoO[0]=sgn*(triangle[(i+1)%t][1]-triangle[i][1]);
                    AtoO[1]=-sgn*(triangle[(i+1)%t][0]-triangle[i][0]);
                    force=0.0*force;
                    std::cout<<"new AtoO is genereated. force={0.0,0.0}"<<std::endl;
                    break;
                }
            }
        }

        std::cout<<"vector from O to NC : "<<AtoO[0]<<" "<<AtoO[1]<<std::endl;

        dda=vec_op::dot(getcorner()[0]+getG(),AtoO);
        ai=0;
        for(int i=0;i<ta;i++){
            da=vec_op::dot(getcorner()[i]+getG(),AtoO);
            std::cout<<i<<" th da : "<<da<<std::endl;
            if(da>dda){
                
                dda=da;
                ai=i;
            }
        }
        ddb=vec_op::dot((*B).getcorner()[0]+(*B).getG(),AtoO);
        bi=0;
        for(int i=0;i<tb;i++){
            db=vec_op::dot((*B).getcorner()[i]+(*B).getG(),AtoO);   //cout<<i<<" th db : "<<db<<endl;
            std::cout<<i<<" th db : "<<db<<std::endl;
            if(db<ddb){
                ddb=db;
                bi=i;
            }
        }
        
        for(int i=0;i<triangle.size();i++){
            std::cout<<"triangle "<<i<<" th node : "<<triangle[i][0]<<" "<<triangle[i][1]<<std::endl;
        }

        //minimize
        xnew=(-1.0)*(getcorner()[ai]+getG())+((*B).getcorner()[bi]+(*B).getG());
        std::cout<<"xnew : "<<xnew[0]<<" "<<xnew[1]<<std::endl;
        
        //xnew 閉曲線になる位置にpush


        unsigned short insertindex=0;

        //same node is added
        bool samenodeFlag=false;
        for(int i=0;i<t;i++){
            double hoge=sgn*vec_op::cross_2(triangle[i%t]-xnew,triangle[i%t]-triangle[(i+1)%t]);
            //2*2 cross is undefined.
            //double hoge=sgn*vec_op::dot(triangle[i%t]-xnew,(-1.0)*triangle[(i+1)%t])
            //cout<<i<<" th sgn : "<<hoge<<endl;
            if(triangle[i%t]==xnew){
                insertindex=(i+1)%t;
                force=(-1.0)*AtoO;
                goVWUpdateFlag=true;
                std::cout<<"same node is added break // force=-AtoO"<<std::endl;
                samenodeFlag=true;
                break;
            }

        }
        if(samenodeFlag){
            std::cout<<"go out of while loop. same node flag"<<std::endl;
            break;
        }

        //node is added
        for(int i=0;i<t;i++){
            if(sgn*((triangle[i%t][0]-xnew[0])*(-triangle[(i+1)%t][1]+triangle[i%t][1])-(triangle[i%t][1]-xnew[1])*(-triangle[(i+1)%t][0]+triangle[i%t][0]))<=0){
                //理論上一回だけこの中に入る
                //xnewがtriangle辺上にあるときは二回0を返す
                std::cout<<i<<" th element is inserted"<<std::endl;
                insertindex=(i+1)%t;
                //if(triangle[i%t][0]==xnew[0]&&triangle[i%t][1]==xnew[1]){
                    //  insertindex=i%t;
                //}else if(triangle[(i+1)%t][0]==xnew[0]&&triangle[(i+1)%t][1]==xnew[1]){
                    //  insertindex=(i+1)%t;
                //}
                force=(-1.0)*AtoO;
                //force[0]=-AtoO[0];
                //force[1]=-AtoO[1];
                std::cout<<"normal break force=-AtoO"<<std::endl;
                break;
            }
        } 

        //if(insertindex==t-1){
        //    cout<<"insertindex==t-1"<<endl;
        //    triangle.push_back(xnew);
        //}else{
            //cout<<"insertindex="<<insertindex<<endl;
            triangle.insert(triangle.begin()+insertindex,xnew);
        //}
        //cout<<"inserindex : "<<insertindex<<endl;
        t=triangle.size();

        for(int i=0;i<triangle.size();i++){
            std::cout<<"triangle "<<i<<" th node : "<<triangle[i][0]<<" "<<triangle[i][1]<<std::endl;
        }


        if(t>10||distDots(escape,xnew)<1){
            force=(-1.0)*AtoO;
            std::cout<<"convergent // force=-AtoO"<<std::endl;
            goVWUpdateFlag=true;
            break;
        }
        
        escape=xnew;

        std::cout<<"final dda,ddb : "<<dda<<" "<<ddb<<std::endl;
        
    }
    }
    std::cout<<"GJK EPA force is over. force : "<<force[0]<<" "<<force[1]<<std::endl;
    return force;
}

void Triangle::Collision(Triangle *B){
    //cout<<"GJK reffers the address : "<<B<<endl;
    std::vector<double> collision,nearestDot,force={0.0,0.0};
    bool vwHasChanged=false;
    std::vector<std::vector<double>>Qnlist,ralist,rblist;
    
    for(int bbb=0;bbb<2;bbb++){
        
    force=GJK_EPA_force(B);
    std::cout<<"force : "<<force[0]<<" "<<force[1]<<std::endl;
    if(vec_op::norm(force)==0.0){
        std::cout<<"force is 0. A pos is "<<getG()[0]<<" "<<getG()[1]<<std::endl;
        if(bbb>0){
            std::cout<<"loop is over. bbb : "<<bbb<<std::endl;
            if(getstop()){
                PosUpdate(-1.0*bbb);
                RotUpdate(-1.0);
            }
            if((*B).getstop()){
                (*B).PosUpdate(-1.0*bbb);
                (*B).RotUpdate(-1.0);
            }
        }
        return;
    }
    
    //V,W update-----------------------------------------------------
    
    //接触点
    //fの方向にf*1.1移動 
    //dots vs lines 最小組について垂線の脚の座標を取得px,py
    //px,py移動
    //A,B xg,yg->xg+d,yg+d

    //if(goVWUpdateFlag){
        //cout<<"enter VWUpdate ..."<<endl;
        //setG({getG()[0]+force[0]*1.01,getG()[1]+force[1]*1.1});
        //B.setG({B.getG()[0]-force[0]*1.01,B.getG()[1]-force[1]*1.1});
        

        //if(!vwHasChanged){
        //    vwHasChanged=true;
            
        //cout<<"before shift G x,y : "<<getG()[0]<<" "<<getG()[1]<<endl;
        //set v,w 
        if(!getstop()&&!getjoint()){
            shiftColPos(1.1*force);
        }
        //cout<<"after shift G x,y : "<<getG()[0]<<" "<<getG()[1]<<endl;

        if(!(*B).getstop()&&!(*B).getjoint()){
            //std::vector<double> minusf={-force[0]*1.1,-force[1]*1.1};
            (*B).shiftColPos((-1.0)*force);
        }

        short ta=getxlist().size();
        short tb=(*B).getxlist().size();
        
        double len,minlen=distLine(getcorner()[0]+getG(),getcorner()[1]+getG(),(*B).getcorner()[0]+(*B).getG());
        short ia=0,ib=0;
        char x='a';//xlist 's owner...a or b
        
        for(int i=0;i<ta;i++){
            for(int j=0;j<tb;j++){
                len=distLine(getcorner()[i]+getG(),getcorner()[(i+1)%ta]+getG(),(*B).getcorner()[j]+(*B).getG());
                if(len<minlen){
                    minlen=len;
                    ia=i;
                    ib=j;
                    x='a';
                }
            }
        }

        //ta=(*B).getxlist().size();
        //tb=(*B).getxlist().size();

        for(int i=0;i<tb;i++){
            for(int j=0;j<ta;j++){
                len=distLine((*B).getcorner()[i]+(*B).getG(),(*B).getcorner()[(i+1)%ta]+(*B).getG(),getcorner()[j]+getG());
                if(len<minlen){
                    minlen=len;
                    ia=j;
                    ib=i;
                    x='b';
                }
            }
        }
        
        if(x=='a'){
            //ta=xlist.size();
            //tb=xlist.size();
            //cout<<"x is a."<<endl;

            collision=getNVector(getcorner()[ia]+getG(),getcorner()[(ia+1)%ta]+getG(),(*B).getcorner()[ib]+(*B).getG());
            nearestDot=(*B).getcorner()[ib]+(*B).getG();
        }else if(x=='b'){
            //ta=(*B).xlist.size();
            //tb=(*B).xlist.size();
            //cout<<"x is b."<<endl;

            collision=getNVector((*B).getcorner()[ib]+(*B).getG(),(*B).getcorner()[(ib+1)%tb]+(*B).getG(),getcorner()[ia]+getG());
            nearestDot=getcorner()[ia]+getG();
        }

        if(!getstop()&&!getjoint()){
            shiftColPos(collision);
        }
        if(!(*B).getstop()&&!(*B).getjoint()){
            (*B).shiftColPos((-1.0)*collision);
        }

        //update v,w
        
        
        double Q,a,b;//v_ab ... relative velocity
        std::vector<double> 
        ra=getjoint()?((nearestDot+collision)-getfix()):((nearestDot+collision)-getG()),
        rb=(*B).getjoint()?((nearestDot+collision)-(*B).getfix()):((nearestDot+collision)-(*B).getG()),
        nn=(1/vec_op::norm(force))*force,
        hh={-nn[1],nn[0]},
        v_ab={(getV()[0]-(*B).getV()[0]+(-ra[1]*getW()+rb[1]*(*B).getW())),getV()[1]-(*B).getV()[1]+(ra[0]*getW()-rb[0]*(*B).getW())};
        
        //ra...Aの衝突地点の相対座標　基準点は自由運動なら重心、ジョイントありなら固定点
        //rb...Bの衝突地点の相対座標
        //nn...法線ベクトル
        //hh...接線ベクトル        
        
        std::cout<<"ra,rb : ("<<ra[0]<<" "<<ra[1]<<") ("<<rb[0]<<" "<<rb[1]<<")"<<std::endl;

        if(
            !getstop()
            &&vec_op::norm(getV())<1.0//remember 
            &&getW()*getW()<0.01//remember
        ){
            std::cout<<"A anteika--------------------"<<std::endl;
            //setstop(1);
        }
        if(
            !(*B).getstop()
            &&vec_op::norm((*B).getV())<1.0
            &&(*B).getW()*(*B).getW()<0.01
        ){
            std::cout<<"B anteika--------------------"<<std::endl;
            //(*B).setstop(1);
        }
        
        a=(vec_op::dot(ra,ra)-pow(vec_op::dot(ra,nn),2))/getI();
        b=(vec_op::dot(rb,rb)-pow(vec_op::dot(rb,nn),2))/(*B).getI();
        Q=(1+getE()*(*B).getE())*(v_ab[0]*nn[0]+v_ab[1]*nn[1])/(1/getM()+1/(*B).getM()+a+b);

        if(!getstop()&&bbb==0){
            if(!getjoint()){
                setV(getV()-(Q/getM())*nn-(vec_op::dot(getV(),hh)*getdfc()/vec_op::norm(getV()))*hh);
            }
            setW(getW()-(Q/getI())*vec_op::cross_2(ra,nn));

        }

        if(!(*B).getstop()&&bbb==0){
            if(!(*B).getjoint()){
                (*B).setV((*B).getV()+(Q/(*B).getM())*nn+(vec_op::dot((*B).getV(),hh)*(*B).getdfc()/vec_op::norm((*B).getV()))*hh);
            }
            (*B).setW((*B).getW()+(Q/(*B).getI())*vec_op::cross_2(rb,nn));

        }

        //std::cout<<"A Pos : "<<getG()[0]<<" "<<getG()[1]<<std::endl;
        //std::cout<<"B Pos : "<<(*B).getG()[0]<<" "<<(*B).getG()[1]<<std::endl;
        //std::cout<<"A V x,y : "<<getV()[0]<<" "<<getV()[1]<<std::endl;
        //std::cout<<"A W : "<<getW()<<" "<<std::endl;
        //std::cout<<"B V x,y : "<<(*B).getV()[0]<<" "<<(*B).getV()[1]<<std::endl;
        //std::cout<<"B W : "<<(*B).getW()<<" "<<std::endl;
        
        if(getstop()){
            PosUpdate(1.0);
            if(bbb==0){
                RotUpdate(1.0);
            }
        }
        if((*B).getstop()){
            (*B).PosUpdate(1.0);
            if(bbb==0){
                RotUpdate(1.0);
            }
        }
    }

    std::cout<<"GJK is over..."<<std::endl<<std::endl;
    //return goEPAFlag;
}


void Triangle::addJoint(std::vector<double> ff){
    joint=1;
    fix=ff;
    If=I;
}

void Triangle::removeJoint(){
    joint=0;
    delete &fix;
}


void frameManager(){
while(true){
    Sleep(GV.getspf());
    GV.setframeFlag(true);
}
    return;
}

void gamedisplay(){
	GV.setframeFlag(false);

    Triangle testA=Triangle({{50,0},{150,0},{50,100}},1,0.5,2,1,0,{0,0});
    testA.addJoint({50,0});

    Triangle testB=Triangle({{0,0},{100,-100},{0,-100}},1,0.5,2,1,-0.1,{0,-5});
    Triangle testC=Triangle({{100,-200},{100,-300},{0,-300}},1,0.5,2,1,0.1,{0,-3});
    Triangle floor=Triangle({{200,150},{-200,150},{0,200}},1,0.5,2,1,0,{0,0});
    //testA.setstop(1);
    //testB.setstop(1);
    testC.setstop(1);
    floor.setstop(1);
    std::vector<Triangle> testlist={testA,testB,testC,floor};

    std::vector<cv::Point> plistA,plistB,plistC,plistf;
    for(int i=0;i<4;i++){
        plistA.push_back(
            cv::Point(
                (int)(GV.getCW()/2+testA.getcorner()[i%3][0]+testA.getG()[0]),
                (int)(GV.getCH()/2+testA.getcorner()[i%3][1]+testA.getG()[1])
            )
        );
        plistB.push_back(
            cv::Point(
                (int)(GV.getCW()/2+testB.getcorner()[i%3][0]+testB.getG()[0]),
                (int)(GV.getCH()/2+testB.getcorner()[i%3][1]+testB.getG()[1])
            )
        );
        plistC.push_back(
             cv::Point(
                (int)(GV.getCW()/2+testB.getcorner()[i%3][0]+testB.getG()[0]),
                (int)(GV.getCH()/2+testB.getcorner()[i%3][1]+testB.getG()[1])
             )
        );
        plistf.push_back(
            cv::Point(
                (int)(GV.getCW()/2+floor.getcorner()[i%3][0]+floor.getG()[0]),
                (int)(GV.getCH()/2+floor.getcorner()[i%3][1]+floor.getG()[1])
            )
        );
    }

    std::vector<std::vector<cv::Point>> plistlist={plistA,plistB,plistC,plistf};
 
while(1){
    cv::Mat img(cv::Size(GV.getCW(),GV.getCH()),CV_8UC3,cv::Scalar(0,0,0));
    
    for(int i=0;i<testlist.size();i++){
        if(!testlist[i].getstop()){
            if(!testlist[i].getjoint()){
                testlist[i].update();
            }else{
                testlist[i].Jointupdate();
            }
        }
    }
    
    //collision i vs j (i<j)
    //for(int arufa=0;arufa<2;arufa++){
    for(int i=0;i<testlist.size();i++){
        for(int j=i+1;j<testlist.size();j++){          
            if(testlist[i].AABB(&(testlist[j]))){
                std::cout<<"-----------------------------------"<<i<<" vs "<<j<<" Collision-----------------------------"<<std::endl;
                testlist[i].Collision(&(testlist[j]));
            }            
        }
    }
      

    for(int i=0;i<plistlist.size();i++){
        plistlist[i].clear();
    }


    for(int i=0;i<testlist.size();i++){
        std::vector<cv::Point> plisttest;
        for(int j=0;j<4;j++){
            plisttest.push_back(
                cv::Point(
                    (int)(GV.getCW()/2+testlist[i].getcorner()[j%3][0]+testlist[i].getG()[0]),
                    (int)(GV.getCH()/2+testlist[i].getcorner()[j%3][1]+testlist[i].getG()[1])
                )
            );
        }
        plistlist.push_back(plisttest);
    }

    
    for(int i=0;i<plistlist.size();i++){
        cv::polylines(img,plistlist[i],true,cv::Scalar(0xff,0,0xff));
    }

    cv::imshow("test",img);
    //cv::waitKey(0);
    img.release();
    
    bool event_is_empty=true;
    while(!GV.getframeFlag()){
        cv::waitKey(GV.getspf());


        event_is_empty=false;
    }
    GV.setframeFlag(false);
}
    return;
}

int main(){
    
    std::thread th_main(gamedisplay);
    std::thread th_frame(frameManager);

    th_main.join();
    th_frame.join();

        
    return 0;
}