#include <iostream>
#include <string>
#include <typeinfo>
#include <math.h>
#include <random>
#include <vector>
#include <algorithm>
#include <Windows.h>

#include <opencv2/opencv.hpp>

using namespace std;

#define CANVAS_WIDTH 800
#define CANVAS_HEIGHT 400

#define GX 0
#define GY 0.5
#define VMax 10

class some_exception{
private:
    const char* msg;
public:
    some_exception(const char* msg):msg(msg){}
    const char* what(){return msg;}
};

class Matrix{//2*2
private:
    vector<vector<double>> a;
public:
    Matrix(vector<vector<double>> aa){
        try{
            for(int i=0;i<2;i++){
                for(int j=0;j<2;j++){
                    a[i][j]=aa[i][j];
                }
            }
        }catch(some_exception e){
            cerr<<"some_exception: "<<e.what()<<std::endl;
            cout<<"Matrix constructor only accepts 2*2 vector"<<endl;
        }
    }
    ~Matrix(){

    }
};

class Stone{
protected:
    double x,y,m,e,vx,vy,r;
public:
    Stone(double xx,double yy,double mm,double ee,double vxx,double vyy,double rr){
        x=xx;y=yy;m=mm;e=ee;vx=vxx;vy=vyy;r=rr;
    }
    ~Stone(){

    }
    double getx(){
        return x;
    }
    double gety(){
        return y;
    }
    double gete(){
        return e;
    }
    double getvx(){
        return vx;
    }
    double getvy(){
        return vy;
    }
    double getr(){
        return r;
    }
    void setx(double xx){
        x=xx;
    }
    void sety(double yy){
        y=yy;
    }
    void setvx(double vxx){
        vx=vxx;
        if(vx>VMax)vx=VMax;
        if(vx<(-VMax))vx=(-VMax);
        if(vx>0)vx=vx-0.01;
        if(vx<0)vx=vx+0.01;
    }
    void setvy(double vyy){
        vy=vyy;
        if(vy>VMax)vy=VMax;
        if(vy<(-VMax))vy=(-VMax);
    }
};

class Rectangle:public Stone{
    
};
class MeMe : public Stone{

};

Stone meme=Stone(100,100,1.0,0.0,0,0,10.0);

class Collider{
private:
    vector<vector<int>> lineSegment;//line segment (x1,y1),(x2,y2)
public:
    Collider(vector<vector<int>>ls){
        lineSegment=ls;
    }
    ~Collider(){

    }
    
    int getxy(int a,int b){
        if(a==0||a==1||b==0||b==1){
            return lineSegment[a][b];
        }else{
            return 0;
        }
    }

    void setxy(int a,int b,int val){
        if(a==0||a==1||b==0||b==1){
            lineSegment[a][b]=val;
        }
    }

    void collisionJudge(Stone* stone_ptr){
        double x_old,y_old,x_new,y_new,f_old,f_new;
	
        x_old=(*stone_ptr).getx();
        y_old=(*stone_ptr).gety();
        x_new=(*stone_ptr).getx()+(*stone_ptr).getvx();
        y_new=(*stone_ptr).gety()+(*stone_ptr).getvy();
        
        //cout<<"head collider : "<<x_old<<" "<<y_old<<" "<<x_new<<" "<<y_new<<endl;
        
        if(lineSegment[0][0]==lineSegment[1][0]){//x1=x2,y=a;
            cout<<"vertical hit"<<endl;
            int x0=lineSegment[0][0];
            int minY,maxY;
            minY=lineSegment[0][1]<=lineSegment[1][1]?lineSegment[0][1]:lineSegment[1][1];
            maxY=lineSegment[0][1]>lineSegment[1][1]?lineSegment[0][1]:lineSegment[1][1];            
            
            if((minY<=y_old)&&(y_old<=maxY)){
                double r_ptr=(*stone_ptr).getr();
                f_old=x_old-x0;
                f_new=x_new-x0;
                
                if(abs(f_new)<r_ptr&&abs(f_old)>r_ptr){//||f_old*f_new<0
                    int sgn=1;
                    if(f_old>0){
                        sgn*=(-1);
                        cout<<"sgn is negative"<<endl;
                    }

                    (*stone_ptr).setx(x0-r_ptr*sgn);
                    (*stone_ptr).setvx(-(*stone_ptr).getvx());
                    return;
                }else{
                    return;
                }
            }
        }else{//y=a*x+b
            int x1,y1,x2,y2;
            bool flag=false;
            double ratio,thrd;
            thrd=0.2;
            x1=lineSegment[0][0];
            y1=lineSegment[0][1];
            x2=lineSegment[1][0];
            y2=lineSegment[1][1];
            ratio=(double)(y2-y1)/(double)(x2-x1);

            //cout<<"ratio : "<<ratio<<endl;
            f_old=ratio*(x_old-x1)-(y_old-y1);
            f_new=ratio*(x_new-x1)-(y_new-y1);
            
            int maxX,minX,maxY,minY;
            maxX=x1>=x2?x1:x2;
            minX=x1<x2?x1:x2;
            maxY=y1>=y2?y1:y2;
            minY=y1<y2?y1:y2;

            if(maxX-minX<20){
                minX-=10;
                maxX+=10;
            }
            if(maxY-minY<20){
                minY-=10;
                maxY+=10;
            }

            flag=((minX<=x_old)&&(x_old<=maxX))&&((minY<=y_old)&&(y_old<=maxY));
            
            if(flag){
                //cout<<"flag is true // near collider // f_old,new : "<<f_old<<" "<<f_new<<endl;
                
                if(abs(f_old)<(*stone_ptr).getr()||f_old*f_new<0){
                    double vx,vy,cx,cy,costh,sinth,th,inner_prd,cross_prd;
                    //velocity
                    vx=(*stone_ptr).getvx();
                    vy=(*stone_ptr).getvy();
                    
                    double r_ptr;//intersection(x,y)
                    r_ptr=(*stone_ptr).getr();

                                        //cout<<"isx,isy : "<<isx<<" "<<isy<<endl;

                    //collider
                    cx=x2-x1;
                    cy=y2-y1;
		    
                    inner_prd=vx*cx+vy*cy;
                    cross_prd=vx*cy-vy*cx;
                    
                    if(cross_prd<0){
                    //cout<<"inner, cross : "<<inner_prd<<" "<<cross_prd<<endl;
                    costh=inner_prd/(sqrt(vx*vx+vy*vy)*sqrt(cx*cx+cy*cy));
                    th=acos(costh);

                    double vx2,vy2,e_ptr;
                    
                    int sgn=-1;

                    //rotate
                    vx2=vx*cos(2*sgn*th)-vy*sin(2*sgn*th);
                    vy2=vx*sin(2*sgn*th)+vy*cos(2*sgn*th);
                    e_ptr=(*stone_ptr).gete();
                    //cout<<"vel vx,vx,vx2,vy2 : "<<vx<<" "<<vy<<" "<<vx2<<" "<<vy2<<endl;
                    
                    double norm_v2=sqrt(vx2*vx2+vy2*vy2);
                    
                    double isx,isy;

                    double det,a,b,c,d,s,t,cx_n,cy_n;
                    int sgn_vx,sgn_vy;
                    a=cy;
                    b=-cx;
                    c=vy;
                    d=-vx;

                    cx_n=x1/sqrt(x1*x1+y1*y1);
                    cy_n=y1/sqrt(x1*x1+y1*y1);

                    sgn_vx=1;

                    if(vy>0){
                        sgn_vy=1;
                    }else{
                        sgn_vy=-1;
                    }
                    
                    s=cy*(x1+sgn_vx*sgn_vy*r_ptr*cy_n)-cx*(y1-sgn_vx*sgn_vy*r_ptr*cx_n);
                    t=vy*x_old-vx*y_old;

                    det=a*d-b*c;
                    
                    if(det!=0){
                        isx=1/det*(d*s-b*t);
                        isy=1/det*(-c*s+a*t);
                        //cout<<"isx isy "<<cy/cx*(isx-lineSegment[0][0])+lineSegment[0][1]-isy<<endl;
                        cout<<"d from collider : "<<abs(cy*(isx-lineSegment[0][0])-cx*(isy-lineSegment[0][1]))/sqrt(cy*cy+cx*cx)<<endl;
                    }

                    if(norm_v2>0.1){
                        (*stone_ptr).setx(isx);
                        (*stone_ptr).sety(isy);
                        (*stone_ptr).setvx(vx2*e_ptr);
                        (*stone_ptr).setvy(vy2*e_ptr);
                    }else{
                        double vtanth,vth;
                        vtanth=vy/vx;
                        vth=atan(vtanth);//when collider is horizontal, vth=0. 
                        
                        (*stone_ptr).setx(isx);
                        (*stone_ptr).sety(isy);
                        (*stone_ptr).setvx(GX*cos(vth)-GY*sin(vth));
                        (*stone_ptr).setvy(GX*sin(vth)-GY*cos(vth));
                    }
                    }//if(cross_ptr)
                    return;
                }else{
                    return;
                }
            }else{
                //hit around corner
                double l_old,l_new,r_ptr,e_ptr,flag=false;
                r_ptr=(*stone_ptr).getr();
                e_ptr=(*stone_ptr).gete();
                //for(int i=0;i<2;i++){
                int i=0;
                l_old=sqrt((lineSegment[i][0]-x_old)*(lineSegment[i][0]-x_old)+(lineSegment[i][1]-y_old)*(lineSegment[i][1]-y_old));
                l_new=sqrt((lineSegment[i][0]-x_new)*(lineSegment[i][0]-x_new)+(lineSegment[i][1]-y_new)*(lineSegment[i][1]-y_new));
                if(l_old>=r_ptr&&l_new<r_ptr){
                    flag=true;
                }
                //}
                if(flag){
                    double vx,vy,sx,sy,c1x,c1y,isx,isy,x1,y1,x2,y2,a,b,c;
                    vx=(*stone_ptr).getvx();
                    vy=(*stone_ptr).getvy();
                    sx=(*stone_ptr).getx();
                    sy=(*stone_ptr).gety();
                    c1x=lineSegment[0][0];
                    c1y=lineSegment[0][1];
                    
                    a=vy*sx-vx*sy-vy*c1x;
                    b=-vy*c1y;
                    c=vy*r_ptr;

                    y1=((-a*vx+b*vy)-sqrt(c*c*(vx*vx+vy*vy)-(a*vy-b*vx)*(a*vy-b*vx)))/sqrt(vx*vx+vy*vy);
                    y2=((-a*vx+b*vy)+sqrt(c*c*(vx*vx+vy*vy)-(a*vy-b*vx)*(a*vy-b*vx)))/sqrt(vx*vx+vy*vy);
    
                    a=-vx*c1x;
                    b=-vy*sx+vx*sy-vx*c1y;
                    c=vx*r_ptr;

                    x1=(-(a*vx+b*vy)-abs(c*c*(vx*vx+vy*vy)-(a*vy-b*vx)*(a*vy-b*vx)))/sqrt(vx*vx+vy*vy);
                    x2=(-(a*vx+b*vy)+abs(c*c*(vx*vx+vy*vy)-(a*vy-b*vx)*(a*vy-b*vx)))/sqrt(vx*vx+vy*vy);
		    
                    vector<double> v1,v2;
                    //double ip1,ip2,ip3,ip4,ipmax,ipmin;
                    double ip1,ip2;

                    v1={x2,y2};
                    //v2={x2,y1};
                    //v3={x1,y2};
                    v2={x1,y1};

                    ip1=v1[0]*vx+v1[1]*vy;
                    //ip2=v2[0]*vx+v2[1]*vy;
                    //ip3=v3[0]*vx+v3[1]*vy; 
                    ip2=v2[0]*vx+v2[1]*vy;

                    //cout<<"1,2,3,4:"<<ip1<<" "<<ip2<<" "<<ip3<<" "<<ip4<<endl;
		    
                    if(ip1>0){
                        isx=v1[0];isy=v1[1];
                    }else{
                        isx=v2[0];isy=v2[1];
                    }

                    double inner_prd,cross_prd,costh,th;
                    inner_prd=((c1x-isx)*vx+(c1y-isy)*vy);
                    cross_prd=((c1x-isx)*vx-(c1y-isy)*vy);
                    costh=inner_prd/(sqrt((c1x-isx)*(c1x-isx)+(c1y-isy)*(c1y-isy))*sqrt(vx*vx+vy*vy));
                    th=acos(costh);

                    int sgn=1;                    
                    if(cross_prd<0)sgn*=(-1);

                    double vx2,vy2;
                    
                    vx2=vx*cos(th*sgn)-vy*sin(th*sgn);
                    vy2=vx*sin(th*sgn)+vy*cos(th*sgn);

                    //(*stone_ptr).setx(isx);
                    //(*stone_ptr).sety(isy);
                    
                    (*stone_ptr).setvx(vx2*e_ptr);
                    (*stone_ptr).setvy(vy2*e_ptr);
                }

                return;
            }
        }
    }
};

vector<Stone> stones;
vector<Collider> colliders;

void title(){
    cv::Mat img(cv::Size(CANVAS_WIDTH,CANVAS_HEIGHT),CV_8UC3,cv::Scalar(0,0,0));
     
    putText(img,"FOREVER",cv::Point(CANVAS_WIDTH/2,CANVAS_HEIGHT/2-40),1,2.4,cv::Scalar(0xff,0xff,0xff));
    putText(img,"PRESS ANY KEY",cv::Point(CANVAS_WIDTH/2,CANVAS_HEIGHT/2),1,1.2,cv::Scalar(0xff,0xff,0xff));
    imshow("test",img);
    cv::waitKey(0);
    img.release();
}

void stoneInitialize(){
    stones.push_back(Stone(400,10,1.0,0.9,3.1,-2.0,10.0));
    stones.push_back(Stone(200,10,1.0,0.9,-2.1,4.0,10.0));

    //cout<<"check : "<<stones[0].getx()<<endl;
    //cout<<"initial e : "<<stones[0].gete()<<endl;
}

void generateInnerCollider(int x,int y,double rot,int W,int H){
    double PI=3.14;
    double th=rot*PI/180;
    colliders.push_back(Collider({{(int)(x+W/2*(double)cos(th)+H/2*(double)sin(th)),(int)(y+W/2*(double)sin(th)-H/2*(double)cos(th))},{(int)(x-W/2*(double)cos(th)+H/2*(double)sin(th)),(int)(y-W/2*(double)sin(th)-H/2*(double)cos(th))}}));
    colliders.push_back(Collider({{(int)(x-W/2*(double)cos(th)+H/2*(double)sin(th)),(int)(y-W/2*(double)sin(th)-H/2*(double)cos(th))},{(int)(x-W/2*(double)cos(th)-H/2*(double)sin(th)),(int)(y-W/2*(double)sin(th)+H/2*(double)cos(th))}}));
    colliders.push_back(Collider({{(int)(x-W/2*(double)cos(th)-H/2*(double)sin(th)),(int)(y-W/2*(double)sin(th)+H/2*(double)cos(th))},{(int)(x+W/2*(double)cos(th)-H/2*(double)sin(th)),(int)(y+W/2*(double)sin(th)+H/2*(double)cos(th))}}));
    colliders.push_back(Collider({{(int)(x+W/2*(double)cos(th)-H/2*(double)sin(th)),(int)(y+W/2*(double)sin(th)+H/2*(double)cos(th))},{(int)(x+W/2*(double)cos(th)+H/2*(double)sin(th)),(int)(y+W/2*(double)sin(th)-H/2*(double)cos(th))}}));
}

void generateCollider(int x,int y,double rot,int W,int H){
    double PI=3.14;
    double th=rot*PI/180;
    
    colliders.push_back(Collider({

        {(int)(x+W/2*(double)cos(th)+H/2*(double)sin(th)),(int)(y+W/2*(double)sin(th)-H/2*(double)cos(th))},
        {(int)(x+W/2*(double)cos(th)-H/2*(double)sin(th)),(int)(y+W/2*(double)sin(th)+H/2*(double)cos(th))}
    
    }));
    
    colliders.push_back(Collider({
    
        {(int)(x+W/2*(double)cos(th)-H/2*(double)sin(th)),(int)(y+W/2*(double)sin(th)+H/2*(double)cos(th))},
        {(int)(x-W/2*(double)cos(th)-H/2*(double)sin(th)),(int)(y-W/2*(double)sin(th)+H/2*(double)cos(th))}
    
    }));
    
    colliders.push_back(Collider({
    
        {(int)(x-W/2*(double)cos(th)-H/2*(double)sin(th)),(int)(y-W/2*(double)sin(th)+H/2*(double)cos(th))},
        {(int)(x-W/2*(double)cos(th)+H/2*(double)sin(th)),(int)(y-W/2*(double)sin(th)-H/2*(double)cos(th))}
    
    }));
    
    colliders.push_back(Collider({
    
        {(int)(x-W/2*(double)cos(th)+H/2*(double)sin(th)),(int)(y-W/2*(double)sin(th)-H/2*(double)cos(th))},
        {(int)(x+W/2*(double)cos(th)+H/2*(double)sin(th)),(int)(y+W/2*(double)sin(th)-H/2*(double)cos(th))}
    
    }));
    

    
    
}



void colliderInitialize(){
    generateCollider(100,100,20,90,20);
    generateCollider(200,300,30,100,20);
    generateCollider(350,200,-10,100,20);
    generateCollider(500,200,-20,100,20);
    generateCollider(600,300,20,100,20);
    generateCollider(750,200,10,20,200);
    generateCollider(400,100,10,20,100);
}

void randomVel(){
    for(int i=0;stones.size();i++){
        stones[i].setvx(5.0);
        stones[i].setvy(4.0);
    }
}

void controllerInitialize(){
    

}
//int counter=0;
void gamedisplay(){
for(int huga=0;true;){
    cv::Mat img(cv::Size(CANVAS_WIDTH,CANVAS_HEIGHT),CV_8UC3,cv::Scalar(0,0,0));
    line(img,cv::Point(10,CANVAS_HEIGHT-10),cv::Point(CANVAS_WIDTH-10,CANVAS_HEIGHT-10),cv::Scalar(0xff,0xff,0xff));

    for(int i=0;i<stones.size();i++){
	    //if(stones[i].getOnCollider()){

        //}
        stones[i].setvx(stones[i].getvx()+GX);
        stones[i].setvy(stones[i].getvy()+GY);
        double x_new,y_new;
        x_new=stones[i].getx()+stones[i].getvx();
        y_new=stones[i].gety()+stones[i].getvy();
        if(x_new<0)x_new+=CANVAS_WIDTH;
        if(x_new>CANVAS_WIDTH)x_new-=CANVAS_WIDTH;
        if(y_new<0)y_new+=CANVAS_HEIGHT;
        if(y_new>CANVAS_HEIGHT)y_new-=CANVAS_HEIGHT;
        stones[i].setx(x_new);
        stones[i].sety(y_new);
    }

    meme.setvx(meme.getvx()+GX);
    meme.setvy(meme.getvy()+GY);
    double memex,memey;
    memex=meme.getx()+meme.getvx();
    memey=meme.gety()+meme.getvy();
    if(memex<0)memex+=CANVAS_WIDTH;
    if(memex>CANVAS_WIDTH)memex-=CANVAS_WIDTH;
    if(memey<0)memey+=CANVAS_HEIGHT;
    if(memey>CANVAS_HEIGHT)memey-=CANVAS_HEIGHT;
    meme.setx(memex);
    meme.sety(memey);

    //collider
    for(int i=0;i<colliders.size();i++){
        line(img,cv::Point(colliders[i].getxy(0,0),colliders[i].getxy(0,1)),
                cv::Point(colliders[i].getxy(1,0),colliders[i].getxy(1,1)),cv::Scalar(0xff,0xff,0xff));
        for(int j=0;j<stones.size();j++){
            Stone* stone_ptr=&stones[j];
            colliders[i].collisionJudge(stone_ptr);
        }
        Stone* meme_ptr=&meme;
        colliders[i].collisionJudge(meme_ptr);    
    }
    
    //stone


    //draw
    for(int i=0;i<stones.size();i++){
        circle(img,cv::Point(stones[i].getx(),stones[i].gety()),stones[i].getr(),cv::Scalar(0xff,0,0xff),-1);
    }

    circle(img,cv::Point(meme.getx(),meme.gety()),10,cv::Scalar(0,0xff,0xff));
    cout<<"meme vel x,y : "<<meme.getvx()<<" "<<meme.getvy()<<endl;

    imshow("test",img);
    img.release();
    
    switch(cv::waitKey(17)){
        case 97://a
            meme.setvx(-10);
            
            //cv::waitKey(17);
            break;
        case 100:
            meme.setvx(10);
            
            //cv::waitKey(17);
            break;
        case 106://j
            meme.setvy(-30);
            //cv::waitKey(17);
            break;
        default:
            meme.setvx(0);
            break;
    };

        //counter++;
}
}

int main(){
    cout<<"hello world"<<endl;
    title();
    stoneInitialize();
    colliderInitialize();
    //controllerInitialize();
    gamedisplay();
}