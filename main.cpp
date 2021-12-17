#include <iostream>
#include <math.h>
#include <random>
#include <vector>
#include <algorithm>

#include <opencv2/opencv.hpp>

using namespace std;

#define PI 3.1415926535

#define CANVAS_WIDTH 400
#define CANVAS_HEIGHT 400


cv::Mat canvas;
//cv::Mat img(cv::Size(CANVAS_WIDTH,CANVAS_HEIGHT),CV_8UC3,cv::Scalar(0,0,0));

int counter=0;

void bbb(double x1,double y1,double x2,double y2){
    double r=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
    

    if(r>5.0){
        //std::cout<<"roop"<<endl;
        double vx=(x2-x1)*cos(PI/2)-(y2-y1)*sin(PI/2);
        double vy=(x2-x1)*sin(PI/2)+(y2-y1)*cos(PI/2);
        //std::cout<<"vx,vy "<<vx<<endl<<vy<<endl;
        
        bbb((double)x1,(double)y1,(double)(x1+(x2-x1)/3),(double)(y1+(y2-y1)/3)); 
        bbb((double)(x1+(x2-x1)/3),(double)(y1+(y2-y1)/3),(double)(x1+(x2-x1)/3+vx/3),(double)(y1+(y2-y1)/3+vy/3)); 
        bbb((double)(x1+(x2-x1)/3+vx/3),(double)(y1+(y2-y1)/3+vy/3),(double)(x2-(x2-x1)/3+vx/3),(double)(y1+(y2-y1)/3+vy/3));
        bbb((double)(x2-(x2-x1)/3+vx/3),(double)(y1+(y2-y1)/3+vy/3),(double)(x2-(x2-x1)/3),(double)(y2-(y2-y1)/3));     
        bbb((double)(x2-(x2-x1)/3),(double)(y2-(y2-y1)/3),(double)x2,(double)y2);
    }else{
        line(canvas,cv::Point((int)(x1+CANVAS_WIDTH/2),(int)(y1+CANVAS_HEIGHT/2)),cv::Point((int)(x2+CANVAS_WIDTH/2),(int)(y2+CANVAS_HEIGHT/2)),cv::Scalar(200,200,0));
    }
}

void unyounyo(){
    cv::Mat img(cv::Size(CANVAS_WIDTH,CANVAS_HEIGHT),CV_8UC3,cv::Scalar(0,0,0));

    double r,th;
    r=180;
    th=(double)counter*3/180;
    canvas=img;
    bbb(r*cos(th),r*sin(th),-r*cos(th),-r*sin(th));

    imshow("requid",img);
    img.cv::Mat::release();
    cv::waitKey(100);
    std::cout<<"new frame"<<endl;
    counter+=3;
    unyounyo();
}





int zoom=2;
double mypow(double x,int n){
    double y=1;
    for(int i=0;i<n;i++){
        y=y*x;
    }
    return y;
}

void mandelbrot(){
    cv::Mat img(cv::Size(CANVAS_WIDTH,CANVAS_HEIGHT),CV_8UC3,cv::Scalar(100,100,0));
    std::cout<<"0,0 color : "<<img.row(0).col(0)<<endl;
    int n,thrd;
    thrd=1000;
    double r,xc,yc,x1,y1,x2,y2;
    for(int i=0;i<CANVAS_HEIGHT;i++){
        for(int j=0;j<CANVAS_WIDTH;j++){
            n=0;
            xc=(double)((double)(j-CANVAS_WIDTH/2)/(double)(CANVAS_WIDTH/mypow(0.95,zoom))-0.1528);
            yc=(double)((double)(i-CANVAS_HEIGHT/2)/(double)(CANVAS_HEIGHT/mypow(0.95,zoom))+1.0397);
            x1=0;
            y1=0;
            do{
                x2=(double)(x1*x1-y1*y1+xc);
                y2=(double)(2*x1*y1+yc);
                r=sqrt(x2*x2+y2*y2);
                x1=x2;
                y1=y2;
                n++;
                
                if(n>=thrd){
                    //std::cout<<img.row(i).col(j)<<endl;
                    img.row(i).col(j)=cv::Scalar(0,0,0);
                    break;
                }
            }while(r<1000);
            
            if(n<thrd){
                int R,G,B;
                double smooth;
                smooth=(double)(n-log(n)/log(thrd));
                smooth=10.0*smooth;
                R=(int)(128+127*sin((double)((smooth+counter)/180)));
                G=(int)(128+127*sin((double)((smooth+counter)/180+2*PI/3)));
                B=(int)(128+127*sin((double)((smooth+counter)/180+4*PI/3)));
                
                img.row(i).col(j)=cv::Scalar(R,G,B);  
                //if(i==0&&j==0){
                  //  std::cout<<"th,R<G<B"<<th<<" "<<R<<" "<<G<<" "<<B<<endl;
                //}
            }
        }
    }
    std::cout<<"roop is over."<<endl;
    imshow("mandelbrot",img);
    //img.cv::Mat::release();
    cv::waitKey(160);
    zoom++;
    counter+=5;
    mandelbrot();
}

void julia(){
    cv::Mat img(cv::Size(CANVAS_WIDTH,CANVAS_HEIGHT),CV_8UC3,cv::Scalar(0,0,0));

    int thrd=1000;
    double x1,y1,x2,y2,r,xc,yc;
    xc=(double)(0.8*cos((double)((double)counter/180)));
    yc=(double)(0.8*sin((double)((double)counter/180)));

    for(int i=0;i<CANVAS_HEIGHT;i++){
        for(int j=0;j<CANVAS_WIDTH;j++){
            int n=0;
            x1=(double)((double)(j-CANVAS_WIDTH/2)/(double)(CANVAS_WIDTH/4));
            y1=(double)((double)(i-CANVAS_HEIGHT/2)/(double)(CANVAS_HEIGHT/4));
    
            do{
                x2=(double)(x1*x1-y1*y1+xc);
                y2=(double)(2*x1*y1+yc);
                r=sqrt(x2*x2+y2*y2);
                x1=x2;
                y1=y2;
                n++;
                if(n>thrd){
                    img.row(i).col(j)=cv::Scalar(0,0,0);
                    break;
                }
            }while(r<100);
	    
            if(n<thrd){
                int R,G,B;
                double smooth;
                smooth=(double)(n-log(n)/log(thrd));
                smooth=10.0*smooth;
                R=(int)(128+127*sin((double)((smooth+counter)/180)));
                G=(int)(128+127*sin((double)((smooth+counter)/180+2*PI/3)));
                B=(int)(128+127*sin((double)((smooth+counter)/180+4*PI/3)));
                
                img.row(i).col(j)=cv::Scalar(R,G,B);  
            }


        }
    }
    cv::imshow("julia",img);
    img.cv::Mat::release();
    cv::waitKey(170);
    counter+=4;
    julia();
}

void newton(){
    cv::Mat img(cv::Size(CANVAS_WIDTH,CANVAS_HEIGHT),CV_8UC3,cv::Scalar(0,0,0));
    int thrd=500;
    double x1,y1,x2,y2,diff,rto;
    rto=1.4-0.4*(double)(sin((double)counter/180));
    //rto=1.0;
    for(int i=0;i<CANVAS_HEIGHT;i++){
        for(int j=0;j<CANVAS_WIDTH;j++){
            int n=0;
            x1=(double)(j-CANVAS_WIDTH/2)/(CANVAS_WIDTH/2)+0.8;
            y1=(double)(i-CANVAS_HEIGHT/2)/(CANVAS_HEIGHT/2);
            do{
                double a,b,c,d;
                //f=x^3-1
                //a=x1*x1*x1-3*x1*y1*y1-1;
                //b=3*x1*x1*y1-y1*y1*y1;
                //c=3*x1*x1-3*y1*y1;
                //d=6*x1*y1;
               
                //x^3-2*x^2+2
                a=x1*x1*x1-3*x1*y1*y1-2*(x1*x1-y1*y1)+2;
                b=3*x1*x1*y1-y1*y1*y1-2*2*x1*y1;
                c=3*(x1*x1-y1*y1)-2;
                d=3*2*x1*y1;
                
                
                x2=x1+rto*((a*c+b*d)/(c*c+d*d));
                y2=y1+rto*((-a*d+b*c)/(c*c+d*d));
                
                diff=abs(x2-x1);
                n++;
                x1=x2;
                y1=y2;
                if(n>thrd){
                    img.row(i).col(j)=cv::Scalar(0,0,0);
                    break;
                }
            }while(diff>0.01);
            
            if(n<thrd){
                int R,G,B;
                double smooth;
                smooth=(double)((double)n-log(n)/log(thrd));
                smooth=(double)(50.0*smooth);
                R=(int)(128+127*sin((double)((smooth+counter)/180)));
                G=(int)(128+127*sin((double)((smooth+counter)/180+2*PI/3)));
                B=(int)(128+127*sin((double)((smooth+counter)/180+4*PI/3)));
                
                img.row(j).col(i)=cv::Scalar(R,G,B);  
            }



        }
    }
    cv::imshow("newton",img);
    img.cv::Mat::release();
    cv::waitKey(140);
    counter+=2;
    newton();
}

int main(){
    std::cout<<"Hello, world!"<<endl;
    newton();
    return 0;
}
