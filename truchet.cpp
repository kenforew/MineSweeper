#include <iostream>
#include <string>
#include <math.h>
#include <random>
#include <vector>
#include <algorithm>

#include <opencv2/opencv.hpp>
namespace truchet{
namespace{
    #define CANVAS_WIDTH 960 //16 tiles
    #define CANVAS_HEIGHT 960 //16 tiles
    #define CELL_SIZE 60 

    int cursorX=CANVAS_WIDTH/(4*CELL_SIZE);
    int cursorY=CANVAS_HEIGHT/(4*CELL_SIZE);

    int initzoom=-50;
    int zoom=initzoom;

    bool initFlag=true;
    bool onGame=false;

    std::vector<std::vector<bool>> truchet;
    std::vector<cv::Scalar> ctank={cv::Scalar(0,0,0),cv::Scalar(0xff,0xff,0xff)};
    std::vector<std::vector<int>> ltank={{0xff,0xff,0},{0xff,0,0xff},{0,0xff,0xff}};

    std::vector<std::vector<std::vector<short>>> liquid;
    std::vector<short> start={0,0xff,0xff};
    std::vector<short> goal={0xff,0,0xff};

    std::vector<int> startxyz={0,0,0};

    cv::Mat canvas;

    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0,1);
    std::uniform_int_distribution<int> distribution2(0,7);
};


void title();
void truchetInitialize();
void display();
int main();

void title(){
    cv::Mat img(cv::Size(CANVAS_WIDTH,CANVAS_HEIGHT),CV_8UC3,cv::Scalar(0,0,0));

    putText(img,"TRUCHET MAZE",cv::Point(CANVAS_WIDTH/2,CANVAS_HEIGHT/2),1,2.4,cv::Scalar(0xff,0xff,0xff),2);
    putText(img,"Press Any Key",cv::Point(CANVAS_WIDTH/2,CANVAS_HEIGHT/2+30),1,1.2,cv::Scalar(0xff,0xff,0xff));
    
    imshow("truchet",img);
    img.release();
    cv::waitKey(0);
}

void truchetInitialize(){
    truchet.clear();
    
    truchet.resize(CANVAS_HEIGHT/(2*CELL_SIZE));
    
    for(int i=0;i<CANVAS_HEIGHT/(2*CELL_SIZE);i++){
        truchet[i].resize(CANVAS_WIDTH/(2*CELL_SIZE));
    }

    for(int i=0;i<CANVAS_HEIGHT/(2*CELL_SIZE);i++){
        for(int j=0;j<CANVAS_WIDTH/(2*CELL_SIZE);j++){
            truchet[i][j]=distribution(generator)==1?true:false;
        }
    }

    startxyz[0]=0;
    startxyz[1]=0;
    startxyz[2]=0;
    
    startxyz[distribution(generator)]=distribution2(generator);
    startxyz[2]=distribution(generator);
}

void mouse_callback(int event,int x,int y,int flags,void *userdata)
{
    if(event==cv::EVENT_MOUSEMOVE){
        short r,g,b;
        r=canvas.at<uchar>(y,3*x);
        g=canvas.at<uchar>(y,3*x+1);
        b=canvas.at<uchar>(y,3*x+2);
        if(onGame&&r==0&&g==0&&b==0){
            //gameover
            initFlag=true;
            onGame=false;
            circle(canvas,cv::Point(x,y),10,cv::Scalar(182,138,202),2);
            imshow("truchet",canvas);
            cv::waitKey(1000);
            putText(canvas,"GAME OVER",cv::Point(CANVAS_WIDTH/2,CANVAS_HEIGHT/2),1,3.0,cv::Scalar(0,0,0xff),5);
            putText(canvas,"Press Any Key",cv::Point(CANVAS_WIDTH/2,CANVAS_HEIGHT/2+30),1,1.2,cv::Scalar(0x8f,0x8f,0x8f),1);
            imshow("truchet",canvas);
            cv::waitKey(0);
            canvas.release();
            zoom=initzoom;
            truchetInitialize();
            display();
            return;
        }else if(onGame&&r==goal[0]&&g==goal[1]&&b==goal[2]){
            //gameclear            
            initFlag=true;
            onGame=false;
            circle(canvas,cv::Point(x,y),10,cv::Scalar(182,138,202),2);
            imshow("truchet",canvas);
            cv::waitKey(1000);
            putText(canvas,"GAME CLEAR",cv::Point(CANVAS_WIDTH/2,CANVAS_HEIGHT/2),1,3.0,cv::Scalar(0xff,0,0),5);
            putText(canvas,"Press Any Key",cv::Point(CANVAS_WIDTH/2,CANVAS_HEIGHT/2+30),1,1.2,cv::Scalar(0x8f,0x8f,0x8f),1);
            imshow("truchet",canvas);
            cv::waitKey(0);
            canvas.release();
            zoom=initzoom;
            truchetInitialize();
            display();
            return;

        }else if(!onGame&&y>70&&r==start[0]&&g==start[1]&&b==start[2]){
            //gamestart
            onGame=true;
        }
    }
}

int counter=0;
void display(){
    for(int huga=0;true;){
    counter++;
    std::cout<<"frame "<<counter<<std::endl;
    cv::Mat img(cv::Size(CANVAS_WIDTH,CANVAS_HEIGHT),CV_8UC3,cv::Scalar(0,0,0));
    
    double pers=pow(1.005,zoom);

    int n=0;
    do{
        cv::rectangle(img,cv::Point(-CANVAS_WIDTH/pow(2,n+1)*pers+CANVAS_WIDTH/2,-CANVAS_HEIGHT/pow(2,n+1)*pers+CANVAS_HEIGHT/2),
                            cv::Point(CANVAS_WIDTH/pow(2,n+1)*pers+CANVAS_WIDTH/2,CANVAS_HEIGHT/pow(2,n+1)*pers+CANVAS_HEIGHT/2),ctank[n%2],-1);
        for(int i=0;i<CANVAS_HEIGHT/(2*CELL_SIZE);i++){
            for(int j=0;j<CANVAS_WIDTH/(2*CELL_SIZE);j++){
                //if(j<3||j>9||i<2||i>7){
                    if(initFlag||(cursorX==j&&cursorY==i)){
                        //cout<<"rect"<<i<<j<<endl;
                        cv::rectangle(img,cv::Point((2*j-CANVAS_WIDTH/(2*CELL_SIZE))*CELL_SIZE/pow(2,n)*pers+CANVAS_WIDTH/2,(2*i-CANVAS_HEIGHT/(2*CELL_SIZE))*CELL_SIZE/pow(2,n)*pers+CANVAS_HEIGHT/2),
                                        cv::Point((2*(j+1)-CANVAS_WIDTH/(2*CELL_SIZE))*CELL_SIZE/pow(2,n)*pers+CANVAS_WIDTH/2,(2*(i+1)-CANVAS_HEIGHT/(2*CELL_SIZE))*CELL_SIZE/pow(2,n)*pers+CANVAS_HEIGHT/2),ctank[n%2],-1);
                    }
                    if(truchet[i][j]){
                        cv::ellipse(img,cv::Point((2*j-CANVAS_WIDTH/(2*CELL_SIZE))*CELL_SIZE/pow(2,n)*pers+CANVAS_WIDTH/2,(2*i-CANVAS_HEIGHT/(2*CELL_SIZE))*CELL_SIZE/pow(2,n)*pers+CANVAS_HEIGHT/2),cv::Size(CELL_SIZE/pow(2,n)*pers,CELL_SIZE/pow(2,n)*pers),0,0,90,ctank[n%2+1],CELL_SIZE*2/3/pow(2,n)*pers);
                        cv::ellipse(img,cv::Point((2*(j+1)-CANVAS_WIDTH/(2*CELL_SIZE))*CELL_SIZE/pow(2,n)*pers+CANVAS_WIDTH/2,(2*(i+1)-CANVAS_HEIGHT/(2*CELL_SIZE))*CELL_SIZE/pow(2,n)*pers+CANVAS_HEIGHT/2),cv::Size(CELL_SIZE/pow(2,n)*pers,CELL_SIZE/pow(2,n)*pers),0,180,270,ctank[n%2+1],CELL_SIZE*2/3/pow(2,n)*pers);
                    }else{
                        cv::ellipse(img,cv::Point((2*(j+1)-CANVAS_WIDTH/(2*CELL_SIZE))*CELL_SIZE/pow(2,n)*pers+CANVAS_WIDTH/2,(2*i-CANVAS_HEIGHT/(2*CELL_SIZE))*CELL_SIZE/pow(2,n)*pers+CANVAS_HEIGHT/2),cv::Size(CELL_SIZE/pow(2,n)*pers,CELL_SIZE/pow(2,n)*pers),0,90,180,ctank[n%2+1],CELL_SIZE*2/3/pow(2,n)*pers);
                        cv::ellipse(img,cv::Point((2*j-CANVAS_WIDTH/(2*CELL_SIZE))*CELL_SIZE/pow(2,n)*pers+CANVAS_WIDTH/2,(2*(i+1)-CANVAS_HEIGHT/(2*CELL_SIZE))*CELL_SIZE/pow(2,n)*pers+CANVAS_HEIGHT/2),cv::Size(CELL_SIZE/pow(2,n)*pers,CELL_SIZE/pow(2,n)*pers),0,270,360,ctank[n%2+1],CELL_SIZE*2/3/pow(2,n)*pers);
                
                    }
                //}
            }
        }
        for(int i=0;i<CANVAS_HEIGHT/(4*CELL_SIZE)+1;i++){
            for(int j=0;j<CANVAS_WIDTH/(4*CELL_SIZE)+1;j++){
                cv::circle(img,cv::Point((i*2-CANVAS_WIDTH/(4*CELL_SIZE))*CELL_SIZE/pow(2,n)*pers+CANVAS_WIDTH/2,
                                        (j*2-CANVAS_HEIGHT/(4*CELL_SIZE))*CELL_SIZE/pow(2,n)*pers+CANVAS_HEIGHT/2),CELL_SIZE/3/pow(2,n)*pers,ctank[n%2+1],-1);
            }
        }
        
        circle(img,cv::Point((cursorX*2+1-CANVAS_WIDTH/(2*CELL_SIZE))*CELL_SIZE/pow(2,n)*pers+CANVAS_WIDTH/2,
                            (cursorY*2+1-CANVAS_HEIGHT/(2*CELL_SIZE))*CELL_SIZE/pow(2,n)*pers+CANVAS_HEIGHT/2),CELL_SIZE*0.8/pow(2,n)*pers,cv::Scalar(0xff,0xff,0x00),2);
        n++; 
    }while(n<4);
    
    if(!onGame){
        int i=0;
        int j=0;
        if(startxyz[0]==0){
            i=16*startxyz[2];j=1;
        }else if(startxyz[1]==0){
            i=1;j=16*startxyz[2];
        }
        cv::circle(img,cv::Point((2*startxyz[0]+i-CANVAS_WIDTH/(2*CELL_SIZE))*CELL_SIZE*pers+CANVAS_WIDTH/2,
                                (2*startxyz[1]+j-CANVAS_HEIGHT/(2*CELL_SIZE))*CELL_SIZE*pers+CANVAS_HEIGHT/2),
                        12,cv::Scalar(start[0],start[1],start[2]),-1);
    }

    if(onGame){
        cv::circle(img,cv::Point(CANVAS_WIDTH/2,CANVAS_HEIGHT/2),
                        12*pers,cv::Scalar(goal[0],goal[1],goal[2]),-1);
    
    }

    if(!onGame){
        putText(img,"Go from",cv::Point(50,50),1,1.2,cv::Scalar(0xff,0xff,0xff));
        cv::circle(img,cv::Point(150,50),12,cv::Scalar(start[0],start[1],start[2]),-1);
        putText(img,"to",cv::Point(170,50),1,1.2,cv::Scalar(0xff,0xff,0xff));
        cv::circle(img,cv::Point(210,50),12,cv::Scalar(goal[0],goal[1],goal[2]),-1);
        putText(img,"along with",cv::Point(230,50),1,1.2,cv::Scalar(0xff,0xff,0xff));
        cv::circle(img,cv::Point(350,50),12,cv::Scalar(0xff,0xff,0xff),-1);
        putText(img,"line",cv::Point(370,50),1,1.2,cv::Scalar(0xff,0xff,0xff));

        putText(img,"       <W>      ",cv::Point(CANVAS_WIDTH/2-20,CANVAS_HEIGHT-70),1,1.2,cv::Scalar(0xff,0xff,0xff));
        putText(img,"<A> <SPACE> <D>",cv::Point(CANVAS_WIDTH/2-20,CANVAS_HEIGHT-50),1,1.2,cv::Scalar(0xff,0xff,0xff));
        putText(img,"       <S>      ",cv::Point(CANVAS_WIDTH/2-20,CANVAS_HEIGHT-30),1,1.2,cv::Scalar(0xff,0xff,0xff));
    }
    initFlag=false;

    imshow("truchet",img);
    canvas=img;
    cv::setMouseCallback("truchet",mouse_callback);
    
    switch(cv::waitKey(60)){
        case 97://a
            cursorX--;
            if(cursorX<0)cursorX=cursorX+CANVAS_WIDTH/(2*CELL_SIZE);
            break;
        case 119://w
            cursorY--;
            if(cursorY<0)cursorY=cursorY+CANVAS_HEIGHT/(2*CELL_SIZE);
            break;
        case 100://d
            cursorX=(cursorX+1)%(CANVAS_WIDTH/(2*CELL_SIZE));
            break;
        case 115://s
            cursorY=(cursorY+1)%(CANVAS_HEIGHT/(2*CELL_SIZE));
            break;
        case 32://space
            truchet[cursorY][cursorX]=truchet[cursorY][cursorX]?false:true;
            break;
        case 112:
            //pause
            break;   
        default:
            break;
    }

    if(onGame){
        zoom++;
    }
    
    img.cv::Mat::release();
    canvas.release();

    //delete img;
    //delete canvas;
    
}
}

    void gamestart(){
        title();
        truchetInitialize();
        display();
    }
};

int main(){
    truchet::gamestart();
    return 0;
}