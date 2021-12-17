#include <iostream>
#include <string>
#include <math.h>
#include <random>
#include <vector>

#include <opencv2/opencv.hpp>

using namespace std;

#define CANVAS_WIDTH 800
#define CANVAS_HEIGHT 400

int mouseDownX=0;
int mouseDownY=0;
int mouseEscapeX=0;
int mouseEscapeY=0;
int mouseUpdateX=0;
int mouseUpdateY=0;
int mouseUpX=0;
int mouseUpY=0;
bool mouseDownFlag=false;
bool leftClickFlag=false;
bool longPressFlag=false;

bool gameover=false;
bool gameclear=false;

int boardX=6;
int boardY=6;
int boardZ=6;
int mines=10;

int interval=CANVAS_HEIGHT/10;
int cellsize=interval/2.2;

vector<vector<vector<bool>>> danger;
vector<vector<vector<bool>>> demined;
vector<vector<vector<int>>> visual;
vector<int> cursor={3,3,3};

cv::Mat canvas;

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

vector<Controller> controllerlist;
vector<Cell> mainlist;

int timeCounter();
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

int timeCounter(){
    int x=0;
    
    return x;
}
int flagCounter(){
    int x=0;
    for(int i=0;i<boardX;i++){
        for(int j=0;j<boardY;j++){
            for(int k=0;k<boardZ;k++){
                if(visual[i][j][k]==-1&&!demined[i][j][k]){
                    x+=1;
                }
            }
        }
    }
    return x;
}

void generateLists(){
    danger.clear();
    demined.clear();
    visual.clear();
    
    danger.resize(boardX);
    demined.resize(boardX);
    visual.resize(boardX);

    for(size_t i=0;i<boardX;i++){
        danger[i].resize(boardY);
        demined[i].resize(boardY);
        visual[i].resize(boardY);
    }

    for(size_t i=0;i<boardX;i++){
        for(size_t j=0;j<boardY;j++){
            danger[i][j].resize(boardZ);
            demined[i][j].resize(boardZ);
            visual[i][j].resize(boardZ);
        }
    }

    for(int i=0;i<boardX;i++){
        for(int j=0;j<boardY;j++){
            for(int k=0;k<boardZ;k++){
                danger[i][j][k]=false;
                demined[i][j][k]=false;
                visual[i][j][k]=0;
            }
        }
    }
}


void modeChange(int m){
    switch(m){
        case 0://easy
            boardX=6;
            boardY=6;
            boardZ=6;
            mines=10;
            break;
        case 1://normal
            boardX=8;
            boardY=8;
            boardZ=8;
            mines=40;
            break;
        case 2://hard
            boardX=10;
            boardY=10;
            boardZ=10;
            mines=99;
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
    for(int i=0;i<boardX;i++){
        for(int j=0;j<boardY;j++){
            for(int k=0;k<boardZ;k++){
                if(demined[i][j][k]){
                    counter++;
                }
            }
        }
    }

    if(counter==boardX*boardY*boardZ-mines){
        gameclear=true;
        gameover=true;
    }

    if(gameclear){
        std::cout<<"game clear"<<endl;
    }
}

void gameInitialize(){
    gameover=false;
    gameclear=false;

    mouseDownX=0;
    mouseDownY=0;
    mouseUpdateX=0;
    mouseUpdateY=0;
    mouseUpX=0;
    mouseUpY=0;

    cursor[0]=(int)boardX/2;
    cursor[1]=(int)boardY/2;
    cursor[2]=(int)boardZ/2;

    generateLists();
    dangerInitialize();

    controllerlist.clear();

    for(int i=0;i<2;i++){
        int var_i=2*i-1;
        double var_l=CANVAS_HEIGHT/4.0*var_i;
        controllerlist.push_back(Controller(var_l,0.0,0.0,var_i,0,0));
        controllerlist.push_back(Controller(0.0,var_l,0.0,0,var_i,0));
        controllerlist.push_back(Controller(0.0,0.0,var_l,0,0,var_i));
    }
    mainlist.clear();

    for(int i=0;i<boardX;i++){
        for(int j=0;j<boardY;j++){
            for(int k=0;k<boardZ;k++){
                double x,y,z;
                x=(2*i-(boardX-1))*interval/2.0;
                y=(2*j-(boardY-1))*interval/2.0;
                z=(2*k-(boardZ-1))*interval/2.0;
                mainlist.push_back(Cell(x,y,z,cv::Scalar(150,150,150),"h",i,j,k));
            }
        }
    }

    gameDisplay();
}

void boardInitialize(){
    for(int i=0;i<boardX;i++){
        for(int j=0;j<boardY;j++){
            for(int k=0;k<boardZ;k++){
                danger[i][j][k]=false;
                demined[i][j][k]=false;
                visual[i][j][k]=0;
            }
        }
    }
}

std::default_random_engine generator;
std::uniform_int_distribution<int> distribution(0,boardX*boardY*boardZ-1);

void dangerInitialize(){
    vector<int> mineIndex;
    bool newIntFlag=true;
            
    while(mineIndex.size()<mines){
        int rand=distribution(generator);
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

    for(int i=0;i<mines;i++){
        int x=(mineIndex[i]/(boardY*boardZ))%boardX;
        int y=(mineIndex[i]/boardZ)%boardY;
        int z=mineIndex[i]%boardZ;
        danger[x][y][z]=true;
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
    if(x>=0&&x<boardX
    &&y>=0&&y<boardY
    &&z>=0&&z<boardZ
    &&x+dx>=0&&x+dx<boardX
    &&y+dy>=0&&y+dy<boardY
    &&z+dz>=0&&z+dz<boardZ
    &&!demined[x+dx][y+dy][z+dz]){
        if(safe(x+dx,y+dy,z+dz)){
            visual[x+dx][y+dy][z+dz]=0;
            demined[x+dx][y+dy][z+dz]=true;
            mainlist[(x+dx)*boardY*boardZ+(y+dy)*boardZ+(z+dz)].setcolor(cellColor(visual[x+dx][y+dy][z+dz]));
            mainlist[(x+dx)*boardY*boardZ+(y+dy)*boardZ+(z+dz)].setlabel(to_string(visual[x+dx][y+dy][z+dz]));
            safechain(x+dx,y+dy,z+dz);
        }else{
            visual[x+dx][y+dy][z+dz]=count(x+dx,y+dy,z+dz);
            demined[x+dx][y+dy][z+dz]=true;
            mainlist[(x+dx)*boardY*boardZ+(y+dy)*boardZ+(z+dz)].setcolor(cellColor(visual[x+dx][y+dy][z+dz]));
            mainlist[(x+dx)*boardY*boardZ+(y+dy)*boardZ+(z+dz)].setlabel(to_string(visual[x+dx][y+dy][z+dz]));
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
    if(x+dx<0||x+dx>boardX-1
    ||y+dy<0||y+dy>boardY-1
    ||z+dz<0||z+dz>boardZ-1){
        return true;
    }else{
        if(danger[x+dx][y+dy][z+dz]){
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
    if(x+dx<0||x+dx>boardX-1
    ||y+dy<0||y+dy>boardY-1
    ||z+dz<0||z+dz>boardZ-1){
        return 0;
    }else{
        return danger[x+dx][y+dy][z+dz]?1:0;
    }
}


void mouse_callback(int event,int x,int y,int flags,void *userdata)
{
    if(event==cv::EVENT_LBUTTONDOWN){
        mouseDownX=x;
        mouseDownY=y;
        mouseEscapeX=x;
        mouseEscapeY=y;
        mouseDownFlag=true;
        leftClickFlag=true;
        longPressFlag=false;
    }
    if(event==cv::EVENT_LBUTTONUP){
        mouseDownFlag=false;
        if(!longPressFlag){
            short r,g,b;
            r=canvas.at<uchar>(y,3*x);
            g=canvas.at<uchar>(y,3*x+1);
            b=canvas.at<uchar>(y,3*x+2);
           if(r==0xff&&g==0xff&&b==0xff){
                int cx,cy,cz;
                cx=cursor[0];
                cy=cursor[1];
                cz=cursor[2];
                if(visual[cx][cy][cz]!=-1){
                    demined[cx][cy][cz]=true;
                    if(danger[cx][cy][cz]){//bomb
                        mainlist[cx*boardY*boardZ+cy*boardZ+cz].setlabel("b");
                        gameover=true;
                    }else if(safe(cx,cy,cz)){//no mine around
                        visual[cx][cy][cz]=0;
                        mainlist[cx*boardY*boardZ+cy*boardZ+cz].setlabel("");
                        mainlist[cx*boardY*boardZ+cy*boardZ+cz].setcolor(cellColor(visual[cx][cy][cz]));
                        safechain(cx,cy,cz);
                    }else{//some mine around
                        visual[cx][cy][cz]=count(cx,cy,cz);
                        mainlist[cx*boardY*boardZ+cy*boardZ+cz].setlabel(to_string(visual[cx][cy][cz]));
                        mainlist[cx*boardY*boardZ+cy*boardZ+cz].setcolor(cellColor(visual[cx][cy][cz]));
                    }
                }
            }else{
                if(r==0xff){
                    cursor[0]=(cursor[0]+1)%boardX;
                }
                if(r==0x00){
                    cursor[0]=(cursor[0]-1)%boardX;
                    while(cursor[0]<0)cursor[0]=cursor[0]+boardX;
                }
                if(g==0xff){
                    cursor[1]=(cursor[1]+1)%boardY;
                }
                if(g==0x00){
                    cursor[1]=(cursor[1]-1)%boardY;
                    while(cursor[1]<0)cursor[1]=cursor[1]+boardY;
                }
                if(b==0xff){
                    cursor[2]=(cursor[2]+1)%boardZ;
                }
                if(b==0x00){
                    cursor[2]=(cursor[2]-1)%boardZ;
                    while(cursor[2]<0)cursor[2]=cursor[2]+boardZ;
                }
            } 
        }
    }
    if(event==cv::EVENT_RBUTTONDOWN){
        mouseDownFlag=true;
        leftClickFlag=true;
        longPressFlag=false;
    }
    if(event==cv::EVENT_RBUTTONUP){
        mouseDownFlag=false;
        if(!longPressFlag){
            int cx,cy,cz;
            cx=cursor[0];
            cy=cursor[1];
            cz=cursor[2];
            if(visual[cx][cy][cz]==-1){
                visual[cx][cy][cz]=0;
                mainlist[cx*boardY*boardZ+cy*boardZ+cz].setlabel("");
            }else if(visual[cx][cy][cz]==0&&!demined[cx][cy][cz]){
                visual[cx][cy][cz]=-1;
                mainlist[cx*boardY*boardZ+cy*boardZ+cz].setlabel("f");
            }
        }
        longPressFlag=false;
        if(!gameover){
            std::cout<<">>>game over<<<"<<endl;   
        }
    }
    if(event==cv::EVENT_MOUSEMOVE){
        mouseUpdateX=x;
        mouseUpdateY=y;
        
        double xd,yd,ld;
        xd=x-mouseDownX;
        yd=y-mouseDownY;
        ld=sqrt(xd*xd+yd*yd);
        if(ld>10){
            longPressFlag=true;
        }
        if(mouseDownFlag&&longPressFlag){
            double rotZ,rotY;
            rotZ=(mouseUpdateX-mouseEscapeX)/300.0;
            rotY=(mouseUpdateY-mouseEscapeY)/300.0;
            for(int i=0;i<boardX*boardY*boardZ;i++){
                mainlist[i].rotation(rotZ,rotY);
            }
            for(int i=0;i<6;i++){
                controllerlist[i].rotation(rotZ,rotY);
            }
        }
        mouseEscapeX=x;
        mouseEscapeY=y;
    }
}

void gameDisplay()
{
    for(int gumi=0;true;){
    
    cv::Mat img(cv::Size(CANVAS_WIDTH,2*CANVAS_HEIGHT),CV_8UC3,cv::Scalar(0,0,0));
    
    putText(img,"R:New Game",cv::Point(10,40),1,1.2,cv::Scalar(0xfe,0xfe,0xfe));
    putText(img,"E:Easy ",cv::Point(10,70),1,1.2,cv::Scalar(0xfe,0xfe,0xfe));
    putText(img,"N:Normal",cv::Point(10,100),1,1.2,cv::Scalar(0xfe,0xfe,0xfe)); 
    putText(img,"H:Hard",cv::Point(10,130),1,1.2,cv::Scalar(0xfe,0xfe,0xfe));
    putText(img,"Flag : "+to_string(flagCounter()),cv::Point(10,200),1,1.2,cv::Scalar(0xfe,0xfe,0xfe));
    putText(img,"Time : "+to_string(timeCounter()),cv::Point(10,230),1,1.2,cv::Scalar(0xfe,0xfe,0xfe));
    
    vector<Cell> sortlist;
    copy(mainlist.begin(),mainlist.end(),back_inserter(sortlist));

    for(int i=0;i<boardX*boardY*boardZ;i++){
        for(int j=i+1;j<boardX*boardY*boardZ;j++){
            if(sortlist[i].getx()>sortlist[j].getx()){
                Cell t=sortlist[i];
                sortlist[i]=sortlist[j];
                sortlist[j]=t;
            }
        }
    }

    for(int i=0;i<boardX*boardY*boardZ;i++){
	    double fy,fz,fr;
        int oy,oz,size;
        fy=sortlist[i].gety();
        fz=sortlist[i].getz();
        fr=1+sortlist[i].getx()/300;
        oy=CANVAS_WIDTH/2;
        oz=CANVAS_HEIGHT/2;
        size=cellsize;
        if(fr>0.0&&fr<2.0
        &&fy*fr+oy>size&&fy*fr+oy<CANVAS_WIDTH-size
        &&fz*fr+oz>size&&fz*fr+oz<CANVAS_HEIGHT-size){
            int vx,vy,vz;
            cv::Scalar cellcolor;
            bool onCursor;

            vx=sortlist[i].getvx();
            vy=sortlist[i].getvy();
            vz=sortlist[i].getvz();
            cellcolor=sortlist[i].getcolor();
            
            onCursor=(cursor[0]==vx)&&(cursor[1]==vy)&&(cursor[2]==vz);

            if(onCursor){
                cellcolor=cv::Scalar(0xff,0xff,0xff);
            }else{
                if(!demined[vx][vy][vz]){
                    int R=fr*10*10+44;
                    int G=fr*10*10;
                    int B=fr*10*10+22;

                    cellcolor=cv::Scalar(B,G,R);
                }
            }

            cv::String celltext;
            if(!demined[vx][vy][vz]){
                if(visual[vx][vy][vz]==-1){
                    celltext="f";
                }else{
                    celltext="";
                }
            }else if(demined[vx][vy][vz]){
                if(visual[vx][vy][vz]==0){
                    celltext="";
                }else{
                    celltext=to_string(visual[vx][vy][vz]);
                }
            }else{
                celltext="error";
            }

            if(!(demined[vx][vy][vz]&&visual[vx][vy][vz]==0)||onCursor){
                
                circle(img,cv::Point(fy*fr+oy,fz*fr+oz),size*fr,cellcolor,-1);            
                circle(img,cv::Point(fy*fr+oy,fz*fr+oz),size*fr,cv::Scalar(0xff,0xff,0xff),1);
                
                putText(img,celltext,cv::Point(fy*fr+oy,fz*fr+oz),1,0.8,cv::Scalar(20,20,20));
                
                if(gameclear){
                    putText(img,"GAME CLEAR",cv::Point(CANVAS_WIDTH/2,CANVAS_HEIGHT/2),1,3.0,cv::Scalar(100,100,100));
                }else if(gameover){
                    putText(img,"GAME OVER",cv::Point(CANVAS_WIDTH/2,CANVAS_HEIGHT/2),1,3.0,cv::Scalar(100,100,100));
                }
            }
        }
    }

    for(int i=0;i<6;i++){
        for(int j=i+1;j<6;j++){
            if(controllerlist[i].getx()>controllerlist[j].getx()){
               Controller t=controllerlist[i];
               controllerlist[i]=controllerlist[j];
               controllerlist[j]=t;
            }
        }
    }

    for(int i=0;i<6;i++){
        double cy,cz,cr,size;
        int oy,oz,R,G,B;

        cy=controllerlist[i].gety();
        cz=controllerlist[i].getz();
        cr=1+controllerlist[i].getx()/200;
        oy=CANVAS_WIDTH/2;
        oz=CANVAS_HEIGHT/2+CANVAS_HEIGHT;
        size=cellsize*cr;

        if(controllerlist[i].getex()==1){
            R=0xff;
        }else if(controllerlist[i].getex()==-1){
            R=0x00;
        }else{
            R=0x88;
        }

        if(controllerlist[i].getey()==1){
            G=0xff;
        }else if(controllerlist[i].getey()==-1){
            G=0x00;
        }else{
            G=0x88;
        }

        if(controllerlist[i].getez()==1){
            B=0xff;
        }else if(controllerlist[i].getez()==-1){
            B=0x00;
        }else{
            B=0x88;
        }

        circle(img,cv::Point(cy*cr+oy,cz*cr+oz),size,cv::Scalar(R,G,B),-1);
        
        if(i==2){
            circle(img,cv::Point(oy,oz),2*cellsize,cv::Scalar(255,255,255),-1);
        }
    }

    imshow("3Dminesweeper", img);    
    
    canvas=img;
    cv::setMouseCallback("3Dminesweeper",mouse_callback,&img);
    
    img.cv::Mat::release();

    switch(cv::waitKey(100)){
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

int main( int argc, char** argv )
{
    gameInitialize();
    return 0;
}
