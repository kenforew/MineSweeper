#include <iostream>
#include <string>
#include <math.h>
#include <random>
#include <vector>

#include <cstdlib>
#include <unistd.h>

#include <chrono>
#include <thread>

#include "3DMS.hpp"

#include <opencv2/opencv.hpp>

GameArea gameArea = GameArea();
Mouse mouse = Mouse();
GameManager gameManager = GameManager();

void controllerInitialize();
void cellInitialize();
void timeCounter();
int flagCounter();
void modeChange(int);
cv::Scalar cellColor(int);
void gameClearJudge();
void gameInitialize();
void dangerInitialize();
void safechain(Point3D);
void safechain_internal(Point3D);
bool is_Safe(Point3D);
bool is_Safe_internal(Point3D);
int count(Point3D);
int count_internal(Point3D);
void gameDisplay();

void controllerInitialize(){

    gameManager.controllerlist.clear();
    for(auto i = 0; i < 2; ++i){
        int index = 2 * i - 1;
        double l = 100.0 * index;
        gameManager.controllerlist.push_back(Controller(l, 0.0, 0.0, index, 0, 0));
        gameManager.controllerlist.push_back(Controller(0.0, l, 0.0, 0, index, 0));
        gameManager.controllerlist.push_back(Controller(0.0, 0.0, l, 0, 0, index));
    }

    for(auto i = 0; i < 6; ++i){
        Controller object = gameManager.controllerlist[i];

        auto R = 0, G = 0, B = 0;

        if(object.getArrow().x == 1) {
            R = 0xff;
        } else if (object.getArrow().x == -1) {
            R = 0x00;
        } else {
            R = 0x88;
        }
        
        if(object.getArrow().y == 1) {
            G = 0xff;
        } else if (object.getArrow().y == -1) {
            G = 0x00;
        } else {
            G = 0x88;
        }
        
        if(object.getArrow().z == 1) {
            B = 0xff;
        } else if (object.getArrow().z == -1) {
            B = 0x00;
        } else {
            B = 0x88;
        }

        gameManager.controllerlist[i].setColor(cv::Scalar(B, G, R));
    }
    
}

void cellInitialize() {
    
    gameManager.celllist.clear();
    for(auto i = 0; i < gameManager.getSize().x; ++i) {
        gameManager.celllist.push_back({});
        for(auto j = 0; j < gameManager.getSize().y; ++j) {
            gameManager.celllist[i].push_back({});
            for(auto k = 0; k < gameManager.getSize().z; ++k) {
                gameManager.celllist[i][j].push_back(Cell(
                    gameManager.getInterval() * (i - (double)(gameManager.getSize().x - 1) / 2),
                    gameManager.getInterval() * (j - (double)(gameManager.getSize().y - 1) / 2),
                    gameManager.getInterval() * (k - (double)(gameManager.getSize().z - 1) / 2),
                    i, j, k
                ));
            }
        }
    }

    dangerInitialize();

    for(auto i = 0; i < gameManager.getSize().x; ++i){
        for(auto j = 0; j < gameManager.getSize().y; ++j){
            for(auto k = 0; k < gameManager.getSize().z; ++k){
                int neighbors = count(Point3D(i, j, k));
                
                gameManager.celllist[i][j][k].setNeighbors(neighbors);
                gameManager.celllist[i][j][k].setColor(cellColor(neighbors));
                gameManager.celllist[i][j][k].setLabel(
                    gameManager.celllist[i][j][k].getDanger()
                    ? (std::string) "b"
                    : std::to_string(neighbors)
                );
            }
        }
    }


}

void timeCounter(){	
    while(true){
        while(mouse.getInitFlag()){
            sleep(1);
            while(mouse.getInitFlag()){
                gameManager.setTime(gameManager.getTime() + 1);
                sleep(1);
            }
        }
    }
    return;
}

int flagCounter(){
    int x = 0;
    for(auto i = 0; i < gameManager.getSize().x; ++i){
        for(auto j = 0; j < gameManager.getSize().y; ++j){
            for(auto k = 0; k < gameManager.getSize().z; ++k){
                if(gameManager.celllist[i][j][k].getFlag()
                && !gameManager.celllist[i][j][k].getDemined()){
                    x++;
                }
            }
        }
    }
    return x;
}

void modeChange(int m){
	switch(m) {
        case 0:
            gameManager.setSize(Point3D(6,6,6));
            gameManager.setMines(10);
            break;
        case 1:
            gameManager.setSize(Point3D(8,8,8));
            gameManager.setMines(40);
            break;
        case 2:
            gameManager.setSize(Point3D(10,10,10));
            gameManager.setMines(99);
            break;
        default:
            break;
    }

	/*
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
    */

    gameInitialize();
}

cv::Scalar cellColor(int x){
    uchar R = 0, G = 0, B = 0;

    if(x % 3 == 0) {
        R = 0x11;
    } else if (x % 3 == 1) {
        R = 0x55;
    } else if (x % 3 == 2) {
        R = 0x99;
    }
    
    float var_g = x / 2;
    int gg = (int) var_g;

    if(gg % 3 == 0) {
        G = 0x44;
    } else if (gg % 3 == 1) {
        G = 0x88;
    } else if (gg % 3 == 2) {
        G = 0xcc;
    }

    float var_h = x / 3;
    int hh = (int) var_h;

    if(hh % 3 == 0) {
        B = 0x77;
    } else if ( hh % 3 == 1) {
        B = 0xbb;
    } else if (hh % 3 == 2) {
        B = 0xff;
    }

    return cv::Scalar(B,G,R);
}

void gameClearJudge(){
    int counter=0;
    for(auto i = 0; i < gameManager.getSize().x; ++i) {
        for(auto j = 0; j < gameManager.getSize().y; ++j) {
            for(auto k = 0; k < gameManager.getSize().z; ++k){
                if(gameManager.celllist[i][j][k].getDemined()){
                    counter++;
                }
            }
        }
    }

    if(counter == (gameManager.volume() - gameManager.getMines())) {
        gameManager.setGameclear(true);
        gameManager.setGameover(true);
        mouse.setInitFlag(false);
    }

    if(gameManager.getGameclear()){
        std::cout << "game clear" << std::endl;
    }
}

void gameInitialize(){
    gameManager.setGameover(false);
    gameManager.setGameclear(false);
    gameManager.setTime(0);
    mouse.setInitFlag(false);

    mouse.setDownPos(Point2D(0, 0));
    mouse.setUpdatePos(Point2D(0, 0));
    mouse.setUpPos(Point2D(0, 0));

    gameManager.setCursor(Point3D(3,3,3));

    controllerInitialize();
    
    cellInitialize();   

    gameDisplay();
}

void dangerInitialize(){
    std::vector<int> mineIndex;
    bool newIntFlag = true;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> distrib(0, gameManager.volume() - 1);

    while(mineIndex.size() < gameManager.getMines()){
        int rand = distrib(gen);
        newIntFlag = true;
        for(auto i = 0; i < mineIndex.size(); ++i){
            if(mineIndex[i] == rand){
                newIntFlag = false;
                break;
            }
        }
        if(newIntFlag){
            mineIndex.push_back(rand);
        }
    }

    for(auto i = 0; i < gameManager.getMines(); ++i){
        int x = (mineIndex[i] / (gameManager.getSize().y * gameManager.getSize().z))
            % gameManager.getSize().x;
        int y = (mineIndex[i] / gameManager.getSize().z)
            % gameManager.getSize().y;
        int z = (mineIndex[i])
            % gameManager.getSize().z;
        gameManager.celllist[x][y][z].setDanger(true);
    }
}

void safechain(Point3D p) {
    for(auto i = 0; i < gameManager.neighborlist.size(); ++i) {
        safechain_internal(
            p + (gameManager.neighborlist[i]));
    }
}

void safechain_internal(Point3D p) {
    if(p.x >= 0 && p.x < gameManager.getSize().x
    && p.y >= 0 && p.y < gameManager.getSize().y
    && p.z >= 0 && p.z < gameManager.getSize().z
    &&!gameManager.celllist[p.x][p.y][p.z].getDemined()){
        gameManager.celllist[p.x][p.y][p.z].setDemined(true);
        if(is_Safe(p)) {
            safechain(p);
        } else {
            return;
        }
    } else {
        return;
    }
}

bool is_Safe(Point3D p) {
    bool result = true;

    for(auto i = 0; i<gameManager.neighborlist.size(); ++i) {
        result = result && is_Safe_internal(
            p + gameManager.neighborlist[i]);
    }

    return result;
}

bool is_Safe_internal(Point3D p) {
    if(p.x < 0 || p.x > gameManager.getSize().x - 1
    || p.y < 0 || p.y > gameManager.getSize().y - 1
    || p.z < 0 || p.z > gameManager.getSize().z - 1) {
        return true;
    }else{
        if(gameManager.celllist[p.x][p.y][p.z].getDanger()) {
            return false;
        } else {
            return true;
        }
    }
}

int count(Point3D p) {
    int sum = 0;

    for(auto i = 0; i < gameManager.neighborlist.size(); ++i) {
        sum += count_internal(
            p + gameManager.neighborlist[i]);
    }

    return sum;
}

int count_internal(Point3D p) {
    if(p.x < 0 || p.x > gameManager.getSize().x - 1
    || p.y < 0 || p.y > gameManager.getSize().y - 1
    || p.z < 0 || p.z > gameManager.getSize().z - 1) {
        return 0;
    } else {
        return gameManager.celllist[p.x][p.y][p.z].getDanger() ? 1 : 0;
    }
}

void mouse_callback(int event, int x, int y, int flags, void *userdata) {
    if(event == cv::EVENT_LBUTTONDOWN) {
        mouse.setDownPos(Point2D(x, y));
        mouse.setEscapePos(Point2D(x, y));
        mouse.setDownFlag(true);
        mouse.setLeftClickFlag(true);
        mouse.setLongPressFlag(false);
    }
    if(event == cv::EVENT_LBUTTONUP) {
        mouse.setDownFlag(false);
        if(mouse.getLongPressFlag()) return;
        uchar r, g, b;
        r = gameArea.getImg().at<uchar>(y, 3 * x + 2);
        g = gameArea.getImg().at<uchar>(y, 3 * x + 1);
        b = gameArea.getImg().at<uchar>(y, 3 * x);
        if(r == 0xff && g == 0xff && b == 0xff) {
            Point3D cursor = gameManager.getCursor();
            Cell object = gameManager.celllist[cursor.x][cursor.y][cursor.z];
            if(!object.getDemined() && !object.getFlag()) {
                gameManager.celllist[cursor.x][cursor.y][cursor.z].setDemined(true);
                if(object.getDanger()){//bomb
                    gameManager.setGameover(true);
                    mouse.setInitFlag(false);
                }else if(is_Safe(cursor)){// no mine around
                    safechain(cursor);
                }

                if(!gameManager.getGameover()){
                    mouse.setInitFlag(true);
                }
            }
        }else if(!(r == 0 && g == 0 && b == 0)) {
            if(r == 0xff) {
                gameManager.setCursor(gameManager.getCursor() + Point3D(1, 0, 0));
            }
            if(r == 0x00) {
                gameManager.setCursor(gameManager.getCursor() + Point3D(-1, 0, 0));
            }
            if(g == 0xff) {
                gameManager.setCursor(gameManager.getCursor() + Point3D(0, 1, 0));
            }
            if(g == 0x00) {
                gameManager.setCursor(gameManager.getCursor() + Point3D(0, -1, 0));
            }
            if(b==0xff){
                gameManager.setCursor(gameManager.getCursor() + Point3D(0, 0, 1));
            }
            if(b==0x00){
                gameManager.setCursor(gameManager.getCursor() + Point3D(0, 0, -1));
            }
        }
    }
    if(event==cv::EVENT_RBUTTONDOWN){
        mouse.setDownFlag(true);
        mouse.setLeftClickFlag(true);
        mouse.setLongPressFlag(false);
    }
    if(event==cv::EVENT_RBUTTONUP){
        mouse.setDownFlag(false);
        //if(!GV.getlongPressFlag()){
        
        if(mouse.getLongPressFlag()) return;
        
        //int cx,cy,cz;
        //cx=GV.getcursor(0);
        //cy=GV.getcursor(1);
        //cz=GV.getcursor(2);
        Point3D c = gameManager.getCursor();
        Cell object(gameManager.celllist[c.x][c.y][c.z]);
        if(!object.getDemined() && object.getFlag()) {
            gameManager.celllist[c.x][c.y][c.z].setFlag(false);
        }else if(!object.getDemined() && !object.getFlag()) {
            gameManager.celllist[c.x][c.y][c.z].setFlag(true);
        }

        //}
    	mouse.setLongPressFlag(false);
        if(gameManager.getGameover()){
            std::cout << ">>>game over<<<" << std::endl;
        }
    }
    if(event==cv::EVENT_MOUSEMOVE){
        mouse.setUpdatePos(Point2D(x, y));

        Point2D p = mouse.getDownPos();
        double ld = sqrt(p.x * p.x + p.y * p.y);
        if(ld > 10){
            mouse.setLongPressFlag(true);
        }
        if(mouse.getDownFlag() && mouse.getLongPressFlag()){
            double rotZ, rotY;
            Point2D rot = (mouse.getUpdatePos() - mouse.getEscapePos());
            for(auto i = 0; i < gameManager.getSize().x; ++i){
                for(auto j = 0; j < gameManager.getSize().y; ++j){
                    for(auto k = 0; k < gameManager.getSize().z; ++k){
                        gameManager.celllist[i][j][k].rotate(rot.x/300.0, rot.y/300.0);
                    }
                }
            }
            for(auto i = 0; i < 6; ++i){
                gameManager.controllerlist[i].rotate(rot.x/300.0, rot.y/300.0);
            }
        }
        mouse.setEscapePos(Point2D(x, y));
    }
}

void gameDisplay(){
    while(true){
        cv::Mat img(cv::Size(gameArea.getCanvas().x, gameArea.getCanvas().y),
                    CV_8UC3,
                    cv::Scalar(0,0,0)
        );

        cv::Scalar textcolor = cv::Scalar(0xfe, 0xfe, 0xfe);
        putText(img, "R:New Game", cv::Point(10, 40), 1, 1.2, textcolor);
        putText(img, "E:Easy", cv::Point(10, 70), 1, 1.2, textcolor);
        putText(img, "N:Normal", cv::Point(10, 100), 1, 1.2, textcolor);
        putText(img, "H:Hard", cv::Point(10, 130), 1, 1.2, textcolor);
        
        putText(img,
                "Flag : " + std::to_string(flagCounter()),
                cv::Point(10, 200), 1, 1.2, textcolor);
        putText(img,
                "Time : " + std::to_string(gameManager.getTime()),
                cv::Point(10,230),1,1.2,textcolor);

        std::vector<Cell> sortlist;
        for(auto i = 0; i < gameManager.getSize().x; ++i) {
            for(auto j = 0; j < gameManager.getSize().y; ++j) {
                for(auto k = 0; k < gameManager.getSize().z; ++k) {
                    sortlist.push_back(gameManager.celllist[i][j][k]);
                }
            }
        }

        int cube = gameManager.volume();
        for(auto i = 0; i < cube; ++i){
            for(auto j = i + 1; j < cube; ++j){
                if(sortlist[i].getPos().x > sortlist[j].getPos().x) {
                    Cell t = sortlist[i];
                    sortlist[i] = sortlist[j];
                    sortlist[j] = t;
                }
            }
        }


        for(auto i = 0; i < cube; ++i){
            double fy, fz, fr;
            int oy, oz, size;
            fy = sortlist[i].getPos().y;
            fz = sortlist[i].getPos().z;
            fr = 1 + sortlist[i].getPos().x / 300;
            oy = gameArea.getCanvas().x / 2;
            oz = gameArea.getCanvas().y / 4;
            size = gameManager.getCellsize() / 2;

            if(fy * fr + oy > size 
            && fy * fr + oy < gameArea.getCanvas().x - size
            && fz * fr + oz > size
            && fz * fr + oz < gameArea.getCanvas().y - size){
                int vx, vy, vz;
                cv::Scalar cellcolor;
                bool onCursor;

                vx = sortlist[i].getIndex().x;
                vy = sortlist[i].getIndex().y;
                vz = sortlist[i].getIndex().z;
                cellcolor = sortlist[i].getColor();
                Cell object = gameManager.celllist[vx][vy][vz];

                onCursor = (gameManager.getCursor() == Point3D(vx, vy, vz));
                if(onCursor) {
                    cellcolor = cv::Scalar(0xff, 0xff, 0xff);
                }else{
                    if(!object.getDemined()) {
                        uchar R = fr * 10 * 10 + 44;
                        uchar G = fr * 10 * 10;
                        uchar B = fr * 10 * 10 + 22;
                        cellcolor = cv::Scalar(B, G, R);
                    }
                }

                cv::String celltext;
                if(!object.getDemined() ){
                    if(object.getFlag()) {
                        celltext="f";
                    } else {
                        celltext="";
                    }
                } else {
                    celltext = object.getLabel();
                }

                if(!(object.getDemined() && (object.getNeighbors() == 0))
                    || onCursor) {
                    circle(img,
                            cv::Point(fy * fr + oy, fz * fr + oz),
                            size * fr, cellcolor, -1);
                    circle(img,
                            cv::Point(fy * fr + oy, fz * fr + oz),
                            size * fr, cv::Scalar(0xee, 0xee, 0xee), 1);
                    if(celltext != "0"){
                        putText(img, celltext,
                                cv::Point(fy * fr + oy, fz * fr + oz),
                                1, 0.8, cv::Scalar(20, 20, 20));
                    }
                    if(gameManager.getGameclear()) {
                        putText(img, "GAME CLEAR",
                                cv::Point(
                                    gameArea.getCanvas().x * 3 / 8,
                                    gameArea.getCanvas().y / 2
                                ),
                                1, 3.0, cv::Scalar(140, 170, 160), 2);
                    } else if(gameManager.getGameover()) {
                        putText(img, "GAME OVER",
                                cv::Point(
                                    gameArea.getCanvas().x * 3 / 8,
                                    gameArea.getCanvas().y / 2
                                ),
                                1, 3.0, cv::Scalar(70, 100, 90), 2);
                    }
                }
            }
        }

        //controller
        std::vector<Controller> ssortlist;
        for(auto i = 0; i < 6; ++i){
            ssortlist.push_back(gameManager.controllerlist[i]);
        }

        for(auto i = 0; i < 6; ++i){
            for(auto j = i + 1; j < 6; ++j){
                if(ssortlist[i].getPos().x > ssortlist[j].getPos().x){
                    Controller t = ssortlist[i];
                    ssortlist[i] = ssortlist[j];
                    ssortlist[j] = t;
                }
            }
        }

        for(auto i = 0; i < 6; ++i) {
            double cy, cz, cr, size;
            int oy, oz;

            Controller object = ssortlist[i];
            cy = object.getPos().y;
            cz = object.getPos().z;
            cr = 1 + object.getPos().x / 200;
            oy = gameArea.getCanvas().x / 2;
            oz = gameArea.getCanvas().y * 3 / 4;
            size = gameManager.getCellsize() / 2 * cr;

            circle(img,
                    cv::Point(cy * cr + oy, cz * cr + oz),
                    size,
                    object.getColor(),
                    -1
            );

            if(i == 2) {
                circle(img,
                        cv::Point(oy, oz),
                        2 * gameManager.getCellsize() / 2,
                        cv::Scalar(0xff, 0xff, 0xff),
                        -1
                );
            }
        }

        cv::imshow("3Dminesweeper", img);

        gameArea.setImg(img);

        cv::setMouseCallback("3Dminesweeper", mouse_callback, &img);

        img.cv::Mat::release();

        switch(cv::waitKey(33)) {
            case 101://press e ... easy
                modeChange(0);
                break;
            case 110://press n ... normal
                modeChange(1);
                break;
            case 104://press h ... hard
                modeChange(2);
                break;
            case 114://press r ... reset
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

int main(int argc,char** argv){
    start();
    return 0;
}