#ifndef TDMS_HPP
#define TDMS_HPP

#include <iostream>
#include <string>
#include <math.h>
#include <random>
#include <vector>

#include <cstdlib>
#include <unistd.h>

#include <chrono>
#include <thread>

#include <opencv2/opencv.hpp>


class Point2D {
	public:
        int x, y;
        
        Point2D() {x = 0, y = 0;}
        Point2D(int _x, int _y): x(_x), y(_y) {} 
        ~Point2D() {}
};

class Point3D{
    public:
        int x, y, z;
        
        Point3D() {x = 0, y = 0, z = 0;}
        Point3D(int _x, int _y, int _z): x( _x), y( _y), z(_z) {}
        ~Point3D() {}
};

class Pos3D{
    public:
        double x, y, z;

        Pos3D(double _x, double _y, double _z): x(_x), y(_y), z(_z) {}
        ~Pos3D() {}

        double getX() {return x;}
        double getY() {return y;}
        double getZ() {return z;}
};

class Object3D{
    protected:
        Pos3D pos;
    public:
        Object3D(double _x, double _y, double _z): pos(Pos3D(_x, _y, _z)) {}
        ~Object3D() {}

        Pos3D getPos(){
            return pos;
        }

        void setPos(Pos3D _pos) {
            pos = _pos;
        }

        void rotate(double rotZ, double rotY) {
            double dx, dy, dz;
            dx = pos.x * cos(rotZ) - pos.y * sin(rotZ);
            dy = pos.x * sin(rotZ) + pos.y * cos(rotZ);
            pos.x = dx;
            pos.y = dy;

            rotY = -rotY;

            dz = pos.z * cos(rotY) - pos.x * sin(rotY);
            dx = pos.z * sin(rotY) + pos.x * cos(rotY);
            pos.z = dz;
            pos.x = dx;
        }
};

class Controller : public Object3D {
    private:
        Point3D arrow;
        //int ex, ey, ez;
        cv::Scalar color;
    public:
        Controller(double _x, double _y, double _z,
                    int _ex, int _ey, int _ez)
        : Object3D(_x, _y, _z), arrow(Point3D(_ex, _ey, _ez)) {}
        ~Controller() {}

        Point3D getArrow() {return arrow;}

        cv::Scalar getColor() {return color;}

        void setArrow(Point3D _arrow) {arrow = _arrow;}

        void setColor(cv::Scalar _color) {color = _color;}
};

class Cell : public Object3D {
    private:
        cv::Scalar color;
        std::string label;
        Point3D index;
        int neighbors;
        bool danger, demined, flag;
	public:
        Cell(double _x, double _y, double _z,
            int _ix, int _iy, int _iz)
        :Object3D(_x, _y, _z)
        //index(Point3D(_ix, _iy, _iz),
        //neighbors(0),
        //danger(false),demined(false),flag(false) 
        {
            index = Point3D(_ix, _iy, _iz);
            neighbors = 0;
            danger = false, demined = false, flag = false;
        }
        ~Cell() {}

        Point3D getIndex() {return index;}
	
        cv::Scalar getColor() {return color;}

        std::string getLabel() {return label;}

        bool getDanger() {return danger;}
        bool getDemined() {return demined;}
        int getNeighbors() {return neighbors;}
        bool getFlag() {return flag;}
	
        void setColor(cv::Scalar _color) {color = _color;}
        void setLabel(std::string _label) {label = _label;}
	
        void setDanger(bool _danger) {danger = _danger;}
        void setDemined(bool _demined) {demined = _demined;}
        void setNeighbors(int _neighbors) {neighbors = _neighbors;}
        void setFlag(bool _flag) {flag = _flag;}
};

class GameArea {
    private:
        Point2D canvas;
        cv::Mat img;
    public:
        GameArea() {canvas = Point2D(800, 800);}
        //GameArea(): canvas(Point2D(800, 800)) {}
        ~GameArea() {}
        
        Point2D getCanvas() {return canvas;}
        cv::Mat getImg() {return img;}
    
        void setCanvas(Point2D _canvas) {canvas = _canvas;}
        void setImg(cv::Mat _img) {img = _img;}
};

class Mouse {
    private:
        Point2D downPos, escapePos, updatePos, upPos;
        bool downFlag, leftClickFlag, longPressFlag, initFlag;
    public:
        Mouse() {
            downPos = Point2D(0, 0),
            escapePos = Point2D(0, 0),
            updatePos = Point2D(0, 0),
            upPos = Point2D(0, 0);
            downFlag = false,
            leftClickFlag = false,
            longPressFlag = false,
            initFlag = false;
        }
        ~Mouse() {};
	
        Point2D getDownPos() {return downPos;}
        Point2D getEscapePos() {return escapePos;}
        Point2D getUpdatePos() {return updatePos;}
        Point2D getUpPos() {return upPos;}

        bool getDownFlag() {return downFlag;}
        bool getLeftClickFlag() {return leftClickFlag;}
        bool getLongPressFlag() {return longPressFlag;}
        bool getInitFlag() {return initFlag;}

	    void setDownPos(Point2D _downPos) {downPos = _downPos;}
        void setEscapePos(Point2D _escapePos) {escapePos = _escapePos;}
        void setUpdatePos(Point2D _updatePos) {updatePos = _updatePos;}
        void setUpPos(Point2D _upPos) {upPos = _upPos;}
        
        void setDownFlag(bool _downFlag) {downFlag = _downFlag;}
        void setLeftClickFlag(bool _leftClickFlag) {leftClickFlag = _leftClickFlag;}
        void setLongPressFlag(bool _longPressFlag) {longPressFlag = _longPressFlag;}
        void setInitFlag(bool _initFlag) {initFlag = _initFlag;}
};

class GameManager {
    private:
        Point3D size, cursor;
        int mines, interval, cellsize, time;
        bool gameover, gameclear;
        
        //std::default_random_engine generator;
        //std::random_device rd;
	public:
        GameManager() {
            size = Point3D(6, 6, 6), cursor = Point3D(3, 3, 3);
            mines = 10, interval = 40, cellsize = 36;
        }
        //GameManager(Point3D _size, Point3D _cursor,
        //            int _mines, int _interval, int _cellsize) {
        //    size = _size, cursor = _cursor;
        //    mines = _mines, interval = _interval, cellsize = _cellsize, time = 0;
        //    gameover = false, gameclear = false;
        //}
        ~GameManager() {}
	
        std::vector<Controller> controllerlist;
        std::vector<std::vector<std::vector<Cell>>> celllist;
        std::vector<Point3D> neighborlist = {
            Point3D(1, 1, 1),
            Point3D(1, 1, 0),
            Point3D(1, 1, -1),
            Point3D(1, 0, 1),
            Point3D(1, 0, 0),
            Point3D(1, 0, -1),
            Point3D(1, -1, 1),
            Point3D(1, -1, 0),
            Point3D(1, -1, -1),
            Point3D(0, 1, 1),
            Point3D(0, 1, 0),
            Point3D(0, 1, -1),
            Point3D(0, 0, 1),
            
            Point3D(0, 0, -1),
            Point3D(0, -1, 1),
            Point3D(0, -1, 0),
            Point3D(0, -1, -1),
            Point3D(-1, 1, 1),
            Point3D(-1, 1, 0),
            Point3D(-1, 1, -1),
            Point3D(-1, 0, 1),
            Point3D(-1, 0, 0),
            Point3D(-1, 0, -1),
            Point3D(-1, -1, 1),
            Point3D(-1, -1, 0),
            Point3D(-1, -1, -1),
        };

        Point3D getSize() {return size;}
        Point3D getCursor() {return cursor;}
        int getMines() {return mines;}
        int getInterval() {return interval;}
        int getCellsize() {return cellsize;}
        int getTime() {return time;}
        bool getGameover() {return gameover;}
        bool getGameclear() {return gameclear;}

        void setSize(Point3D _size) {size = _size;}
        void setCursor(Point3D _cursor) {
            cursor = _cursor;
            
            if(cursor.x < 0) cursor.x += size.x;
            if(cursor.x >= size.x) cursor.x -= size.x;
            if(cursor.y < 0) cursor.y += size.y;
            if(cursor.y >= size.y) cursor.y -= size.y;
            if(cursor.z < 0) cursor.z += size.z;
            if(cursor.z >= size.z) cursor.z -= size.z;
        }
        void setMines(int _mines) {mines = _mines;}
        void setInterval(int _interval) {interval = _interval;}
        void setCellsize(int _cellsize) {cellsize = _cellsize;}
        void setTime(int _time) {time = _time;}
        void setGameover(bool _gameover) {gameover = _gameover;}
        void setGameclear(bool _gameclear) {gameclear = _gameclear;}

        int volume(){
            return size.x * size.y * size.z;
        }

        //int random() {
        //    std::mt19937 gen(rd());
        //    std::uniform_int_distribution<int> distrib(0, volume() - 1);
        //    return distrib(gen);
        //}
};

Point2D operator+(const Point2D &p1, const Point2D &p2) {
    Point2D ans = Point2D(0, 0);
    ans.x = p1.x + p2.x;
    ans.y = p1.y + p2.y;
    return ans;
}

Point3D operator+(const Point3D &p1, const Point3D &p2) {
    Point3D ans = Point3D(0, 0, 0);
    ans.x = p1.x + p2.x;
    ans.y = p1.y + p2.y;
    ans.z = p1.z + p2.z;
    return ans;
}

Point2D operator-(const Point2D &p1, const Point2D &p2) {
    Point2D ans = Point2D(0, 0);
    ans.x = p1.x - p2.x;
    ans.y = p1.y - p2.y;
    return ans;
}

Point3D operator-(const Point3D &p1, const Point3D &p2) {
    Point3D ans = Point3D(0, 0, 0);
    ans.x = p1.x - p2.x;
    ans.y = p1.y - p2.y;
    ans.z = p1.z - p2.z;
    return ans;
}

bool operator==(const Point3D &p1, const Point3D &p2) {
    if(p1.x == p2.x
    && p1.y == p2.y
    && p1.z == p2.z) {
        return true;
    } else {
        return false;
    }
}

#endif