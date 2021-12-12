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

    void rotation(double rotZ,double rotY){
        double dx,dy,dz;
        dx=x*cos(rotZ)-y*sin(rotZ);
        dy=x*sin(rotZ)+y*cos(rotZ);
        x=dx;
        y=dy;
        
        dz=z*cos(rotY)-x*sin(rotY);
        dx=z*sin(rotY)+x*cos(rotY);
        z=dz;
        x=dx;
    }
};

class Controller : public Physics{
public:
    int ex;
    int ey;
    int ez;
    
    using Physics::Physics;
    //Controller(int ex,int ey,int ez)
    //:Physics(double x,double y,double z){}

    Controller(int xx,int yy,int zz){
        ex=xx;
        ey=yy;
        ez=zz;
    }
};

class Cell : public Physics{
public:
    Scalar color;
    string label;
    int vx;
    int vy;
    int vz;
    
    using Physics::Physics;
    //Cell(Scalar color,string label,int vx,int vy,int vz)
    //:Physics(double x,double y,double z){}

    Cell(Scalar cc,string ll,int xx,int yy,int zz){
        color=cc;
        label=ll;
        vx=xx;
        vy=yy;
        vz=zz;
    }
};


