#include <iostream>
#include <cmath>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>

//------ Robot Configs
#define _cnf_rotate 4.0
#define _cnf_move 50.0
#define _s_size 0.3
#define _sharp_deg 120.0
#define _radian 0.0174533

//------Robot Spawn Point Configs
//-- 1/2 is Middle of Field's Width According to Scale
#define _cnf_spawn_x 3.5/10
//-- 4/11 is Middle of Field's Length According to Humanoid Laws and Scale
#define _cnf_spawn_y 4/11

using namespace std;
using namespace cv;

//-- Prototypes
double Radian(double);

class Field
{
    protected:
        Mat World;
        double _size;
        int _line_size;
    public:
        Field();
        Mat Accessor()
        {
            return World;
        }
        int _sizeAccess()
        {
            return _size;
        }
        int Set();
};

int Field::Set()
{
    World = Mat(1, 1, CV_8UC3, 0.0);
    resize(World, World, Size(_size, _size * 8/11), INTER_LINEAR);

    //-- Points 
    Point _top_left(0,0);
    Point _down_right(_size - 1, _size * 8/11 - 1);
    Point _corner_1(_size * 1/11, _size * 1/11);
    Point _corner_2(_size * 10/11, _size * 7/11);
    Point _mid_line_start(_size / 2, _size * 1/11);
    Point _mid_line_end(_size / 2, _size * 7/11);
    Point _mid_middle(_size / 2, _size * 4/11);
    Point _penalty_r1(_size * 1/11, _size * 1.5/11);
    Point _penalty_r2(_size * 3/11, _size * 6.5/11);
    Point _penalty_l1(_size * 10/11, _size * 6.5/11);
    Point _penalty_l2(_size * 8/11, _size * 1.5/11);
    Point _goal_area_r1(_size * 1/11, _size * 2.5/11);
    Point _goal_area_r2(_size * 2/11, _size * 5.5/11);
    Point _goal_area_l1(_size * 10/11, _size * 2.5/11);
    Point _goal_area_l2(_size * 9/11, _size * 5.5/11);
    Point _goal_r1(_size * 0.4/11, _size * 2.7/11);
    Point _goal_r2(_size * 1/11, _size * 5.3/11);
    Point _goal_l1(_size * 10/11, _size * 2.7/11);
    Point _goal_l2(_size * 10.6/11, _size * 5.3/11);

    //-- Lines and Shapes
    //- Corner
    rectangle(World, _top_left, _down_right, Scalar(100,100,100), _line_size, 8, 0);
    //- Field
    rectangle(World, _corner_1, _corner_2, Scalar(100,100,100), _line_size, 8, 0);
    //- Middle
    line(World, _mid_line_start, _mid_line_end, Scalar(100,100,100), _line_size, 8, 0);
    circle(World, _mid_middle, (_size * 1.5 / 22), Scalar(100,100,100), _line_size, 8, 0);
    //- Penalty Area
    rectangle(World, _penalty_r1, _penalty_r2, Scalar(100,100,100), _line_size, 8, 0);
    rectangle(World, _penalty_l1, _penalty_l2, Scalar(100,100,100), _line_size, 8, 0);
    //- Goal Area
    rectangle(World, _goal_area_r1, _goal_area_r2, Scalar(100,100,100), _line_size, 8, 0);
    rectangle(World, _goal_area_l1, _goal_area_l2, Scalar(100,100,100), _line_size, 8, 0);
    //- Goal
    rectangle(World, _goal_r1, _goal_r2, Scalar(100,100,100), _line_size, 8, 0);
    rectangle(World, _goal_l1, _goal_l2, Scalar(100,100,100), _line_size, 8, 0);
return 0;
}

Field::Field()
    {
        double size;
        cout << "\033[1;96mEnter Scale \033[0m:" << endl << "\033[1;96m - \033[0m"; cin >> size;
        _size = size;
        if (_size >= 2500)
        {
            _line_size = 10;
        }
        else 
        {
            _line_size = 4;
        }
        Set();
    }

class Robot : public Field
{
    private:
        //-- We Have Arrow Shape with 4 Points and Their Own Cordinates
        double _cnf_scale;
        double _cnf_size;
        double _x_O, _y_O;
        double _x_R, _y_R;
        double _x_L, _y_L;
        double _x_D, _y_D;
        double _bot_rotate;
        
    public:
        //-- Constructor to Set Spawn Position 
        Robot();
        
        //-- Funtion to Create Shape
        //---- Shape Draw The Position on Field
        int Shape();
        //---- Rotates The Shape
        int Rotate(double);
        //---- Move The Shape
        int Move(double);  
};

Robot::Robot()
{
    _cnf_scale = _sizeAccess();
    _cnf_size = _cnf_scale * _s_size / 11;
    _bot_rotate = 0;
    _x_O = _cnf_scale * _cnf_spawn_x;
    _y_O = _cnf_scale * _cnf_spawn_y;
}

int Robot::Shape()
{
    Mat Arrow;
    World.copyTo(Arrow);
    // set other 3 points, and call rotate and move function and draw shape
    //-- Point Center
    Point _center_pt_O(_x_O, _y_O);

    //-- Point Direction 
    _x_D = _x_O + _cnf_size * cos(Radian(_bot_rotate));
    _y_D = _y_O + _cnf_size * sin(Radian(_bot_rotate));
    Point _direction_pt_D(_x_D, _y_D);

    //-- Point Right
    _x_R = _x_O + _cnf_size * cos(Radian(_bot_rotate - _sharp_deg));
    _y_R = _y_O + _cnf_size * sin(Radian(_bot_rotate - _sharp_deg));
    Point _right_pt_R(_x_R, _y_R);

    //-- Point Left
    _x_L = _x_O + _cnf_size * cos(Radian(_bot_rotate + _sharp_deg));
    _y_L = _y_O + _cnf_size * sin(Radian(_bot_rotate + _sharp_deg));
    Point _left_pt_L(_x_L, _y_L);

    //-- DR
    line(Arrow, _direction_pt_D, _right_pt_R, Scalar(115,241,231), _line_size, 8, 0);
    //-- RO
    line(Arrow, _right_pt_R, _center_pt_O, Scalar(115,241,231), _line_size, 8, 0);
    //-- OL
    line(Arrow, _center_pt_O, _left_pt_L, Scalar(115,241,231), _line_size, 8, 0);
    //-- LD
    line(Arrow, _left_pt_L, _direction_pt_D, Scalar(115,241,231), _line_size, 8, 0);

    //-- Show Output
    imshow("Football Field", Arrow);
    waitKey(0);
    destroyAllWindows();
    return 0;
}

int Robot::Rotate(double r)
{
    _bot_rotate += r;
    Shape();
    return 0;
}

int Robot::Move(double m)
{
    _x_O = _x_O + m * cos(Radian(_bot_rotate));
    _y_O = _y_O + m * sin(Radian(_bot_rotate));
    Shape();
    return 0;
}

int main()
{
    //-- User View Size of Field
    Field _board;
    Robot _bot; 
    _bot.Shape();

    cout << " move : " << endl;
    double move,rot;
    cin >> move;
    _bot.Move(move);
    cout << " rotate" << endl;
    cin >> rot;
    _bot.Rotate(rot);
    return 0;
}

double Radian(double deg)
{
    deg *= _radian;
    return deg;
}