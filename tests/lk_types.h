#pragma once
#ifndef __lk_types_h__
#define __lk_types_h__
#include <iostream>
#include <iomanip>
#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include <string.h>
#include <queue>

namespace lk{

template<typename T> class Point_;

template<typename T> class Point3_{
public:
  Point3_():x(0),y(0),z(0){}
  Point3_(T _x, T _y, T _z):x(_x),y(_y),z(_z){}
  Point3_(Point_<T>_p, T _z=0):x(_p.x),y(_p.y),z(_z){}
  ~Point3_(){}

  operator Point3_<float>() const {
    return Point3_<float>((float)x,(float)y,(float)z);
  }
  operator Point3_<double>() const {
    return Point3_<double>((double)x,(double)y,(double)z);
  }

  inline bool operator==(Point3_<T> p){
    if(p.x==x && p.y==y && p.z==z) return true;
    return false;
  }
  inline bool operator!=(Point3_<T> &p){
    if(p.x!=x || p.y!=y || p.z!=z) return true;
    return false;
  }
  inline bool operator<(const Point3_<T> &k2) const{
    return x < k2.x
        || (x == k2.x && y < k2.y)
        || (x == k2.x && y == k2.y && z < k2.z);
  }
  inline Point3_<T> operator-(Point3_<T> p){
    return Point3_<T>(x-p.x,y-p.y,z-p.z);
  }
  inline Point3_<T> operator-(){
    return Point3_<T>(-x,-y,-z);
  }
  inline Point3_<T> operator*(double a){
    return Point3_<T>(a*x,a*y,a*z);
  }
  inline void operator-=(Point3_<T> p){
    x-=p.x;
    y-=p.y;
    z-=p.z;
  }
  inline void operator+=(Point3_<T> p){
    x+=p.x;
    y+=p.y;
    z+=p.z;
  }
  inline Point3_<T> operator+(Point3_<T> p){
    return Point3_<T>(x+p.x,y+p.y,z+p.z);
  }
  inline T l2(){
    return x*x+y*y+z*z;
  }
  inline bool isnan(){
    return (std::isnan(x)||std::isnan(y)||std::isnan(z)||
            std::isinf(x)||std::isinf(y)||std::isinf(z));
  }
  friend std::ostream& operator<<(std::ostream& os, const Point3_<T> &p){
    os  << "[x:" << std::fixed << std::setprecision(4) <<p.x
        << " y:" << std::fixed << std::setprecision(4) << p.y
        << " z(du):" << std::fixed << std::setprecision(4)
        << (p.z/3.1415926)*180 <<"]";
    return os;
  }

  T x=0;
  T y=0;
  T z=0;

  typedef T value_type;
};

template<typename T> class Point_{
public:
  Point_():x(0),y(0){}
  Point_(T _x, T _y):x(_x),y(_y){}
  Point_(Point3_<T> _p):x(_p.x),y(_p.y){}
  ~Point_(){}

#define FUNC_WITH_ALL_TYPE(FUNC)\
  FUNC(double)\
  FUNC(float)\
  FUNC(int)\
  FUNC(int8_t)\
  FUNC(uint8_t)\

#define FUNC_SET(TYPE)\
  inline void set(TYPE _x,TYPE _y){x=_x;y=_y;}\
  inline void operator=(Point_<TYPE> p){x=p.x;y=p.y;}\

#define FUNC_CONVERT_TO(TYPE)\
  inline Point_<TYPE> to_##TYPE() const{return Point_<TYPE>(x,y);}\

#define OPERATOR_WITH_TYPE(OP,TYPE)\
  inline Point_<T> operator OP (Point_<TYPE> p){return Point_<T>(x OP p.x,y OP p.y);}\
  inline Point_<T> operator OP (TYPE d){return Point_<T>(x OP d,y OP d);}\

#define FUNC_OPERATOR_ADD(TYPE) OPERATOR_WITH_TYPE(+,TYPE)
#define FUNC_OPERATOR_SUB(TYPE) OPERATOR_WITH_TYPE(-,TYPE)
#define FUNC_OPERATOR_MUL(TYPE) OPERATOR_WITH_TYPE(*,TYPE)
#define FUNC_OPERATOR_DIV(TYPE) OPERATOR_WITH_TYPE(/,TYPE)

  FUNC_WITH_ALL_TYPE(FUNC_SET)
  FUNC_WITH_ALL_TYPE(FUNC_CONVERT_TO)
  FUNC_WITH_ALL_TYPE(FUNC_OPERATOR_ADD)
  FUNC_WITH_ALL_TYPE(FUNC_OPERATOR_SUB)
  FUNC_WITH_ALL_TYPE(FUNC_OPERATOR_MUL)
  FUNC_WITH_ALL_TYPE(FUNC_OPERATOR_DIV)

  inline bool operator==(Point_<T> &p){
    return (p.x==x && p.y==y);
  }
  inline bool operator!=(Point_<T> &p){
    return (p.x!=x || p.y!=y);
  }
  inline bool operator<(const Point_<T> &k2) const{
    return x < k2.x || (x == k2.x && y < k2.y);
  }
  inline Point_<T> operator^(double d){
    return Point_<T>(pow(x,d),pow(y,d));
  }

  operator Point_<float>() const {
    return Point_<float>((float)x,(float)y);
  }
  operator Point_<double>() const {
    return Point_<double>((double)x,(double)y);
  }

  inline T l2(){
    return x*x+y*y;
  }
  inline T l(){
    return sqrt(x*x+y*y);
  }
  inline double distance(Point_<T> p){
    return hypot(x-p.x,y-p.y);
  }
  inline double direction(Point_<T> p){
    return atan2(y-p.y,x-p.x);
  }

  friend std::ostream& operator<<(std::ostream& os, const Point_<T> &p){
    os << "[x:"<<p.x << " y:" << p.y<< "]";
    return os;
  }

  T x=0;
  T y=0;

  typedef T value_type;
};

typedef Point_<int> Point;
typedef Point_<int> Point2i;
typedef Point_<int8_t> Point2i8;
typedef Point_<float> Point2f;
typedef Point_<double> Point2d;
typedef Point3_<int> Point3i;
typedef Point3_<int> P3I;
typedef Point3_<float> Point3f;
typedef Point3_<double> Point3d;
typedef Point3d P3D;
typedef std::map<lk::Point, int> Point2IKeyMap;
typedef std::map<lk::Point, int> P2IKM;
typedef std::map<lk::Point2d, int> Point2DKeyMap;
typedef std::map<lk::Point2d, int> P2DKM;
typedef std::vector<lk::Point> Point2iList;
typedef std::vector<lk::Point3i> Point3iList;
typedef std::vector<lk::Point2d> P2DL;
typedef std::vector<lk::Point2f> P2FL;
typedef std::vector<lk::Point2i> P2IL;
typedef std::vector<lk::Point3i> P3IL;
typedef std::vector<lk::Point3d> P3DL;
typedef std::vector<lk::Point3f> P3FL;
typedef std::vector<Point2iList> LookupM;
typedef std::vector<P2FL> LookupW;

template<typename T>
Point3_<T> P2To3(Point_<T> p){
  return Point3_<T>(p.x,p.y,0);
}

template<typename _Tp> class PointPair_{
public:
  PointPair_(){}
  PointPair_(_Tp x1,_Tp y1,_Tp x2,_Tp y2):p1(Point_<_Tp>(x1,y1)),p2(Point_<_Tp>(x2,y2)){}
  PointPair_(Point_<_Tp> _p1, Point_<_Tp> _p2):p1(_p1),p2(_p2){}
  Point_<_Tp> p1; // point in map, F point
  Point_<_Tp> p2; // point in scan, G point
  typedef _Tp value_type;
};
typedef PointPair_<int> P2IP;
typedef PointPair_<double> P2DP;
typedef std::vector<P2IP> P2IPList;
typedef std::vector<P2DP> P2DPList;

template<typename _Tp> class CentreRange_{
public:
  CentreRange_(){}
  CentreRange_(Point3_<_Tp> _c, Point3_<_Tp> _r):c(_c),r(_r){}
  Point3_<_Tp> c; // point in map, F point
  Point3_<_Tp> r; // point in scan, G point
};
typedef CentreRange_<double> CR;

template<typename _Tp> class TimePoint_{
public:
  TimePoint_(){}
  TimePoint_(int _t,Point_<_Tp> _p):time(_t),point(_p){}
  TimePoint_(Point_<_Tp> _p):time(1),point(_p){}
  inline bool operator<(const TimePoint_<_Tp> &tp) const{
    return time<tp.time;
  }
  inline bool operator>(const TimePoint_<_Tp> &tp) const{
    return time>tp.time;
  }
  inline TimePoint_<double> toDouble()const{
    return TimePoint_<double>(time,point.toDouble());
  }
  int time;
  Point_<_Tp> point;
};
typedef TimePoint_<double> TimePoint2D;
typedef TimePoint_<double> TP2D;
typedef TimePoint_<int> TP2I;
typedef std::vector<TP2D> TimePoint2DList;
typedef std::vector<TP2D> TP2DL;
typedef std::priority_queue<TP2D> TimePoint2DPriorityQueue;
typedef std::priority_queue<TP2D> TP2DPQ;
typedef std::priority_queue<TP2I> TP2IPQ;

template<typename _Tp> class TimePointPair_{
public:
  TimePointPair_(){}
  TimePointPair_(Point_<_Tp> p1_,Point_<_Tp> p2_):p1(p1_),p2(p2_),time(1){}
  TimePointPair_(Point_<_Tp> p1_,Point_<_Tp> p2_,int t_):p1(p1_),p2(p2_),time(t_){}
  TimePointPair_(PointPair_<_Tp> pp_):p1(pp_.p1),p2(pp_.p2),time(1){}
  int time;
  Point_<_Tp> p1;
  Point_<_Tp> p2;
};
typedef TimePointPair_<double> TP2DP;
typedef std::vector<TP2DP> TimePoint2DPairList;
typedef std::vector<TP2DP> TP2DPL;

class TimePose{
public:
  TimePose():time(0),label(0){}
  TimePose(int64_t _time, Point3d _pose):
    time(_time),pose(_pose),label(0){}
  TimePose(int64_t _time, Point3d _pose, int _label):
    time(_time),pose(_pose),label(_label){}
  TimePose(Point3d _pose):time(1),pose(_pose),label(0){}
  inline bool operator<(const TimePose &tp) const{
    return time<tp.time;
  }
  inline bool operator>(const TimePose &tp) const{
    return time>tp.time;
  }
  friend std::ostream& operator<<(std::ostream& os, const TimePose &p){
    os << "[time:"<<p.time
       << " pose:" << p.pose
       << " label:" << p.label
       <<"]";
    return os;
  }
  int64_t time=0;
  Point3d pose;
  int label=0;
  // =1 : pose re able by outside
  // =0 : pose not re able
};
typedef TimePose TP3D;
typedef std::vector<TimePose> TimePoseList;
typedef std::vector<TimePose> TP3DL;


class Quaternion{
public:
  Quaternion(){}
  Quaternion(double _x, double _y, double _z, double _w):
  x(_x),y(_y),z(_z),w(_w){}
  ~Quaternion(){}
  friend std::ostream& operator<<(std::ostream& os, const Quaternion &q){
    os << "[x:"<<q.x << " y:" << q.y << " z:" << q.z << " w:" << q.w <<"]";
    return os;
  }
  double x=0;
  double y=0;
  double z=0;
  double w=0;
};

class Pose3d{
public:
  Pose3d(){}
  ~Pose3d(){}

  lk::Point3d position;
  lk::Quaternion orientation;
  int64_t timeStamp;
};

class Color{
public:
  Color(){}
  ~Color(){}

  double r;
  double g;
  double b;
};

typedef std::vector<Color> ColorList;

class ScorePose{
public:
  ScorePose(){}
  ScorePose(double _s,lk::Point3d _p):score(_s),pose(_p){}
  double score;
  lk::Point3d pose;
  double flag=0;

  bool operator <(const ScorePose& sp) const{
    return score<sp.score;
  }
  bool operator >(const ScorePose& sp) const{
    return score>sp.score;
  }
  bool operator <(double s) const{
    return score<s;
  }
  bool operator >(double s) const{
    return score>s;
  }

  friend std::ostream& operator<<(std::ostream& os, const ScorePose &sp){
    os << "[" << std::fixed << std::setprecision(4) << sp.score
       << ":" << std::fixed << std::setprecision(4) << sp.pose.x
       << "," << std::fixed << std::setprecision(4) << sp.pose.y
       << "," << std::fixed << std::setprecision(4)
       << (sp.pose.z/3.1415926)*180 << "(du)(" << sp.flag << ")]";
    return os;
  }
};
typedef ScorePose SP;
typedef std::vector<SP> ScorePoseList;
typedef ScorePoseList SPL;

template<typename _Tp> class Rect_
{
public:
  Rect_():x(0),y(0),w(0),h(0){}
  Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height)
    :x(_x),y(_y),w(_width),h(_height){}
  Rect_(lk::Point_<double> _xy, _Tp _width, _Tp _height)
    :x(_xy.x),y(_xy.y),w(_width),h(_height){}
  Rect_(lk::Point_<float> _xy, _Tp _width, _Tp _height)
    :x(_xy.x),y(_xy.y),w(_width),h(_height){}
  Rect_(lk::Point_<int> _xy, _Tp _width, _Tp _height)
    :x(_xy.x),y(_xy.y),w(_width),h(_height){}

  inline Point_<_Tp> br(){
    return Point_<_Tp>(x+w,y+h);
  }
  inline Point_<_Tp> tl(){
    return Point_<_Tp>(x,y);
  }
  inline bool operator==(Rect_<_Tp> _r){
    return (_r.x==x && _r.y==y && _r.w==w && _r.h==h);
  }
  inline bool contains(Point_<double> p){
    return !((p.x < x || p.x >= x+w)||(p.y < y || p.y >= y+h));
  }
  inline bool contains(Point_<float> p){
    return !((p.x < x || p.x >= x+w)||(p.y < y || p.y >= y+h));
  }
  inline bool contains(Point_<int> p){
    return !((p.x < x || p.x >= x+w)||(p.y < y || p.y >= y+h));
  }
  inline bool contains(Point3_<double> p){
    return !((p.x < x || p.x >= x+w)||(p.y < y || p.y >= y+h));
  }
  inline bool contains(Point3_<float> p){
    return !((p.x < x || p.x >= x+w)||(p.y < y || p.y >= y+h));
  }
  inline bool contains(Point3_<int> p){
    return !((p.x < x || p.x >= x+w)||(p.y < y || p.y >= y+h));
  }

  inline _Tp getArea(){
    return w*h;
  }

  Rect_<_Tp> getUnion(Rect_<_Tp> _r){
    Rect_<_Tp> out;
    out.x=std::min(x,_r.x);
    out.y=std::min(y,_r.y);
    _r.br().x>br().x?out.w=_r.br().x-out.x:out.w=br().x-out.x;
    _r.br().y>br().y?out.h=_r.br().y-out.y:out.h=br().y-out.y;
    return out;
  }
  Rect_<_Tp> getInter(Rect_<_Tp> _r){
    Rect_<_Tp> out;
    out.x=std::max(x,_r.x);
    out.y=std::max(y,_r.y);
    _r.br().x<br().x ? out.w=_r.br().x-out.x : out.w=br().x-out.x;
    _r.br().y<br().y ? out.h=_r.br().y-out.y : out.h=br().y-out.y;
    if(out.w<0||out.h<0){
      out.w=0;
      out.h=0;
    }
    return out;
  }
  Rect_<_Tp> enlarge(_Tp _r){
    Rect_<_Tp> out;
    out.x=x-_r;
    out.y=y-_r;
    out.w=w+2*_r;
    out.h=h+2*_r;
    return out;
  }

  bool isWrong(){
    return (w<=0||h<=0);
  }

  friend std::ostream& operator<<(std::ostream& os, const Rect_<_Tp> &dt){
    os << "[x:"<<dt.x << " y:" << dt.y
       << " w:" << dt.w << " h:" << dt.h << "]";
    return os;
  }

  _Tp x, y, w, h; //< the top-left corner, as well as width and height of the rectangle
  int flag;
};

typedef Rect_<int> Rect;
typedef Rect_<double> Rectd;
typedef std::vector<Rect> RectList;
typedef std::vector<Rectd> RectdList;

template<typename _Tp> class Mat_
{
public:
  int width;
  int height;
  std::vector<_Tp> data;
  _Tp* p;

  Mat_(){}
  Mat_(int _w, int _h):width(_w),height(_h){
    data.resize(width*height,0);
    p=data.data();
  }
  ~Mat_(){}
  inline _Tp& at(int row, int col){
    return p[row*width+col];
  }
  inline _Tp& at(lk::Point2i _p){
    return p[_p.y*width+_p.x];
  }
  inline void setSize(int _w, int _h){
    width=_w;
    height=_h;
    data.resize(width*height,0);
    p=data.data();
  }
  inline void setZero(){
    memset(data.data(),0,data.size()*sizeof(_Tp));
  }
};

typedef lk::Mat_<uint8_t> MatU8;
typedef lk::Mat_<int> MatS32;
typedef lk::Mat_<double> MatF64;
typedef lk::Mat_<double> Mat;

struct Pose{
  float x;
  float y;
  float theta;
};

struct Line{
  lk::Point2d from;
  lk::Point2d to;
  std::vector<lk::Point2d> List;
};

template<typename T1, typename T2> class FindMax_{
public:
  T1 k;
  T2 v;
  FindMax_():v(std::numeric_limits<T2>::min()){}
  inline void check(T1 _k, T2 _v){
    if(_v>v){
      v=_v;
      k=_k;
    }
  }
  inline bool found(){
    return (v>std::numeric_limits<T2>::min());
  }
};

template<typename T1,typename T2> class FindMin_{
public:
  T1 k;
  T2 v;
  FindMin_():v(std::numeric_limits<T2>::max()){}
  inline void check(T1 _k, T2 _v){
    if(_v<v){
      v=_v;
      k=_k;
    }
  }
  inline bool found(){
    return (v<std::numeric_limits<T2>::max());
  }
};

typedef FindMax_<int,int> FindMaxS32;
typedef FindMax_<int,double> FindMaxF64;
typedef FindMax_<int,double> FindMax;
typedef FindMin_<int,int> FindMinS32;
typedef FindMin_<int,double> FindMinF64;
typedef FindMin_<int,double> FindMin;

struct AngleRange{
  AngleRange(){}
  AngleRange(float _a, float _r):a(_a),r(_r){}
  float a = 0.f;
  float r = 0.f;
  typedef float value_type;

  bool operator < (const AngleRange &ar)const {
    return a < ar.a;
  }
  void operator=(AngleRange ar){
    a=ar.a;
    r=ar.r;
  }
  friend std::ostream& operator<<(std::ostream& os, const AngleRange &ar){
    os << "[a:"<<ar.a << "("<< ar.a/3.1415926*180.0 <<") r:" << ar.r << "]";
    return os;
  }
};
typedef AngleRange AR;
typedef std::vector<AngleRange> AngleRangeList;
typedef std::vector<AngleRange> ARL;

struct AngleRangeIntensity{
  AngleRangeIntensity(){}
  AngleRangeIntensity(double _a, double _r, int _i=0){
    ar.a=_a;
    ar.r=_r;
    i=_i;
  }
  AngleRangeIntensity(double _a, double _r, int _i=0, int _f=0){
    ar.a=_a;
    ar.r=_r;
    i=_i;
    flag=_f;
  }
  AngleRangeIntensity(AngleRange _ar, int _i=0,int _f=0){
    ar=_ar;
    i=_i;
    flag=_f;
  }
  AngleRange ar;
  int16_t i=0;
  int16_t flag=0; // 1 is columu
  bool operator < (const AngleRangeIntensity &ari)const {
    return ar.a < ari.ar.a;
  }
  void operator=(AngleRangeIntensity ari){
    ar=ari.ar;
    i=ari.i;
    flag=ari.flag;
  }
  friend std::ostream& operator<<(std::ostream& os, const AngleRangeIntensity &ari){
    os << "[a:"<<ari.ar.a/3.1415926*180.0
       << " r:" << ari.ar.r
       << "i:" << ari.i
       << "f:" << ari.flag
       << "]";
    return os;
  }
};


typedef AngleRangeIntensity ARI;
typedef std::vector<AngleRangeIntensity> ARIL;
}
#endif