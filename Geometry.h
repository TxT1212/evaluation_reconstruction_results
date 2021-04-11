#include <iostream>
#include <vector>
#include <map>
#include <stack>
#include <algorithm>
#include <cmath>

using namespace std;

/*** 常用常量 ***/
const double PI = 3.14159265;

/******************************* 基本几何数据类型 *******************************/
// 点,二维和三维,同时点也可以表示一个矢量
struct GeoPoint
{
    double x;    // x坐标
    double y;    // y坐标
    double z;    // z坐标（默认为0，如果需要三维点则给z赋值）

    GeoPoint(double a = 0, double b = 0, double c = 0) { x = a; y = b; z = c; } // 构造函数
};
// 点的加法
GeoPoint add(const GeoPoint& lhs, const GeoPoint& rhs)
{
    GeoPoint res;

    res.x = lhs.x + rhs.x;
    res.y = lhs.y + rhs.y;
    res.z = lhs.z + rhs.z;

    return res;
}
// 点的减法
GeoPoint sub(const GeoPoint& lhs, const GeoPoint& rhs)
{
    GeoPoint res;

    res.x = lhs.x - rhs.x;
    res.y = lhs.y - rhs.y;
    res.z = lhs.z - rhs.z;

    return res;
}
// 向量的乘法
GeoPoint mul(const GeoPoint& p, double ratio)
{
    GeoPoint res;

    res.x = p.x * ratio;
    res.y = p.y * ratio;
    res.z = p.z * ratio;

    return res;
}
// 向量的除法
GeoPoint div(const GeoPoint& p, double ratio)
{
    GeoPoint res;
    
    res.x = p.x / ratio;
    res.y = p.y / ratio;
    res.z = p.z / ratio;

    return res;
}
// 点判断相等
bool equal(const GeoPoint& lhs, const GeoPoint& rhs)
{
    return(lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z);
}
// 线,包括线段和直线
struct Line
{
    GeoPoint s;    // 起点
    GeoPoint e;    // 终点
    bool is_seg; // 是否是线段

    Line() {};    // 默认构造函数
    Line(GeoPoint a, GeoPoint b, bool _is_seg = true) { s = a; e = b; is_seg = _is_seg; }    // 构造函数(默认是线段)
};
// 三角形平面
struct Triangle
{
    GeoPoint v0;
    GeoPoint v1;
    GeoPoint v2;
    bool is_plane;

    Triangle() {}; // 默认构造函数
    Triangle(GeoPoint a, GeoPoint b, GeoPoint c, bool _is_plane = false) { v0 = a; v1 = b; v2 = c; is_plane = _is_plane; }// 构造函数（默认是三角形）
};

/******************************* 计算几何算法目录 *******************************/

// 一、点
// 1.1、两点之间的距离
double distance(const GeoPoint& p1, const GeoPoint& p2);

// 1.2、矢量长度
double length(const GeoPoint& vec);

// 1.3、矢量标准化
GeoPoint normalize(const GeoPoint& vec);

// 1.4、矢量点乘
double dotMultiply(const GeoPoint& op, const GeoPoint& p1, const GeoPoint& p2);
double dotMultiply(const GeoPoint& vec1, const GeoPoint& vec2);

// 1.5、矢量叉乘
GeoPoint multiply(const GeoPoint& op, const GeoPoint& p1, const GeoPoint& p2);
GeoPoint multiply(const GeoPoint& vec1, const GeoPoint& vec2);

// 1.6、点到线的距离
double ptolDistance(const GeoPoint& p, const Line& l);

// 1.7、点到线的投影点
GeoPoint ptolProjection(const GeoPoint& p, const Line& l);

// 1.8、点关于线的对称点
GeoPoint ptolSymmetry(const GeoPoint& p, const Line& l);

// 1.9、点是否在线上
bool isponl(const GeoPoint& p, const Line& l);

// 1.10、矢量夹角正弦
double Sin(const GeoPoint& op, const GeoPoint& p1, const GeoPoint& p2);
double Sin(const GeoPoint& vec1, const GeoPoint& vec2);

// 1.11、矢量夹角余弦
double Cos(const GeoPoint& op, const GeoPoint& p1, const GeoPoint& p2);
double Cos(const GeoPoint& vec1, const GeoPoint& vec2);

// 1.12、矢量夹角正切
double Tan(const GeoPoint& op, const GeoPoint& p1, const GeoPoint& p2);
double Tan(const GeoPoint& vec1, const GeoPoint& vec2);

// 1.13、矢量夹角角度
double Angle(const GeoPoint& op, const GeoPoint& p1, const GeoPoint& p2,bool is_radian = true);
double Angle(const GeoPoint& vec1, const GeoPoint& vec,bool is_radian = true);

// 1.14、判断三点是否共线
bool isGeoPointsCollinear(const GeoPoint& p1, const GeoPoint& p2, const GeoPoint& p3);

// 二、线
// 2.1、线段是否相交
bool isSegIntersect(const Line& l1, const Line& l2,GeoPoint& inter_p);

// 2.2、求直线的夹角
double angleOfLines(const Line& l1, const Line& l2,bool is_radian = true);

// 2.3、一阶贝塞尔曲线插值
vector<GeoPoint> firstOrderBezier(const GeoPoint& s, const GeoPoint& e, int inter_num);

// 2.4、二阶贝塞尔曲线插值
vector<GeoPoint> secondOrderBezier(const GeoPoint& s, const GeoPoint& e, const GeoPoint& p,int inter_num);

// 2.5、三阶贝塞尔曲线插值
vector<GeoPoint> thirdOrderBezier(const GeoPoint& s, const GeoPoint& e, const GeoPoint& p1,const GeoPoint& p2,int inter_num);

// 三、三角形
// 3.1、三角形三个点是否能够构成三角形
bool isTriangle(const Triangle& t);

// 3.2、点是否在三角形内部
bool isGeoPointInTriangle(const Triangle& t, const GeoPoint& p);

// 3.3、点到平面的投影点（距离最近的点）
GeoPoint ptotProjection(const Triangle& t, const GeoPoint& p);

// 3.4、点到平面的距离
double ptotDistance(const Triangle& t, const GeoPoint& p);

// 3.5、线段和平面的交点
GeoPoint ltotInterGeoPoint(const Triangle& t, const Line& l);

// 3.6、计算平面的单位法向量
GeoPoint getUnitNormal(const Triangle& t);

// 3.7、计算三角形的面积
double areaOfTriangle(const Triangle& t);


/******************************* 计算几何算法实现 *******************************/
//一、点

// 1.1、两点之间的距离
//
// 参数：p1 : 第一个点 p2: 第二个点
//  
double distance(const GeoPoint& p1, const GeoPoint& p2)
{
    return(sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2)));
}

// 1.2、矢量的长度
//
// 参数： vec 矢量
//
double length(const GeoPoint& vec)
{
    return (sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2)));
}

// 1.3、矢量标准化（矢量的长度规约到1）
//
// 参数： vec ： 矢量
//
GeoPoint normalize(const GeoPoint& vec)
{
    GeoPoint res;

    res = div(vec, length(vec));

    return res;
}

// 1.4、矢量点乘
//
// 参数：(p1-op)为矢量1，（p2-op）为矢量2
//
double dotMultiply(const GeoPoint& op, const GeoPoint& p1, const GeoPoint& p2)
{
    return ((p1.x - op.x) * (p2.x - op.x) + (p1.y - op.y) * (p2.y - op.y) + (p1.z - op.z) * (p2.z - op.z));
}
// 参数：vec1为矢量1，vec2为矢量2
//
double dotMultiply(const GeoPoint& vec1, const GeoPoint& vec2)
{
    return(vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z);
}

// 1.5、矢量叉乘
//
// 参数：(p1-op)为矢量1，（p2-op）为矢量2
// 
GeoPoint multiply(const GeoPoint& op, const GeoPoint& p1, const GeoPoint& p2)
{
    GeoPoint result;

    result.x = (p1.y - op.y) * (p2.z - op.z) - (p2.y - op.y) * (p1.z - op.z);
    result.y = (p1.z - op.z) * (p2.x - op.x) - (p2.z - op.z) * (p1.x - op.x);
    result.z = (p1.x - op.x) * (p2.y - op.y) - (p2.x - op.x) * (p1.y - op.y);

    return result;
}
// 参数： vec1为矢量1，vec2为矢量2
//
GeoPoint multiply(const GeoPoint& vec1, const GeoPoint& vec2)
{
    GeoPoint result;

    result.x = vec1.y * vec2.z - vec2.y * vec1.z;
    result.y = vec1.z * vec2.x - vec2.z * vec1.x;
    result.z = vec1.x * vec2.y - vec2.x * vec1.y;

    return result;
}

// 1.6、点到线的距离
// 
// 参数： p : 点  l：直线
//
double ptolDistance(const GeoPoint& p, const Line& l)
{
    GeoPoint line_vec = sub(l.e,l.s);
    GeoPoint GeoPoint_vec = sub(p, l.s);

    // 首先计算点在线段投影长度
    double project_len = dotMultiply(line_vec, GeoPoint_vec) / length(line_vec);

    // 根据勾股定理计算点的距离
    double distance = sqrt(pow(length(line_vec), 2) - pow(project_len, 2));

    return distance;
}

// 1.7、点到线的投影点
//
// 参数：p : 点  l : 线
//
GeoPoint ptolProjection(const GeoPoint& p, const Line& l)
{
    GeoPoint line_vec = sub(l.e, l.s);
    GeoPoint GeoPoint_vec = sub(p, l.s);
    GeoPoint unit_line_vec = normalize(line_vec);

    // 计算投影长度
    double project_len = dotMultiply(GeoPoint_vec, unit_line_vec);

    // 投影点
    GeoPoint project_p = add(l.s,mul(unit_line_vec, project_len));

    return project_p;
}

// 1.8、点关于线的对称点
//
// 参数： p : 点  l : 对称线
//
GeoPoint ptolSymmetry(const GeoPoint& p, const Line& l)
{
    // 首先求出点在直线上的投影点
    GeoPoint project_p = ptolProjection(p, l);

    // 点到投影点的向量
    GeoPoint project_vec = sub(project_p, p);

    // 对称点
    GeoPoint symmetry_p = add(p, mul(project_vec, 2));

    return symmetry_p;
}

// 1.9、点是否在线上
// 线分为直线和线段，直线表示的是直线是否经过点
//
// 参数：p : 点  l : 线段或者线
// 
bool isponl(const GeoPoint& p, const Line& l)
{
    GeoPoint line_vec = sub(l.e, l.s);
    GeoPoint GeoPoint_vec1 = sub(p, l.s);
    GeoPoint GeoPoint_vec2 = sub(p, l.e);

    GeoPoint mul_vec = multiply(line_vec, GeoPoint_vec1);
    double dot = dotMultiply(GeoPoint_vec1, GeoPoint_vec2);
    // 点是否在线段上
    if (l.is_seg)
    {
        if (equal(p,l.s) || equal(p,l.e))
            return true;
        return (0.0 == length(mul_vec) && dot < 0.0);
        
    }
    // 点是否在直线上
    else
    {
        return (0.0 == length(mul_vec));
    }
}

// 1.10、矢量夹角正弦
//
// 参数： op : 矢量公共点 p1 : 矢量1端点 p2 : 矢量2端点
//
double Sin(const GeoPoint& op, const GeoPoint& p1, const GeoPoint& p2)
{
    GeoPoint vec1 = sub(p1, op);
    GeoPoint vec2 = sub(p2, op);

    return Sin(vec1, vec2);
}
// 参数： vec1 矢量1  vec2 矢量2
// 
double Sin(const GeoPoint& vec1, const GeoPoint& vec2)
{
    return sqrt(1.0 - pow(Cos(vec1, vec2), 2));
}

// 1.11、矢量夹角余弦
//
// 参数： op : 矢量公共点 p1 : 矢量1端点 p2 : 矢量2端点
//
double Cos(const GeoPoint& op, const GeoPoint& p1, const GeoPoint& p2)
{
    GeoPoint vec1 = sub(p1, op);
    GeoPoint vec2 = sub(p2, op);

    return Cos(vec1, vec2);
}
// 参数： vec1 矢量1  vec2 矢量2
// 
double Cos(const GeoPoint& vec1, const GeoPoint& vec2)
{
    GeoPoint unit_vec1 = normalize(vec1);
    GeoPoint unit_vec2 = normalize(vec2);

    return dotMultiply(unit_vec1, unit_vec2);
}

// 1.12、矢量夹角正切
//
// 参数： op : 矢量公共点 p1 : 矢量1端点 p2 : 矢量2端点
//
double Tan(const GeoPoint& op, const GeoPoint& p1, const GeoPoint& p2)
{
    GeoPoint vec1 = sub(p1, op);
    GeoPoint vec2 = sub(p2, op);

    return Tan(vec1, vec2);
}
// 参数： vec1 矢量1  vec2 矢量2
// 
double Tan(const GeoPoint& vec1, const GeoPoint& vec2)
{
    double cos = Cos(vec1, vec2);
    double sin = Sin(vec1, vec2);

    // 分母不为零
    if (0.0 == cos)
        return -1;
    else
        return (sin / cos);
}

// 1.13、计算点的夹角角度
// 参数:  op : 矢量公共点 p1 : 矢量1端点 p2 : 矢量2端点 is_radian : 默认为弧度制
//
double Angle(const GeoPoint& op, const GeoPoint& p1, const GeoPoint& p2, bool is_radian)
{
    double cos_value = Cos(op, p1, p2);

    if (is_radian)
    {
        return acos(cos_value);
    }
    else
    {
        return (acos(cos_value) / PI * 180.0);
    }
}
// 参数： vec1 : 矢量1 vec2 : 矢量2
// 
double Angle(const GeoPoint& vec1, const GeoPoint& vec2,bool is_radian)
{
    double cos_value = Cos(vec1, vec2);

    if (is_radian)
    {
        return acos(cos_value);
    }
    else
    {
        return (acos(cos_value) / PI * 180.0);
    }
}

// 1.14、判断三点是否共线
bool isGeoPointsCollinear(const GeoPoint& p1, const GeoPoint& p2, const GeoPoint& p3)
{
    Line line(p1, p2, false);

    // 判断第三个点是否在前两个点的线段上
    return isponl(p3, line);
}

// 二、线

// 2.1、线段是否相交
// 其中如果线段的端点重合或者某个线段端点在另一个线段上也算相交
// 线段判断是否相交，如果是直线则相当于判断是否平行
//
// 参数： l1 : 线段1  l2 : 线段2  inter_p : 如果相交返回交点
//
bool isSegIntersect(const Line& l1, const Line& l2,GeoPoint& inter_p)
{
    GeoPoint line1 = sub(l1.e, l1.s);
    GeoPoint line2 = sub(l2.e, l2.s);
    GeoPoint norm1 = normalize(line1);
    GeoPoint norm2 = normalize(line2);
    // 线段相交
    if (l1.is_seg)
    {
        // 端点在线段上
        if (isponl(l1.s, l2))
        {
            inter_p = l1.s;
            return true;
        }
        if (isponl(l1.e, l2))
        {
            inter_p = l1.e;
            return true;
        }
        if (isponl(l2.s, l1))
        {
            inter_p = l2.s;
            return true;
        }
        if (isponl(l2.e, l1))
        {
            inter_p = l2.e;
            return true;
        }
        // 判断线段是否相互跨立
        double dot1 = dotMultiply(multiply(sub(l2.s, l1.s), line1), multiply(sub(l2.e, l1.s), line1));
        double dot2 = dotMultiply(multiply(sub(l1.s, l2.s), line2), multiply(sub(l1.e, l2.s), line2));
        if (dot1 < 0.0 && dot2 < 0.0)
        {
            double t1 = length(multiply(sub(l1.s, l2.s), norm2)) / length(multiply(norm2, norm1));
            double t2 = length(multiply(sub(l2.s, l1.s), norm1)) / length(multiply(norm1, norm2));

            inter_p = add(l1.s, mul(norm1, t1));
            return true;
        }
        else
        {
            return false;
        }

    }
    // 直线相交
    else
    {
        if (Cos(line1, line2) == 1.0)
            return false;

        double t1 = length(multiply(sub(l1.s, l2.s), norm2)) / length(multiply(norm2, norm1));
        double t2 = length(multiply(sub(l2.s, l1.s), norm1)) / length(multiply(norm1, norm2));

        inter_p = add(l1.s, mul(norm1, t1));
        return true;
    }
}

// 2.2、求直线的夹角
// 
// 
// 参数： l1 : 线段1 l2 : 线段2
//
double angleOfLines(const Line& l1, const Line& l2,bool is_radian)
{
    GeoPoint line1 = sub(l1.e, l1.s);
    GeoPoint line2 = sub(l2.e, l2.s);

    return Angle(line1, line2, is_radian);
}

// 2.3、一阶贝塞尔曲线插值
// 
// 参数： s： 起点 e : 终点 inter_num：插值点数量（不包括起始点）
// 返回值包括起始点
//
vector<GeoPoint> firstOrderBezier(const GeoPoint& s, const GeoPoint& e, int inter_num)
{
    vector<GeoPoint> res;
    res.push_back(s);
    for (int i = 1; i <= inter_num; ++i)
    {
        double a1 = double(i) / double(inter_num + 1);
        double a2 = 1.0 - a1;
        res.push_back(add(mul(s, a2), mul(e, a1)));
    }
    res.push_back(e);

    return res;
}

// 2.4、二阶贝塞尔曲线插值
// 
// 参数： s： 起点 e : 终点 p : 控制点 inter_num：插值点数量（不包括起始点）
// 返回值包括起始点
//
vector<GeoPoint> secondOrderBezier(const GeoPoint& s, const GeoPoint& e, const GeoPoint& p, int inter_num)
{
    vector<GeoPoint> res;
    res.push_back(s);
    for (int i = 1; i <= inter_num; ++i)
    {
        double a = double(i) / double(inter_num + 1);
        double a1 = pow(a,2);
        double a2 = 2 * a * (1.0 - a);
        double a3 = pow(1.0 - a, 2);
        res.push_back(add(add(mul(s, a3), mul(p, a2)), mul(e, a1)));
    }
    res.push_back(e);

    return res;
}

// 2.5、三阶贝塞尔曲线插值
// 
// 参数： s： 起点 e : 终点 p1，p2 : 控制点 inter_num：插值点数量（不包括起始点）
// 返回值包括起始点
//
vector<GeoPoint> thirdOrderBezier(const GeoPoint& s, const GeoPoint& e, const GeoPoint& p1, const GeoPoint& p2, int inter_num)
{
    vector<GeoPoint> res;
    res.push_back(s);
    for (int i = 1; i <= inter_num; ++i)
    {
        double a = double(i) / double(inter_num + 1);
        double a1 = pow(a, 3);
        double a2 = 3 * pow(a, 2) * (1.0 - a);
        double a3 = 3 * pow(1.0 - a, 2) * a;
        double a4 = pow(1.0 - a, 3);
        res.push_back(add(add(add(mul(s, a4), mul(p1, a3)), mul(p2, a2)),mul(e,a1)));
    }
    res.push_back(e);

    return res;
}

// 三、三角形

// 3.1、三角形三个点是否能够构成三角形
// 不共线的三个点组成一个三角形
// 
// 参数： t : 三角形
// 
bool isTriangle(const Triangle& t)
{
    return isGeoPointsCollinear(t.v0, t.v1, t.v2);
}

// 3.2、点是否在三角形内部（重心法）
// 算法方法链接： https://www.cnblogs.com/graphics/archive/2010/08/05/1793393.html
//
// 参数： t : 三角形 p : 需要判断的点 u,v分别为用于表示点在两条边上投影系数
//
bool isGeoPointInTriangle(const Triangle& t, const GeoPoint& p)
{
    GeoPoint vec1 = sub(t.v1, t.v0);
    GeoPoint vec2 = sub(t.v2, t.v0);
    GeoPoint vec_p = sub(p, t.v0);

    double dot00 = dotMultiply(vec1, vec1);
    double dot01 = dotMultiply(vec1, vec2);
    double dot02 = dotMultiply(vec1, vec_p);
    double dot11 = dotMultiply(vec2, vec2);
    double dot12 = dotMultiply(vec2, vec_p);

    double inverDeno = 1 / (dot00 * dot11 - dot01 * dot01);
    double u, v;

    u = (dot11 * dot02 - dot01 * dot12) * inverDeno;
    v = (dot00 * dot12 - dot01 * dot02) * inverDeno;

    if (u < 0 || u > 1) return false;
    if (v < 0 || v > 1) return false;
    if (u + v < 1)return true;
    else return false;
}

// 3.3、点到平面最近的点，即点到平面的投影
// 
// 参数：t : 三角形 p : 点
// 
GeoPoint ptotProjection(const Triangle& t, const GeoPoint& p)
{
    GeoPoint vec_p = sub(p, t.v0);
    GeoPoint unit_normal = getUnitNormal(t);

    double ratio = dotMultiply(vec_p, unit_normal);

    return sub(p, mul(unit_normal, ratio));
}

// 3.4、点到平面的距离
//
// 参数： t : 三角形所在的平面 p : 需要判断的点
//
double ptotDistance(const Triangle& t, const GeoPoint& p)
{
    GeoPoint project_p = ptotProjection(t, p);

    return distance(p, project_p);
}

// 3.5、线段和平面的交点
// 
// 参数： t : 三角形所在平面 l : 直线
//
GeoPoint ltotInterGeoPoint(const Triangle& t, const Line& l)
{
    GeoPoint line_vec = sub(l.e, l.s);
    GeoPoint GeoPoint_vec = sub(t.v0, l.s);
    GeoPoint unit_plane_normal = getUnitNormal(t);

    double ratio = dotMultiply(GeoPoint_vec, unit_plane_normal) / dotMultiply(unit_plane_normal, line_vec);

    return add(l.s, mul(line_vec, ratio));
}

// 3.6、计算平面的单位法向量
// 
// 参数： t : 三角形平面
// 
GeoPoint getUnitNormal(const Triangle& t)
{
    GeoPoint vec1 = sub(t.v1, t.v0);
    GeoPoint vec2 = sub(t.v2, t.v0);

    return normalize(multiply(vec1, vec2));
}

// 3.7、计算三角形的面积
//
// 参数： t : 三角形平面
//
double areaOfTriangle(const Triangle& t)
{
    return (0.5 * length(multiply(sub(t.v1,t.v0),sub(t.v2,t.v0))));
}