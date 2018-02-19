// This
#include "FindTransform.hpp"

Vec3::Vec3(double x, double y, double z) :x_(x), y_(y), z_(z)
{
    
}
Vec3::Vec3(const Vec3 &o) : x_(o.x_), y_(o.y_), z_(o.z_)
{
    
}

Vec3 Vec3::cross(Vec3 o) const
{
    Vec3 res;
    res.x_ = y_*o.z_ - z_*o.y_;
    res.y_ = z_*o.x_ - x_*o.z_;
    res.z_ = x_*o.y_ - y_*o.x_;
    return res;
}

double Vec3::dot(Vec3 o) const
{
    return x_*o.x_ + y_*o.y_ + z_*o.z_;
}

double Vec3::length() const
{
    return sqrt(x_*x_ + y_*y_ + z_*z_);
}

Vec3 Vec3::norm() const
{
    Vec3 res;
    double l = length();
    res.x_ = x_ / l;
    res.y_ = y_ / l;
    res.z_ = z_ / l;
    return res;
}

Vec3 Vec3::operator*(double d) const
{
    return Vec3(x_*d, y_*d, z_*d);
}

Vec3 Vec3::operator+(Vec3 o) const
{
    return Vec3(x_ + o.x_, y_ + o.y_,z_ + o.z_);
}

Vec3 Vec3::operator-(Vec3 o) const
{
    return Vec3(x_ - o.x_, y_ - o.y_,z_ - o.z_);
}

OnMat3::OnMat3(Vec3 r1, Vec3 r2, Vec3 r3) : r1_(r1), r2_(r2), r3_(r3)
{
    c1_.x_ = r1_.x_; c2_.x_ = r1_.y_; c3_.x_ = r1_.z_;
    c1_.y_ = r2_.x_; c2_.y_ = r2_.y_; c3_.y_ = r2_.z_;
    c1_.z_ = r3_.x_; c2_.z_ = r3_.y_; c3_.z_ = r3_.z_;
}
OnMat3::OnMat3(const OnMat3 &o) : r1_(o.r1_), r2_(o.r2_), r3_(o.r3_)
{
    c1_.x_ = r1_.x_; c2_.x_ = r1_.y_; c3_.x_ = r1_.z_;
    c1_.y_ = r2_.x_; c2_.y_ = r2_.y_; c3_.y_ = r2_.z_;
    c1_.z_ = r3_.x_; c2_.z_ = r3_.y_; c3_.z_ = r3_.z_;
}

double OnMat3::det() const
{
    return r1_.x_*r2_.y_*r3_.z_ + r1_.y_*r2_.z_*r3_.x_ + r1_.z_*r2_.x_*r3_.y_ - r1_.z_*r2_.y_*r3_.x_ - r1_.y_*r2_.x_*r3_.z_ - r1_.x_*r2_.z_*r3_.y_;
}

OnMat3 OnMat3::transpose() const
{
    return OnMat3(Vec3(r1_.x_, r2_.x_, r3_.x_), Vec3(r1_.y_, r2_.y_, r3_.y_), Vec3(r1_.z_, r2_.z_, r3_.z_));
}

OnMat3 OnMat3::operator*(OnMat3 o) const
{
    return OnMat3(  Vec3(r1_.dot(o.c1_), r1_.dot(o.c2_), r1_.dot(o.c3_)),
                    Vec3(r2_.dot(o.c1_), r2_.dot(o.c2_), r2_.dot(o.c3_)),
                    Vec3(r3_.dot(o.c1_), r3_.dot(o.c2_), r3_.dot(o.c3_)));
}

Vec3 OnMat3::operator*(Vec3 o)
{
    return Vec3(r1_.dot(o), r2_.dot(o), r3_.dot(o));
}

Mat4::Mat4()
{
    r1c1_ = 1.0;	r1c2_ = 0.0;	r1c3_ = 0.0;	r1c4_ = 0.0;
    r2c1_ = 0.0;	r2c2_ = 1.0;	r2c3_ = 0.0;	r2c4_ = 0.0;
    r3c1_ = 0.0;	r3c2_ = 0.0;	r3c3_ = 1.0;	r3c4_ = 0.0;
    r4c1_ = 0.0;	r4c2_ = 0.0;	r4c3_ = 0.0;	r4c4_ = 1.0;
}

Mat4::Mat4(OnMat3 rotation, Vec3 translation, double scaling)
{
    r1c1_ = scaling * rotation.r1_.x_;	r1c2_ = scaling * rotation.r1_.y_;	r1c3_ = scaling * rotation.r1_.z_;	r1c4_ = translation.x_;
    r2c1_ = scaling * rotation.r2_.x_;	r2c2_ = scaling * rotation.r2_.y_;	r2c3_ = scaling * rotation.r2_.z_;	r2c4_ = translation.y_;
    r3c1_ = scaling * rotation.r3_.x_;	r3c2_ = scaling * rotation.r3_.y_;	r3c3_ = scaling * rotation.r3_.z_;	r3c4_ = translation.z_;
    r4c1_ = 0.0;						r4c2_ = 0.0;						r4c3_ = 0.0;						r4c4_ = 1.0;
}

Vec3 Mat4::operator*(Vec3 o)
{
    return Vec3(
                r1c1_ * o.x_ + r1c2_* o.y_ + r1c3_* o.z_ + r1c4_,
                r2c1_ * o.x_ + r2c2_* o.y_ + r2c3_* o.z_ + r2c4_,
                r3c1_ * o.x_ + r3c2_* o.y_ + r3c3_* o.z_ + r3c4_
                );
}

void FindTransform::findTransform(Vec3 fromA, Vec3 fromB, Vec3 fromC, Vec3 toA, Vec3 toB, Vec3 toC)
{
    Vec3 a1 = toA;
    Vec3 b1 = toB;
    Vec3 c1 = toC;
    Vec3 a2 = fromA;
    Vec3 b2 = fromB;
    Vec3 c2 = fromC;
        
    Vec3 y1 = (a1 - c1).cross(b1 - c1).norm();
    Vec3 z1 = (a1 - c1).norm();
    Vec3 x1 = y1.cross(z1);
    
    Vec3 y2 = (a2 - c2).cross(b2 - c2).norm();
    Vec3 z2 = (a2 - c2).norm();
    Vec3 x2 = y2.cross(z2);
    OnMat3 mat1 = OnMat3(x1, y1, z1).transpose();
    OnMat3 mat2 = OnMat3(x2, y2, z2).transpose();
    
    OnMat3 rotation = mat1 * mat2.transpose();
    Vec3 translation = c1 - c2;
    
    double scale = (a1 - c1).length() / (a2 - c2).length();
    
    translation = rotation * c2 * (-scale) + c1;
    Mat4 transformation(rotation, translation, scale);
    transform_ = transformation;
}

double FindTransform::error(Vec3 fromA, Vec3 toA)
{
    return (transform_*fromA - toA).length();
}
