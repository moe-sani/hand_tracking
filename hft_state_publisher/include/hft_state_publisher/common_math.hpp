/**
 * @file    common_math.hpp
 * @author  Nikhil Deshpande
 * @version 2.0
 * @date    21-Apr-2017 / 03-May-2018
 * @brief   Common Math functions
 * @details This class is for common math functions.
 */

#ifndef COMMON_MATH_HPP
#define COMMON_MATH_HPP

#include "Eigen/Core"
#include "Eigen/SVD"
#include "Eigen/Dense"

//--------------------
// Template functions -- common math
//--------------------

template <typename T> T sign( T );
template <typename T> float Mean( T );
template <typename T> float Variance( T );
template <typename T> float Std( T );
template <typename T> float DegToRad( T );
template <typename T> float RadToDeg( T );

template <typename T>
T sign(T n)
{
    if (n < 0) return -1;
    if (n > 0) return 1;
    return 0;
}

template <typename T>
float Mean(T x)
{
    int n = x.size();
    double xm = 0;
    for(int i = 0; i < n; i++)
        xm += x[i];
    xm /= n;
    return xm;
}

template <typename T>
float Variance(T x)
{
    int n = x.size();
    double xm = Mean(x);
    double var = 0;
    for(int i = 0; i < n; i++)
        var += pow(x[i] - xm, 2);
    var /= n;
    return var;
}

template <typename T>
float Std(T x){
    double std =0;
    std = sqrt(Variance(x));
    return std;
}

template <typename T>
float DegToRad(T deg)
{
    float rad = 2*M_PI / 360 * (float)deg;
    return rad;
}

template <typename T>
float RadToDeg(T rad)
{
    float deg = (float)rad * 180 / M_PI;
    return deg;
}

//-----------
// Common_math CLASS
//-----------

class Common_math
{
public:

    /* Default Constructor */
    Common_math();

    /* Default Destructor */
    ~Common_math();

    /**
     *
     */
    Eigen::VectorXd degreesToRadians( Eigen::VectorXd );

    /**
     *
     */
    Eigen::VectorXd radiansToDegrees( Eigen::VectorXd );

    /**
     *
     */
    Eigen::MatrixXd truncate( Eigen::MatrixXd, int );

    /**
     *
     */
    Eigen::MatrixXd precision_Matrix( Eigen::MatrixXd, double dTol = 1e-14);

    /**
     *
     */
    Eigen::VectorXd precision_Vector( Eigen::VectorXd, double dTol = 1e-14);

    /**
     *
     */
    Eigen::Matrix3d Rx( double, bool bDegrees = false );
    Eigen::Matrix3d Ry( double, bool bDegrees = false );
    Eigen::Matrix3d Rz( double, bool bDegrees = false );

    /**
     *
     */
    Eigen::Matrix3d skew( Eigen::Vector3d );

    /**
     *
     */
    Eigen::Matrix4d makeDH_matrix( double dAlpha, double A, double d, double dTheta );

    /**
     *
     */
    Eigen::MatrixXd pseudo_Matrix(Eigen::MatrixXd , double dTol = 1e-14 );

    /**
     *
     */
    bool null_Matrix(Eigen::MatrixXd , Eigen::MatrixXd &result, double dTol = 0);

};

#endif

