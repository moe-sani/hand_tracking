/**
 * @file    inv_kin_soln.hpp
 * @author  Nikhil Deshpande
 * @version 1.0
 * @date    01-Aug-2017
 * @brief   Inv Kinematic Solutions
 */

#ifndef INV_KIN_SOLN_HPP
#define INV_KIN_SOLN_HPP

#include <ur5_microphone/UR5_control.hpp>
#include <iostream>


//-----------
// Inv_kin_soln CLASS
//-----------


class Inv_kin_soln
{
public:
    /* Default Constructor */
    Inv_kin_soln();

    /* Default Destructor */
    ~Inv_kin_soln();

    /*!
     * \brief theta contains the joint values
     */
    double dTheta[JOINTS_UR5];
};

#endif
