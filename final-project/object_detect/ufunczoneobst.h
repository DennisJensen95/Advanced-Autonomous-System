/***************************************************************************
 *   Copyright (C) 2005 by Christian Andersen and DTU                             *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef UFUNC_MRCOBST_H
#define UFUNC_MRCOBST_H

#include <cstdlib>
#include <vector>
#include <bits/stdc++.h> 
#include <ulms4/ufunclaserbase.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

#define PI 3.14159265
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
 * Laserscanner function to demonstrate
 * simple laser scanner data handling and analysis
 * @author Christian Andersen
*/
class UFunczoneobst : public UFuncLaserBase
{
public:
  /**
  Constructor */
  UFunczoneobst()
  { // set the command (or commands) handled by this plugin
    setCommand("zoneobst detect x y th determine", "zoneobstif", "obstacle detect for MRC (Compiled " __DATE__ " " __TIME__ ")");
    createBaseVar();
  }
  /**
  Handle incomming command
  (intended for command separation)
  Must return true if the function is handled -
  otherwise the client will get a failed - reply */
  virtual bool handleCommand(UServerInMsg * msg, void * extra);

  protected:
    void createBaseVar();
    UVariable *var_zone;
  
  private:
    // variable to store good line fits
    vector<vector<double>> goodLineFitsWorldCoordinates;

    /*
    * Prints the contents of a 1D vector to the console
    * */ 
    void printVec(vector<double> & result);
    
    /*
    * Prints the contents of a 2D vector to the console
    * */
    void printMat(vector<vector<double>> &result);

    /*
    * This function provides the structure to perform a robust least squares approximation of line segments found 
    * with a laser scan.
    * The function only detects lines within the "green area" marked on the map.
    * The function is able to detect multiple lines in this area and ignores potential "corners" of objects.
    *
    * INPUT:  vector<double> x: X values of a laser scan
    *         vector<double> y: Y values of a laser scan
    *         vector<vector<double>> &lines: address of 2-D vector to store alhpa,r parameters of good line fits
    *         
    * 
    * OUTPUT:
    *         bool value stating if the function is successful or not
    * */
    bool DoLsqLineProcessing(vector<double> x, vector<double> y, vector<vector<double>> &lines);
    
    /*
    * This function implements the core algorithm to do least squares approximation of
    * (x,y) data points to fit a line.
    * 
    * INPUT:  vector<double> x: X values of a laser scan
    *         vector<double> y: Y values of a laser scan
    * 
    * OUTPUT:
    *         vector<double>: 1-D vector containing alpha,r parameters of the line fit
    * */
    vector<double> lsqline(vector<double> x, vector<double> y);
    
    /*
    * This function transforms a line estimate from the laser frame to the world frame
    * using the pose of the laser in the world frame. 
    * 
    * INPUT:  vector<double> lineL: alpha,r parameters of a line in the laser frame
    *         vector<double> poseW: (x,y,th) pose of the laser scanner in the world frame
    * 
    * OUTPUT:
    *         vector<double>: alhpa,r parameters of the line in the world frame
    * */
    vector<double> transline(vector<double> lineL, vector <double> poseW);
    
    /*
    * This function finds the pose of the laser scanner in the world frame using the 
    * pose of the robot in the world frame.
    * The laser scanner is mounted on the robot 26 cm from the centre.
    * 
    * INPUT:  vector<double> pose: address of (x,y,th) pose of the robot in the world frame to be transformed to laser pose in world
    * 
    * */
    void transform(vector<double> &pose);
    
    /*
    * This function converts a set of (x,y) points from the laser frame to the 
    * world frame using the pose of the laser scanner in the world frame.
    * 
    * 
    * INPUT:  vector<double> pose: (x,y,th) pose of the laser scanner in the world frame
    *         double &x: address of X location of a point in the laser frame
    *         double &y: address of Y location of a point in the laser frame
    * 
    * */
    void transform(vector<double> pose, double &x, double &y);

    /* Reference: https://www.geeksforgeeks.org/rounding-floating-point-number-two-decimal-places-c-c/
    *
    * 
    * 37.66666 * 100 =3766.66
    * 3766.66 + .5 =3767.16    for rounding off value
    * then type cast to int so value is 3767
    * then divided by 100 so the value converted into 37.67
    * 
    * */
    float round(float var);

    /*
    * This function implements the top layer structure to determine which object that has been spotted by the laser scanner.
    * 
    * INPUT:  vector<vector<double>> &goodLines: address of 2-D vector containing alhpa,r parameters of good line fits
    *         int &ojbect: address of integer value of the detected object (1 to 4)
    *         vector<double> &pointO: address vector containing the coordinates of point o
    *         double &objectPose: address of double containin the pose of the object
    * 
    * OUTPUT:
    *         bool value stating if the function is successful or not
    * */
    bool DoObjectProcessing(vector<vector<double>> &goodLines, int &object, vector<double> &pointO, double &objectPose, double &objectSSD);
    
    /*
    * This function removes potential duplicates in a 2xN matrix. Elements are considered duplicates if 
    * the euclidean distance is >0.07 of another row. In this case, the average is stored and the duplicate
    * element is deleted.
    *  
    * INPUT:  vector<vector<double>> &v: address of 2xN vector where each row is a pair of associated parameters
    * 
    * */
    void RemoveDuplicates(vector<vector<double>> &v);

    /*
    * This function implements the core algorithm to find the (x,y) location of the intersection
    * between two line fits.
    * 
    * INPUT:  vector<double> u: 1-D vector containing alhpa,r parameters of a good line fit
    *         vector<double> v: 1-D vector containing alpha,r parameters of another good line fit
    * 
    * OUTPUT:
    *         vector<double>: (X,Y) location of the intersection between lines u and v
    * */
    vector<double> FindIntersection(vector<double> &u, vector<double> &v);

    /*
    * This function implements the structure to find a matrix with all intersections between all 
    * lines in a matrix v.
    * The function filters out intersections that are outside of the "green area" on the map.
    * 
    * 
    * INPUT:  vector<vector<double>> v: address of 2-D vector containing alhpa,r parameters of good line fits
    * 
    * OUTPUT:
    *         vector<vector<double>>: 2-D vector with (X,Y) location of the intersection between lines in v
    * */
    vector<vector<double>> GetIntersectionMatrix(vector<vector<double>> &v);

    /*
    * This function implements the underlying structure to determine which object that has been spotted by the laser scanner.
    * 
    * 
    * INPUT:  vector<vector<double>> goodLines: address of 2-D vector containing alhpa,r parameters of good line fits
    *         vector<vector<double>> intersectionsXY: address of X,Y coordinates of intersection between lines
    *         int &ojbect: address of integer value of the detected object (1 to 4)
    *         vector<double> &pointO: address vector containing the coordinates of point o
    *         double &objectPose: address of double containin the pose of the object
    * 
    * OUTPUT:
    *         bool value stating if the function is successful or not
    * */
    bool DetermineObject(vector<vector<double>> &goodLines, vector<vector<double>> &intersectionsXY, int &object, vector<double> &pointO, double &objectPose, double &objectSSD);
    
    /*
    * INPUT:  vector<double> p1: 1-D vector of doubles describing a point location
    *         vector<double> p2: 1-D vector of doubles describing a point location
    * 
    * OUTPUT:
    *         double value giving the euclidian distance between point p1 and p2
    * */
    double CalcDistanceBetweenPoints(vector<double> p1, vector<double> p2);

    /*
    * INPUT:  vector<double> a: 1xN vector of doubles
    *         vector<double> b: 1xN vector of doubles
    * 
    * OUTPUT:
    *         double value giving the SSD measure between vector a and b 
    * */
    double CalcSSD(vector<double> a, vector<double> b);

    /*
    * This function implements the basic structure to find the location of point o and the pose of a triangle.
    * 
    * INPUT:  vector<vector<double>> goodLines: 2-D vector containing alhpa,r parameters of good line fits
    *         vector<vector<double>> intersectionsXY: X,Y coordinates of intersection between lines
    *         vector<double> &point: address of vector containing the coordinates of point o
    *         double &objectPose: address of double containin the pose of the object
    * 
    * OUTPUT:
    *         bool value stating if the function is successful or not
    * */
    bool FindPointOAndPoseTriangle(vector<vector<double>> goodLines, vector<vector<double>> intersectionsXY, vector<double> &point, double &objectPose);

    /*
    * This function implements the basic structure to find the location of point o and the pose of a rectangle.
    * 
    * INPUT:  vector<vector<double>> intersectionsXY: X,Y coordinates of intersection between lines
    *         vector<double> &point: address of vector containing the coordinates of point o
    *         double &objectPose: address of double containin the pose of the object
    * 
    * OUTPUT:
    *         bool value stating if the function is successful or not
    * */
    bool FindPointOAndPoseSquare(vector<vector<double>> intersectionsXY, vector<double> &point, double &objectPose);

    /*
    * INPUT:  vector<double> a: 1-D vector of doubles describing a point location.
    * 
    * OUTPUT:
    *         double value giving the distance from Origo to the point described by a.
    * */
    double CalcDistToPoint(vector<double> a);

    /*
    * Create file with the result of the object detection
    * */
    void WriteResult2File(int object, vector<double> pointO, double objectPose, double objectSSD);
};



#endif

