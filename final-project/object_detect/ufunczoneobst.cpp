/***************************************************************************
 *   Copyright (C) 2005 by Christian Andersen and DTU                      *
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
#include "ufunczoneobst.h"

using namespace std;

#ifdef LIBRARY_OPEN_NEEDED

/**
 * This function is needed by the server to create a version of this plugin */
UFunctionBase *createFunc()
{ // create an object of this type
  /** replace 'UFunczoneobst' with your class name */
  return new UFunczoneobst();
}
#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
// #define SMRWIDTH 0.4
bool UFunczoneobst::handleCommand(UServerInMsg *msg, void *extra)
{ // handle a plugin command
  const int MRL = 500;
  char reply[MRL];
  bool ask4help;
  bool detectObject = false;
  bool determineObject = false;
  const int MVL = 30;
  char val[MRL];
  char value[MVL];
  vector<int> g1;
  ULaserData *data;
  //
  double xo = 0;
  double yo = 0;
  double tho = 0;
  //
  int i, j, imax;
  double r, delta;
  double minRange; // min range in meter
  // double minAngle = 0.0; // degrees
  // double d,robotwidth;
  double zone[9];
  // check for parameters - one parameter is tested for - 'help'
  ask4help = msg->tag.getAttValue("help", value, MVL);
  detectObject = msg->tag.getAttValue("detect", value, MVL);
  determineObject = msg->tag.getAttValue("determine", value, MVL);

  if (msg->tag.getAttValue("x", val, MVL))
  {
    xo = strtod(val, NULL);
  }
  if (msg->tag.getAttValue("y", val, MVL))
  {
    yo = strtod(val, NULL);
  }
  if (msg->tag.getAttValue("th", val, MVL))
  {
    tho = strtod(val, NULL);
  }

  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart(msg, "zoneobst");
    sendText("--- available zoneobst options\n");
    sendText("help            This message\n");
    sendText("fake=F          Fake some data 1=random, 2-4 a fake corridor\n");
    sendText("device=N        Laser device to use (see: SCANGET help)\n");
    sendText("see also: SCANGET and SCANSET\n");
    sendHelpDone();
  }
  else if (detectObject)
  {
    // Get laser data
    data = getScan(msg, (ULaserData *)extra);

    // check if data is valid
    if (data->isValid())
    {
      // create vector with robot pose (x,y,th) in world given by the caller
      vector<double> poseW;
      poseW.reserve(3);
      poseW.push_back(xo);
      poseW.push_back(yo);
      poseW.push_back(tho);
      // find the pose of the laser scanner in world
      transform(poseW);

      // vectors to store x,y data
      vector<double> x;
      vector<double> y;

      // loop over each data point and only store good values that fall within the "green area"
      for (int i = 0; i < data->getRangeCnt(); i++)
      {
        double range = data->getRangeMeter(i);
        double angle = data->getAngleRad(i);

        // x,y data in the laser frame
        double xx = cos(angle) * range;
        double yy = sin(angle) * range;
        // transform to world frame for test
        transform(poseW, xx, yy);
        if (xx >= 0.95 && xx <= 3.05 && yy >= 0.95 && yy <= 2.05 && range > 0.03)
        {
            // the data is saved in the laser frame
            x.push_back(cos(angle)*range);
            y.push_back(sin(angle)*range);
        }
      }

      /*printf("x:\n");
      printVec(x);
      printf("y:\n");
      printVec(y);
      printf("\n");*/

      // find good line fits and store in 2-D vector lines
      vector<vector<double>> lines;
      bool state = DoLsqLineProcessing(x, y, lines);

      // state=TRUE if one or more good line fits has been found
      if (state)
      {
        // transform lines to world frame and store in 2-D vector lineW
        vector<vector<double>> lineW;
        for (uint i = 0; i < lines.size(); i++)
        {
          lineW.push_back(transline(lines[i], poseW));
        }

        // store world lines in the global variable "goodLineFitsWorldCoordinates"
        for (uint i = 0; i < lineW.size(); i++)
        {
          goodLineFitsWorldCoordinates.push_back(lineW[i]);
        }

        // print result to laser server
        printf("\nLaser pose in world:\t(%.2f,%.2f,%.2f)\n", poseW[0], poseW[1], poseW[2]);

        printf("Line parameters (world):\n");
        printMat(goodLineFitsWorldCoordinates);
        cout << endl;
      }
      else
      {
        printf("No good lines found!\n");
      }
      
    }
  }
  else if (determineObject)
  {
    // Define variables to store information on the object in question
    // Initialize values in the case that no object is found.
    int object = 0;
    vector<double> pointO = {0,0};
    double objectPose = 0;
    double objectSSD = 1000;

    // remove duplicate line parameters and print what was found
    RemoveDuplicates(goodLineFitsWorldCoordinates);
    printf("\nFinal line parameters (world):\n");
    printMat(goodLineFitsWorldCoordinates);
    
    // perform object processing
    bool FoundObject = DoObjectProcessing(goodLineFitsWorldCoordinates, object, pointO, objectPose, objectSSD);

    vector<vector<double>> newGoodLines; // variable to store reduced goodLineFits matrix
    uint itr = 0;
    while(itr < goodLineFitsWorldCoordinates.size() && goodLineFitsWorldCoordinates.size() > 3){
      if (objectSSD > 0.0005){
        pointO = {0,0}; // make sure to reset the point o location

        newGoodLines = goodLineFitsWorldCoordinates;
        newGoodLines.erase(newGoodLines.begin()+itr); // delete element
        FoundObject = DoObjectProcessing(newGoodLines, object, pointO, objectPose, objectSSD);
      }
      else{
        break;
      }
      itr++;
    }

    if (FoundObject)
    {
      // print result
      printf("\n\nRESULT:\n");
      printf("Object:\t\t\t\t%d\n", object);
      printf("Object SSD:\t\t\t%f\n", objectSSD);
      printf("Point o coordinates (x,y):\t(%.2f, %.2f)\n", pointO[0], pointO[1]);
      printf("Object pose:\t\t\t%.2f rad\n\n", objectPose);
    }
    else
    {
      printf("\nObject not found!\n");
    }

    /* SMRCL reply format */
    snprintf(reply, MRL, "<laser l0=\"%d\" l1=\"%g\" l2=\"%g\" l3=\"%g\" />\n",
              object, pointO[0], pointO[1], objectPose);
    // send this string as the reply to the client
    sendMsg(msg, reply);

    // write the result to a file for statistics
    WriteResult2File(object, pointO, objectPose, objectSSD);
  }
  else
  { // do some action and send a reply
    data = getScan(msg, (ULaserData *)extra);
    //
    if (data->isValid())
    {                  // make analysis for closest measurement
      minRange = 1000; // long range in meters
      imax = data->getRangeCnt();
      delta = imax / 9.0;
      for (j = 0; j < 9; j++)
        zone[j] = minRange;
      for (j = 0; j < 9; j++)
      {
        for (i = 0 + (int)(j * delta); i < (int)((j + 1) * delta); i++)
        { // range are stored as an integer in current units
          r = data->getRangeMeter(i);
          if (r >= 0.020)
          { // less than 20 units is a flag value for URG scanner
            if (r < zone[j])
              zone[j] = r;
          }
        }
      }
      /* SMRCL reply format */
      snprintf(reply, MRL, "<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" l3=\"%g\" l4=\"%g\" "
                           "l5=\"%g\" l6=\"%g\" l7=\"%g\" l8=\"%g\" />\n",
               zone[0], zone[1], zone[2], zone[3], zone[4],
               zone[5], zone[6], zone[7], zone[8]);
      // send this string as the reply to the client
      sendMsg(msg, reply);
      // save also as gloabl variable
      for (i = 0; i < 9; i++)
        var_zone->setValued(zone[i], i);
    }
    else
      sendWarning(msg, "No scandata available");
  }
  // return true if the function is handled with a positive result
  // used if scanpush or push has a count of positive results
  return true;
}

void UFunczoneobst::createBaseVar()
{ // add also a global variable (global on laser scanner server) with latest data
  var_zone = addVarA("zone", "0 0 0 0 0 0 0 0 0", "d", "Value of each laser zone. Updated by zoneobst.");
}

/*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
* * * * * * * * * * * * * * OUR FUNCTIONS * * * * * * * * * * * * * * * * * *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*/

bool UFunczoneobst::DoObjectProcessing(vector<vector<double>> &goodLines, int &object, vector<double> &pointO, double &objectPose, double &objectSSD)
{
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

  // we need atleast three lines for anything useful
  if (goodLines.size() > 2)
  {
    
    // find the intersections between the lines
    vector<vector<double>> intersectionsXY;
    intersectionsXY = GetIntersectionMatrix(goodLines); // extract X,Y values of all line intersections

    // remove the duplicates again and print result
    printf("\nIntersections (x,y):\n");
    RemoveDuplicates(intersectionsXY);
    printMat(intersectionsXY);

    // determine the data for the object
    bool FoundObject = DetermineObject(goodLines, intersectionsXY, object, pointO, objectPose, objectSSD);

    return FoundObject;
  }
  else
  {
    printf("Not enough lines!\n");
  }

  return false;
}

bool UFunczoneobst::DetermineObject(vector<vector<double>> &goodLines, vector<vector<double>> &intersectionsXY, int &object, vector<double> &pointO, double &objectPose, double &objectSSD)
{
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

  // If the laser scanner has found 4 intersections, we assume it is a rectangle
  if (intersectionsXY.size() == 4)
  { 
    // lengths of possible squares
    vector<double> obj1 = {0.15, 0.40};
    vector<double> obj2 = {0.20, 0.30};

    // calculate the distance between each point
    vector<double> lengths;
    for (uint i = 0; i < intersectionsXY.size() - 1; i++)
    {
      for (uint j = i + 1; j < intersectionsXY.size(); j++)
      {
        lengths.push_back(CalcDistanceBetweenPoints(intersectionsXY[i], intersectionsXY[j]));
      }
    }
    // sort lengths vector from smallest to biggest
    sort(lengths.begin(), lengths.end());
    lengths.pop_back(); //delete diagonal
    lengths.pop_back(); //delete diagonal

    // calculate mean of measurement of smallest and biggest side 
    double temp1, temp2;
    temp1 = accumulate(lengths.begin(), lengths.begin() + int(lengths.size() / 2), 0.0) / 2.0;
    temp2 = accumulate(lengths.begin() + int(lengths.size() / 2), lengths.end(), 0.0) / 2.0;

    // reduce to a 1x2 vector containing average of each measurement
    lengths.pop_back();
    lengths.pop_back();
    lengths[0] = temp1;
    lengths[1] = temp2;

    // printVec(lengths);

    // use SSD to determine which object we are dealing with
    if (CalcSSD(lengths, obj1) < CalcSSD(lengths, obj2))
    {
      objectSSD = CalcSSD(lengths, obj1);
      object = 1;
    }
    else
    {
      objectSSD = CalcSSD(lengths, obj2);
      object = 2;
    }

    // find point o and pose
    bool foundPointO = FindPointOAndPoseSquare(intersectionsXY, pointO, objectPose);
    if (not foundPointO)
    {
      printf("No point o found!\n");
    }
  }
  else if (intersectionsXY.size() == 3) // If the laser scanner has found 3 intersections, we assume it is a triangle
  {
    // lengths of possible triangles
    vector<double> obj3 = {0.10, 0.40};
    vector<double> obj4 = {0.15, 0.3};

    // calculate the distance between each point
    vector<double> lengths;
    for (uint i = 0; i < intersectionsXY.size() - 1; i++)
    {
      for (uint j = i + 1; j < intersectionsXY.size(); j++)
      {
        lengths.push_back(CalcDistanceBetweenPoints(intersectionsXY[i], intersectionsXY[j]));
      }
    }
    // sort lengths vector from smallest to biggest
    sort(lengths.begin(), lengths.end());
    lengths.pop_back(); // remove biggest value (hypotenuse)

    // SSD between the measured lengths and the triangles
    if (CalcSSD(lengths, obj3) < CalcSSD(lengths, obj4))
    {
      objectSSD = CalcSSD(lengths, obj3);
      object = 3;
    }
    else
    {
      objectSSD = CalcSSD(lengths, obj4);
      object = 4;
    }

    // find point o and pose
    bool foundPointO = FindPointOAndPoseTriangle(goodLines, intersectionsXY, pointO, objectPose);
    if (not foundPointO)
    {
      printf("No point o found!\n");
    }
  }
  else
  { // not enough points
    return false;
  }

  return true;
}

bool UFunczoneobst::FindPointOAndPoseSquare(const vector<vector<double>>& intersectionsXY, vector<double> &point, double &objectPose)
{
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
  // vector to store initial point to calculate pose

  vector<double> point1 = {0, 0};
  double smallest = 10000;

  // point o is average of all X,Y coords
  for (uint i = 0; i < intersectionsXY.size(); i++)
  {
    point[0] += intersectionsXY[i][0] / (double)intersectionsXY.size();
    point[1] += intersectionsXY[i][1] / (double)intersectionsXY.size();

    // find the point which is closest to origo to use as "starting" point for the heading calculation
    if (CalcDistToPoint(intersectionsXY[i]) < smallest)
    {
      smallest = CalcDistToPoint(intersectionsXY[i]);
      point1[0] = intersectionsXY[i][0];
      point1[1] = intersectionsXY[i][1];
    }
  }

  // find biggest side
  // this must be done twice as the furthest away point corresponds to the diagonal element
  int idx1 = 0;
  int idx2 = 0;
  double biggest = 0;
  double dist;
  for (uint k = 0; k < intersectionsXY.size(); k++)
  {
    dist = CalcDistanceBetweenPoints(point1, intersectionsXY[k]);
    if (dist > biggest)
    {
      biggest = dist;
      idx1 = (int)k;
    }
  }
  biggest = 0;
  for (uint k = 0; k < intersectionsXY.size(); k++)
  {
    dist = CalcDistanceBetweenPoints(point1, intersectionsXY[k]);
    if (dist > biggest && (int)k != idx1)
    {
      biggest = dist;
      idx2 = (int)k;
    }
  }

  // the object heading is then found as the angle of the vector spanning from the previously found point to its furthest away point
  objectPose = atan2(intersectionsXY[idx2][1] - point1[1], intersectionsXY[idx2][0] - point1[0]);

  return true;
}

double UFunczoneobst::CalcDistToPoint(vector<double> a)
{
    /*
  * INPUT:  vector<double> a: 1-D vector of doubles describing a point location.
  * 
  * OUTPUT:
  *         double value giving the distance from Origo to the point described by a.
  * */
  return sqrt(pow(a[0], 2) + pow(a[1], 2) * 1.0);
}

bool UFunczoneobst::FindPointOAndPoseTriangle(const vector<vector<double>>& goodLines, const vector<vector<double>>& intersectionsXY, vector<double> &point, double &objectPose)
{
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

  for (uint i = 0; i < goodLines.size() - 1; i++)
  {
    for (uint j = i + 1; j < goodLines.size(); j++)
    {

      // find the angle between two lines
      double a1 = goodLines[i][0];
      double a2 = goodLines[j][0];
      double angle = atan2(cos(a2), -sin(a2)) - atan2(cos(a1), -sin(a1));

      // normalize to ]-pi;pi]
      if (angle > PI)
      {
        angle -= 2 * PI;
      }
      else if (angle <= -PI)
      {
        angle += 2 * PI;
      }

      // if it is a right angle, we know where we are on the triangle
      if (abs(PI / 2 - abs(angle)) < 0.1)
      {
        // the intersection between these lines must be point o
        vector<double> temp = FindIntersection(goodLines[i], goodLines[j]);
        point[0] = temp[0];
        point[1] = temp[1];

        // find biggest side as this is parallel to the heading
        int idx;
        double biggest = 0;
        double dist;
        for (uint k = 0; k < intersectionsXY.size(); k++)
        {
          dist = CalcDistanceBetweenPoints(point, intersectionsXY[k]);
          if (dist > biggest)
          {
            biggest = dist;
            idx = (int)k;
          }
        }
        // the object heading is then found as the angle of the vector spanning from point o to the furthest away point
        objectPose = atan2(intersectionsXY[idx][1] - point[1], intersectionsXY[idx][0] - point[0]);

        return true;
      }
    }
  }
  return false;
}

double UFunczoneobst::CalcSSD(const vector<double>& a, const vector<double>& b)
{
  /*
  * INPUT:  vector<double> a: 1xN vector of doubles
  *         vector<double> b: 1xN vector of doubles
  * 
  * OUTPUT:
  *         double value giving the SSD measure between vector a and b 
  * */
  double SSD = 0.0;

  if (a.size() != b.size())
  {
    printf("Vectors not same length!\n");
    return SSD;
  }

  for (uint i = 0; i < a.size(); i++)
  {
    SSD += pow(a[i] - b[i], 2);
  }
  return SSD;
}

double UFunczoneobst::CalcDistanceBetweenPoints(const vector<double>& p1, const vector<double>& p2)
{
  /*
  * INPUT:  vector<double> p1: 1-D vector of doubles describing a point location
  *         vector<double> p2: 1-D vector of doubles describing a point location
  * 
  * OUTPUT:
  *         double value giving the euclidian distance between point p1 and p2
  * */
  return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2) * 1.0);
}

vector<vector<double>> UFunczoneobst::GetIntersectionMatrix(const vector<vector<double>> &v)
{
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
  
  // preallocate memory
  vector<vector<double>> intersections;
  intersections.reserve(v.size()-1);
  
  for (uint i = 0; i < v.size() - 1; i++)
  {
    for (uint j = i + 1; j < v.size(); j++)
    {
      vector<double> temp = FindIntersection(v[i], v[j]);
      // perform quality check to make sure the line intersection
      // is within the green area
      if (temp[0] > 0.95 && temp[0] < 3.05 && temp[1] > 0.95 && temp[1] < 2.05)
      {
        intersections.push_back(temp);
      }
    }
  }
  return intersections;
}

vector<double> UFunczoneobst::FindIntersection(const vector<double> &u, const vector<double> &v)
{
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
  double a1 = u[0];
  double r1 = u[1];
  double a2 = v[0];
  double r2 = v[1];

  double t, s;

  t = (-r1 * sin(a1) * sin(a2) - cos(a2) * cos(a1) * r1 + r2) / (cos(a1) * sin(a2) - sin(a1) * cos(a2));
  s = (sin(a2) * sin(a1) * r2 + cos(a2) * cos(a1) * r2 - r1) / (cos(a1) * sin(a2) - sin(a1) * cos(a2));

  double x1, x2, y1, y2;

  x1 = r1 * cos(a1) + t * (-sin(a1));
  x2 = r2 * cos(a2) + s * (-sin(a2));
  y1 = r1 * sin(a1) + t * cos(a1);
  y2 = r2 * sin(a2) + s * cos(a2);

  vector<double> intersectionXY = {(x1 + x2) / 2, (y1 + y2) / 2};

  return intersectionXY;
}

void UFunczoneobst::RemoveDuplicates(vector<vector<double>> &v)
{
  /*
  * This function removes potential duplicates in a 2xN matrix. Elements are considered duplicates if 
  * both values in a row are within >0.08 of another row. In this case, the average is stored and the duplicate
  * element is deleted.
  *  
  * INPUT:  vector<vector<double>> &v: address of 2xN vector where each row is a pair of associated parameters
  * 
  * */

  uint itr = 0;
  while (true)
  {
    double a = v[itr][0];
    double r = v[itr][1];

    uint j = v.size() - 1;
    double kk = 1.0;
    while (true)
    {
      if (j == itr)
      {
        break;
      }
      if (sqrt(pow((a - v[j][0]),2) + pow((r - v[j][1]),2)) < 0.07) //assume duplicate if euclidian distance < 0.07 
      { //(abs(a - v[j][0]) < 0.08 && abs(r - v[j][1]) < 0.08)
        // recursive average of the similar elements
        v[itr][0] = (v[itr][0]*kk+v[j][0])/(kk+1.0);
        v[itr][1] = (v[itr][1]*kk+v[j][1])/(kk+1.0);
        v.erase(v.begin() + j);
        kk = kk + 1.0;
      }

      j--;
    }

    itr++;

    if (itr == v.size())
    {
      break;
    }
  }
}

bool UFunczoneobst::DoLsqLineProcessing(const vector<double>& x, const vector<double>& y, vector<vector<double>> &lines)
{
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

  int n = x.size();
  vector<int> parts = {5,7,2,3};

  for (uint itr = 0; itr < parts.size(); itr++){
    int part = parts[itr];
    int delta = n / part;
    int matches = 0;

    cout << endl << "Attempt with parts = " << part << "..." << endl;
    if (n > part * 2)
    {
      vector<vector<double>> lineMat;
      lineMat.reserve(part);

      // extract line segments and do least squares estimation for each segment
      for (int i = 0; i < part; i++)
      {
        vector<double> tempX, tempY, tempL;
        for (int j = 0 + (int)(i * delta); j < (int)((i + 1) * delta); j++)
        {
          tempX.push_back(x[j]);
          tempY.push_back(y[j]);
        }

        /*printf("i = %d\n", i);
        printf("tempX:\n\t");
        printVec(tempX);
        printf("tempY:\n\t");
        printVec(tempY);*/

        lineMat.push_back(lsqline(tempX, tempY));
      }

      // copy of lineMat
      auto lineMatCopy(lineMat);

      // round numbers
      for (int i = 0; i < part; i++)
      {
        lineMat[i][0] = round(lineMat[i][0]);
        lineMat[i][1] = round(lineMat[i][1]);
      }

      for (int i = 0; i < part; i++)
      {
        printf("Line %d:\t\talpha=%f\tr=%f\n", i, lineMat[i][0], lineMat[i][1]);

        matches = 0;
        for(int j = 0; j < part; j++){
          if (abs(lineMat[j][0] - lineMat[i][0]) < 0.06 && abs(lineMat[j][1] - lineMat[i][1]) < 0.06)
          {
            matches++;
          }
        }

        if (matches > 1)
        {
          lines.push_back(lineMatCopy[i]);
        }

        
      }

      // return true if 1 or more line parameters have been added
      if (lines.size() > 0)
      {
        RemoveDuplicates(lines);
        return true;
      }
    }
    else{
      cout << "Not enough data points (" << n << ")." << endl;
    }
  }
  return false;
}

vector<double> UFunczoneobst::lsqline(const vector<double>& x, const vector<double>& y)
{
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

  int n = x.size();
  double xmean, ymean, sumx, sumy, sumx2, sumy2, sumxy;

  for (int j = 0; j < n; j++)
  {
    sumx += x[j];
    sumy += y[j];
  }

  xmean = sumx / (double)n;
  ymean = sumy / (double)n;

  sumx2 = 0;
  sumy2 = 0;
  sumxy = 0;
  for (int i = 0; i < n; i++)
  {
    sumx2 += x[i] * x[i];
    sumy2 += y[i] * y[i];
    sumxy += x[i] * y[i];
  }

  double a = 0.5 * atan2((2 * sumx * sumy - 2 * (double)n * sumxy), pow(sumx, 2) - pow(sumy, 2) - (double)n * sumx2 + (double)n * sumy2);
  double r = xmean * cos(a) + ymean * sin(a);

  if (r < 0)
  {
    r = abs(r);
    if (a < 0)
    {
      a += PI;
    }
    else
    {
      a -= PI;
    }
  }

  // make sure we are in ]-pi;pi]
  a = atan2(sin(a),cos(a));

  vector<double> line;
  line.reserve(2);
  line.push_back(a);
  line.push_back(r);

  return line;
}

vector<double> UFunczoneobst::transline(const vector<double>& lineL, const vector<double>& poseW)
{
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
 
  double a = lineL[0] + poseW[2];
  double r = lineL[1] + (cos(a) * poseW[0] + sin(a) * poseW[1]);

  if (r < 0)
  {
    r = abs(r);
    if (a < 0)
    {
      a += PI;
    }
    else
    {
      a -= PI;
    }
  }

  vector<double> lineW;
  lineW.reserve(2);
  lineW.push_back(a);
  lineW.push_back(r);

  return lineW;
}

void UFunczoneobst::transform(const vector<double>& pose, double &x, double &y)
{
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

  double th_lw = pose[2];

  double tempx = cos(th_lw)*x - sin(th_lw)*y + pose[0];
  double tempy = sin(th_lw)*x + cos(th_lw)*y + pose[1];

  x = tempx;
  y = tempy;
}

void UFunczoneobst::transform(vector<double> &pose)
{
  /*
  * This function finds the pose of the laser scanner in the world frame using the 
  * pose of the robot in the world frame.
  * The laser scanner is mounted on the robot 26 cm from the centre.
  * 
  * INPUT:  vector<double> pose: address of (x,y,th) pose of the robot in the world frame to be transformed to laser pose in world
  * 
  * */

  pose[0] = cos(pose[2]) * 0.26 + pose[0];
  pose[1] = sin(pose[2]) * 0.26 + pose[1];
}

void UFunczoneobst::printVec(const vector<double> &result)
{
  for (unsigned int i = 0; i < result.size(); i++)
  {
    cout << result.at(i) << ' ';
  }
  cout << endl;
}

void UFunczoneobst::printMat(const vector<vector<double>> &result)
{
  for (const vector<double> &v : result)
  {
    for (double x : v)
      cout << x << ' ';
    cout << endl;
  }
}

float UFunczoneobst::round(float var)
{
  /* Reference: https://www.geeksforgeeks.org/rounding-floating-point-number-two-decimal-places-c-c/
  *
  * 
  * 37.66666 * 100 =3766.66
  * 3766.66 + .5 =3767.16    for rounding off value
  * then type cast to int so value is 3767
  * then divided by 100 so the value converted into 37.67
  * 
  * */

  float value = (int)(var * 100 + .5);
  return (float)value / 100;
}


void UFunczoneobst::WriteResult2File(int object, vector<double> pointO, double objectPose, double objectSSD){
  
  // create string with result
  char str[100];
  snprintf(str, sizeof(str), "%d %.2f %.2f %.2f %f", object, pointO[0], pointO[1], objectPose, objectSSD);
  string Str = str;
  
  // file path
  const char *path="/home/smr/test/results_cpp.txt";

  // file to be read
  ifstream fin(path);    
  
  // temporary file to write to
  ofstream fout;                
  fout.open("/home/smr/test/temp.txt", ios::out);
  
  string lineold; // used to store newest line read from line
  string linenew; // stores the old line
  
  // the last line in results_cpp.txt is a line of "-1"s 
  // and should be excluded
  while(getline(fin, linenew)){
    if (!lineold.empty()){
        fout << lineold << endl;
    }
    lineold = linenew;
  }
  
  // append the result
  fout << Str << endl;
  
  fout.close();
  fin.close();
  
  remove(path);        
  rename("/home/smr/test/temp.txt", path);
}


/*
void UFunczoneobst::WriteResult2File(int object, vector<double> pointO, double objectPose, double objectSSD){
  // Creation of ofstream class object 
  ofstream fout; 
  
  // file path
  const char *path="/home/smr/test/results_cpp.txt";

  // flag to see if the file exists
  bool flag = false;
    
  // check if the file exists
  ifstream fin(path);
  if (fin.good())
  {
    flag = true;
  }

  // open file and append
  fout.open(path, ios::app);
  
  // Execute if file successfully opened 
  if (fout) {
    //
    if(not flag){
      fout << "Object | Point o x | Point o y | Object pose | SSD" << endl;
    }

    // create formatted string as C-string
    char str[100];
    snprintf(str, sizeof(str), "%d %.2f %.2f %.2f %f", object, pointO[0], pointO[1], objectPose, objectSSD);
    string Str = str;

    // write to file
    fout << Str << endl;
  }

  // Close the File 
  fout.close();
}*/