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
  bool findObject = false;
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
  findObject = msg->tag.getAttValue("findobject", value, MVL);

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
  else if (findObject)
  {
    data = getScan(msg, (ULaserData *)extra);

    // Get laser data
    if (data->isValid())
    {
      vector<double> poseR, poseW;
      poseR.push_back(xo);
      poseR.push_back(yo);
      poseR.push_back(tho);
      poseW = transform(poseR);


      //vector<double> r;
      //vector<double> th;
      vector<double> x;
      vector<double> y;
      for (int i = 0; i < data->getRangeCnt(); i++)
      {
        double range = data->getRangeMeter(i);
        double angle = data->getAngleRad(i);
        double xx = cos(angle) * range;
        double yy = sin(angle) * range;
        transform(poseW, xx, yy);
        if (xx >= 0.95 && xx <= 3.05 && yy >= 0.95 && yy <= 2.05)
        {
            //r.push_back(range);
            //th.push_back(angle);
            x.push_back(xx);
            y.push_back(yy);
        }
      }

      printf("x:\n");
      printVec(x);
      printf("y:\n");
      printVec(y);
      printf("\n");

      double xmean = accumulate(x.begin(), x.end(), 0.0) / x.size();
      double ymean = accumulate(y.begin(), y.end(), 0.0) / y.size();

      printf("Approximate position (x,y):\t\t(%.2f,%.2f)\n", xmean, ymean);

      /* SMRCL reply format */
      snprintf(reply, MRL, "<laser l0=\"%g\" l1=\"%g\" />\n", xmean, ymean);
      // send this string as the reply to the client
      sendMsg(msg, reply);
    }
  }
  else if (detectObject)
  {
    data = getScan(msg, (ULaserData *)extra);

    // Get laser data
    if (data->isValid())
    {
      vector<double> poseR, poseW;
      poseR.push_back(xo);
      poseR.push_back(yo);
      poseR.push_back(tho);
      poseW = transform(poseR);


      //vector<double> r;
      //vector<double> th;
      vector<double> x;
      vector<double> y;
      for (int i = 0; i < data->getRangeCnt(); i++)
      {
        double range = data->getRangeMeter(i);
        double angle = data->getAngleRad(i);
        double xx = cos(angle) * range;
        double yy = sin(angle) * range;
        transform(poseW, xx, yy);
        if (xx >= 0.95 && xx <= 3.05 && yy >= 0.95 && yy <= 2.05)
        {
            //r.push_back(range);
            //th.push_back(angle);
            x.push_back(cos(angle)*range);
            y.push_back(sin(angle)*range);
        }
      }

      // Transform to world coordinates
      vector<vector<double>> lines;
      bool state = DoLsqLineProcessing(x, y, lines);

      if (state)
      {
        vector<vector<double>> lineW;
        for (uint i = 0; i < lines.size(); i++)
        {
          lineW.push_back(transline(lines[i], poseW));
        }

        for (uint i = 0; i < lineW.size(); i++)
        {
          goodLineFitsWorldCoordinates.push_back(lineW[i]);
        }

        printf("\nRobot pose in world:\t(%.2f,%.2f,%.2f)\n", poseR[0], poseR[1], poseR[2]);
        printf("Laser pose in world:\t(%.2f,%.2f,%.2f)\n", poseW[0], poseW[1], poseW[2]);

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
    if (goodLineFitsWorldCoordinates.size() < 3)
    {
      printf("Not enough values to determine object!\n");
      /* SMRCL reply format */
      snprintf(reply, MRL, "<laser l0=\"%d\" l1=\"%g\" l2=\"%g\" l3=\"%g\" />\n",
               0, 0.0, 0.0, 0.0);
      // send this string as the reply to the client
      sendMsg(msg, reply);
    }
    else
    {
      int object;
      vector<double> pointO;
      double objectPose;
      bool FoundObject = DoObjectProcessing(goodLineFitsWorldCoordinates, object, pointO, objectPose);
      if (FoundObject)
      {
        printf("\n\nRESULT:\n");
        printf("Object:\t\t\t%d\n", object);
        printf("Point o coordinates:\t(%.2f, %.2f)\n", pointO[0], pointO[1]);
        printf("Object pose:\t\t%.2f\n\n", objectPose);
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
    }
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

bool UFunczoneobst::DoObjectProcessing(vector<vector<double>> &v, int &object, vector<double> &pointO, double &objectPose)
{
  /*
  * INPUT:  vector<vector<double>> &v: address of 2-D vector containing alhpa,r parameters of good line fits
  *         int &ojbect: address of integer value of the detected object (1 to 4)
  *         vector<double> &pointO: address vector containing the coordinates of point o
  *         double &objectPose: address of double containin the pose of the object
  * 
  * OUTPUT:
  *         bool value stating if the function is successful or not
  * */
  RemoveDuplicates(v);
  printf("\nFinal line parameters (world):\n");
  printMat(v);

  if (v.size() > 1)
  {
    vector<vector<double>> resultXY;
    resultXY = GetIntersectionMatrix(v); // extract X,Y values of all line intersections

    printf("\nIntersections (x,y):\n");
    RemoveDuplicates(resultXY);
    printMat(resultXY);

    bool FoundObject = DetermineObject(resultXY, object, pointO, objectPose, v);

    return FoundObject;
  }
  else
  {
    printf("Not enough lines!\n");
  }

  return false;
}

bool UFunczoneobst::DetermineObject(vector<vector<double>> v, int &object, vector<double> &pointO, double &objectPose, vector<vector<double>> lineMat)
{
  /*
  * INPUT:  vector<vector<double>> v: X,Y coordinates of intersection between lines
  *         int &ojbect: address of integer value of the detected object (1 to 4)
  *         vector<double> &pointO: address vector containing the coordinates of point o
  *         double &objectPose: address of double containin the pose of the object
  * 
  * OUTPUT:
  *         bool value stating if the function is successful or not
  * */

  object = 0;
  pointO.push_back(0);
  pointO.push_back(0);
  objectPose = 0;

  if (v.size() == 4)
  { // assume square
    // lengths of possible squares
    vector<double> obj1 = {0.15, 0.40};
    vector<double> obj2 = {0.20, 0.30};

    vector<double> lengths;
    for (uint i = 0; i < v.size() - 1; i++)
    {
      for (uint j = i + 1; j < v.size(); j++)
      {
        lengths.push_back(CalcDistanceBetweenPoints(v[i], v[j]));
      }
    }
    sort(lengths.begin(), lengths.end());
    lengths.pop_back(); //delete diagonal
    lengths.pop_back(); //delete diagonal

    double temp1, temp2;
    temp1 = accumulate(lengths.begin(), lengths.begin() + int(lengths.size() / 2), 0.0) / 2.0;
    temp2 = accumulate(lengths.begin() + int(lengths.size() / 2), lengths.end(), 0.0) / 2.0;

    lengths.pop_back();
    lengths.pop_back();
    lengths[0] = temp1;
    lengths[1] = temp2;

    printVec(lengths);

    if (CalcSSD(lengths, obj1) < CalcSSD(lengths, obj2))
    {
      object = 1;
    }
    else
    {
      object = 2;
    }

    // find point o and pose
    bool foundPointO = FindPointOAndPoseSquare(lineMat, v, pointO, objectPose);
    if (not foundPointO)
    {
      printf("No point o found!\n");
    }
  }
  else if (v.size() == 3)
  { // assume triangle
    // lengths of possible triangles
    vector<double> obj3 = {0.10, 0.40};
    vector<double> obj4 = {0.15, 0.3};

    vector<double> lengths;
    for (uint i = 0; i < v.size() - 1; i++)
    {
      for (uint j = i + 1; j < v.size(); j++)
      {
        lengths.push_back(CalcDistanceBetweenPoints(v[i], v[j]));
      }
    }
    sort(lengths.begin(), lengths.end());
    lengths.pop_back();

    // SSD between the measured lengths and the triangles
    if (CalcSSD(lengths, obj3) < CalcSSD(lengths, obj4))
    {
      object = 3;
    }
    else
    {
      object = 4;
    }

    // find point o and pose
    bool foundPointO = FindPointOAndPoseTriangle(lineMat, v, pointO, objectPose);
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

bool UFunczoneobst::FindPointOAndPoseSquare(vector<vector<double>> v, vector<vector<double>> matXY, vector<double> &point, double &objectPose)
{
  // vector to store initial point to calculate pose
  vector<double> point1 = {0, 0};
  double smallest = 10000;

  // point o is average of all X,Y coords
  for (uint i = 0; i < matXY.size(); i++)
  {
    point[0] += matXY[i][0] / (double)matXY.size();
    point[1] += matXY[i][1] / (double)matXY.size();

    if (CalcDistToPoint(matXY[i]) < smallest)
    {
      smallest = CalcDistToPoint(matXY[i]);
      point1[0] = matXY[i][0];
      point1[1] = matXY[i][1];
    }
  }

  // find biggest side
  int idx1 = 0;
  int idx2 = 0;
  double biggest = 0;
  double dist;
  for (uint k = 0; k < matXY.size(); k++)
  {
    dist = CalcDistanceBetweenPoints(point1, matXY[k]);
    if (dist > biggest)
    {
      biggest = dist;
      idx1 = (int)k;
    }
  }
  biggest = 0;
  for (uint k = 0; k < matXY.size(); k++)
  {
    dist = CalcDistanceBetweenPoints(point1, matXY[k]);
    if (dist > biggest && (int)k != idx1)
    {
      biggest = dist;
      idx2 = (int)k;
    }
  }

  objectPose = atan2(matXY[idx2][1] - point1[1], matXY[idx2][0] - point1[0]);

  return true;
}

double UFunczoneobst::CalcDistToPoint(vector<double> a)
{
  return sqrt(pow(a[0], 2) + pow(a[1], 2) * 1.0);
}

bool UFunczoneobst::FindPointOAndPoseTriangle(vector<vector<double>> v, vector<vector<double>> matXY, vector<double> &point, double &objectPose)
{

  for (uint i = 0; i < v.size() - 1; i++)
  {
    for (uint j = i + 1; j < v.size(); j++)
    {

      double a1 = v[i][0];
      double a2 = v[j][0];

      double angle = atan2(cos(a2), -sin(a2)) - atan2(cos(a1), -sin(a1));

      if (angle > PI)
      {
        angle -= 2 * PI;
      }
      else if (angle <= -PI)
      {
        angle += 2 * PI;
      }

      if (abs(PI / 2 - abs(angle)) < 0.1)
      {
        vector<double> temp = FindIntersection(v[i], v[j]);
        point[0] = temp[0];
        point[1] = temp[1];

        // find biggest side
        int idx;
        double biggest = 0;
        double dist;
        for (uint k = 0; k < v.size(); k++)
        {
          dist = CalcDistanceBetweenPoints(point, matXY[k]);
          if (dist > biggest)
          {
            biggest = dist;
            idx = (int)k;
          }
        }
        objectPose = atan2(matXY[idx][1] - point[1], matXY[idx][0] - point[0]);

        return true;
      }
    }
  }
  return false;
}

double UFunczoneobst::CalcSSD(vector<double> a, vector<double> b)
{
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

double UFunczoneobst::CalcDistanceBetweenPoints(vector<double> p1, vector<double> p2)
{
  return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2) * 1.0);
}

vector<vector<double>> UFunczoneobst::GetIntersectionMatrix(vector<vector<double>> v)
{
  vector<vector<double>> intersections;
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

vector<double> UFunczoneobst::FindIntersection(vector<double> u, vector<double> v)
{
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
      if (abs(a - v[j][0]) < 0.08 && abs(r - v[j][1]) < 0.08) //assume duplicate if both alpha and r are within 0.08 of eachother
      {
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

bool UFunczoneobst::DoLsqLineProcessing(vector<double> x, vector<double> y, vector<vector<double>> &lines)
{
  int n = x.size();
  int parts = 7;
  int delta = n / parts;
  int matches = 0;

  if (n > parts * 2)
  {
    vector<vector<double>> lineMat; //, lineMatCopy;

    // extract line segments and do least squares estimation for each segment
    for (int i = 0; i < parts; i++)
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
    for (int i = 0; i < parts; i++)
    {
      lineMat[i][0] = round(lineMat[i][0]);
      lineMat[i][1] = round(lineMat[i][1]);
    }

    cout << endl;
    for (int i = 0; i < parts; i++)
    {
      printf("Line %d:\t\talpha=%f\tr=%f\n", i, lineMat[i][0], lineMat[i][1]);
      // Count occurences of lineMat[i]
      /*matches = count(lineMat.begin(), lineMat.end(), lineMat[i]);
      //printf("Matches: %d\n", matches)

      if (matches > 1)
      {
        lines.push_back(lineMatCopy[i]);
      }*/

      matches = 0;
      for(int j = 0; j < parts; j++){
        if (abs(lineMat[j][0] - lineMat[i][0]) < 0.03 && abs(lineMat[j][1] - lineMat[i][1]) < 0.03)
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
  return false;
}

vector<double> UFunczoneobst::lsqline(vector<double> x, vector<double> y)
{
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

  vector<double> line;
  line.push_back(a);
  line.push_back(r);

  return line;
}

vector<double> UFunczoneobst::transline(vector<double> lineL, vector<double> poseW)
{
  /*
  * lineL = [aL, rL]
  * poseW = [xL, yL, thL]^W
  */

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
  lineW.push_back(a);
  lineW.push_back(r);

  return lineW;
}

void UFunczoneobst::transform(vector<double> pose, double &x, double &y)
{
  double th_lw = pose[2];

  double tempx = cos(th_lw)*x - sin(th_lw)*y + pose[0];
  double tempy = sin(th_lw)*x + cos(th_lw)*y + pose[1];

  x = tempx;
  y = tempy;
}

vector<double> UFunczoneobst::transform(vector<double> poseR)
{
  vector<double> poseW;

  poseW.push_back(cos(poseR[2]) * 0.26 + poseR[0]);
  poseW.push_back(sin(poseR[2]) * 0.26 + poseR[1]);
  poseW.push_back(poseR[2]);

  return poseW;
}

void UFunczoneobst::printVec(vector<double> &result)
{
  for (unsigned int i = 0; i < result.size(); i++)
  {
    cout << result.at(i) << ' ';
  }
  cout << endl;
}

void UFunczoneobst::printMat(vector<vector<double>> &result)
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
  // 37.66666 * 100 =3766.66
  // 3766.66 + .5 =3767.16    for rounding off value
  // then type cast to int so value is 3767
  // then divided by 100 so the value converted into 37.67
  float value = (int)(var * 100 + .5);
  return (float)value / 100;
}