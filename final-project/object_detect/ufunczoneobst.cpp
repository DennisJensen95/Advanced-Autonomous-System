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
UFunctionBase * createFunc()
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
bool UFunczoneobst::handleCommand(UServerInMsg * msg, void * extra)
{  // handle a plugin command
  const int MRL = 500;
  char reply[MRL];
  bool ask4help;
  bool detectObject = false;
  bool determineObject = false;
  const int MVL = 30;
  char val[MRL];
  char value[MVL];
  vector <int> g1;
  ULaserData * data;
  //
  double xo = 0;
  double yo = 0;
  double tho = 0;
  //
  int i,j,imax;
  double r,delta;
  double minRange; // min range in meter
  // double minAngle = 0.0; // degrees
  // double d,robotwidth;
  double zone[9];
  // check for parameters - one parameter is tested for - 'help'
  ask4help = msg->tag.getAttValue("help", value, MVL);
  detectObject = msg->tag.getAttValue("detect", value, MVL);
  determineObject = msg->tag.getAttValue("determine", value, MVL);

  if (msg->tag.getAttValue("x", val, MVL)) {
		xo = strtod(val, NULL);
	}
  if (msg->tag.getAttValue("y", val, MVL)) {
		yo = strtod(val, NULL);
	}
  if (msg->tag.getAttValue("th", val, MVL)) {
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
    data = getScan(msg, (ULaserData*)extra);

    // Get laser data 
    if (data->isValid()) 
    {
      vector<double> r;
      vector<double> th;
      vector<double> x;
      vector<double> y;
      int j = 0;
      for (int i = 0; i<data->getRangeCnt(); i++)
      {
        // printf("Dist: %.2f, AngleDeg: %.2f\n", )
        double range = data->getRangeMeter(i);
        double angle = data->getAngleRad(i);
        if (range > 0.020 && range < 1.500)
        {
          if(angle > -30*PI/180 && angle < 30*PI/180){
            r.push_back(range);
            th.push_back(data->getAngleRad(i));
            x.push_back(cos(th[j])*r[j]);
            y.push_back(sin(th[j])*r[j]);
            j++;
          }
        }
      }
      //printVec(r);

      // Transform to world coordinates
      vector<vector<double>> lines;
      bool state = DoLsqLineProcessing(x,y,lines);
      
      if (state){
	      vector<double> poseR, poseW;
	      poseR.push_back(xo);
	      poseR.push_back(yo);
	      poseR.push_back(tho);

	      poseW = transform(poseR);

	      vector<vector<double>> lineW;
        for (uint i = 0; i<lines.size(); i++){
          lineW.push_back(transline(lines[i], poseW));
        }

        for (uint i = 0; i<lineW.size(); i++){
          goodLineFitsWorldCoordinates.push_back(lineW[i]);
        }

	      printf("Robot pose in world:\t(%.2f,%.2f,%.2f)\n", poseR[0], poseR[1], poseR[2]);
	      printf("Laser pose in world:\t(%.2f,%.2f,%.2f)\n", poseW[0], poseW[1], poseW[2]);

	      printf("Line parameters (world):\n");
	      printMat(goodLineFitsWorldCoordinates);
      }
    }
  }
  else if(determineObject){
    if (goodLineFitsWorldCoordinates.size() == 0){
      printf("Not enough values to determine object!\n");
    }
    else{
      int object = DoObjectProcessing(goodLineFitsWorldCoordinates);
      printf("Object = %d\n", object);
    }
  }
  else
  { // do some action and send a reply
    data = getScan(msg, (ULaserData*)extra);
    //
    if (data->isValid())
    { // make analysis for closest measurement
      minRange = 1000; // long range in meters
      imax=data->getRangeCnt();
      delta=imax/9.0;
      for (j=0;j<9;j++)
	      zone[j]=minRange;
      for(j=0;j<9;j++){
      for (i = 0+(int)(j*delta); i < (int)((j+1)*delta); i++)
      { // range are stored as an integer in current units
	      r = data->getRangeMeter(i);
        if (r >= 0.020)
        { // less than 20 units is a flag value for URG scanner
          if (r<zone[j])
	          zone[j]=r;
        }
      }
      }
      /* SMRCL reply format */
      snprintf(reply, MRL, "<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" l3=\"%g\" l4=\"%g\" "
                                  "l5=\"%g\" l6=\"%g\" l7=\"%g\" l8=\"%g\" />\n", 
	                   zone[0],zone[1],zone[2],zone[3],zone[4],
                           zone[5],zone[6],zone[7],zone[8]);
      // send this string as the reply to the client
      sendMsg(msg, reply);
      // save also as gloabl variable
      for(i = 0; i < 9; i++)
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

int UFunczoneobst::DoObjectProcessing(vector<vector<double>> &v){
  RemoveDuplicates(v);

  if (v.size() > 1){
    vector<vector<double>> XYresult;
    XYresult = GetIntersectionMatrix(v); // extract X,Y values of all line intersections

    printMat(XYresult);



  }
  else {
    printf("Not enough lines!\n");
  }
  
  return 0;
}


vector<vector<double>> UFunczoneobst::GetIntersectionMatrix(vector<vector<double>> v){
  vector<vector<double>> intersections;
  for (uint i = 0; i < v.size()-1; i++){
    for (uint j = i+1; j<v.size(); j++){
      vector<double> temp = FindIntersection(v[i],v[j]);
      // perform quality check to make sure the line intersection 
      // is within the green area
      if(temp[0] > 1.0 || temp[0] < 3.0 || temp[1] > 1.0 || temp[1] < 2.0){
        intersections.push_back(temp);
      }
    }
  }
  return intersections;
}

vector<double> UFunczoneobst::FindIntersection(vector<double> u, vector<double> v){
  double a1 = u[0];
  double r1 = u[1];
  double a2 = v[0];
  double r2 = v[1];

  double t,s;

  t = (-r1*sin(a1)*sin(a2)-cos(a2)*cos(a1)*r1+r2)/(cos(a1)*sin(a2)-sin(a1)*cos(a2));
  s = (sin(a2)*sin(a1)*r2+cos(a2)*cos(a1)*r2-r1)/(cos(a1)*sin(a2)-sin(a1)*cos(a2));

  double x1,x2,y1,y2;

  x1 = r1*cos(a1)+t*(-sin(a1));
  x2 = r2*cos(a2)+s*(-sin(a2));
  y1 = r1*sin(a1)+t*cos(a1);
  y2 = r2*sin(a2)+s*cos(a2);

  vector<double> intersectionXY = {(x1+x2)/2, (y1+y2)/2};

  return intersectionXY;
}

void UFunczoneobst::RemoveDuplicates(vector<vector<double>> &v){
  uint itr = 0;
  while (true){
    double a = v[itr][0];
    double r = v[itr][1];
    
    uint j = v.size()-1;
    while (true){
        if (j==itr){
          break;
        }
        if (abs(a-v[j][0]) < 0.01 && abs(r-v[j][1]) < 0.01){
        v.erase(v.begin() + j);
        }

        j--;
    }
    
    itr++;
    
    if (itr == v.size()){
        break;
    }
  }
}

bool UFunczoneobst::DoLsqLineProcessing(vector<double> x, vector<double> y, vector<vector<double>> &lines){
  int n = x.size();
  int parts = 5;
  int delta = n/parts;
  int matches = 0;

  if (n > parts*2){
    vector<vector<double>> lineMat;//, lineMatCopy;

    // extract line segments and do least squares estimation for each segment
    for (int i = 0; i<parts; i++){
      vector<double> tempX, tempY, tempL;
      for (int j = 0+(int)(i*delta); j < (int)((i+1)*delta); j++){
        tempX.push_back(x[j]);
        tempY.push_back(y[j]);
      }
      
      /*printf("i = %d\n", i);
      printf("tempX:\n\t");
      printVec(tempX);
      printf("tempY:\n\t");
      printVec(tempY);*/

      lineMat.push_back(lsqline(tempX,tempY));
    }

    // copy of lineMat
    auto lineMatCopy(lineMat);

    // round numbers
    for(int i = 0; i<parts; i++) {
      lineMat[i][0] = round(lineMat[i][0]);
      lineMat[i][1] = round(lineMat[i][1]);
    }
    
    for(int i = 0; i<parts; i++){
      printf("Line %d:\t\talpha=%f\tr=%f\n", i, lineMat[i][0], lineMat[i][1]);
      // Count occurences of lineMat[i] 
      matches = count(lineMat.begin(), lineMat.end(), lineMat[i]);
      //printf("Matches: %d\n", matches);

      if(matches>1){
        line.push_back(lineMatCopy[i]);
      }
    }

    // return true if 1 or more line parameters have been added
    if(line.size()>0){
      return true;
    }

  }
  return false;
}

vector<double> UFunczoneobst::lsqline(vector<double> x, vector<double> y){
  int n = x.size();
  double xmean, ymean, sumx, sumy, sumx2, sumy2, sumxy;

  for (int j = 0; j < n; j++) 
  {
    sumx += x[j];
    sumy += y[j];
  }

  xmean = sumx/(double)n;
  ymean = sumy/(double)n;
  
  sumx2 = 0;
  sumy2 = 0;
  sumxy = 0;
  for (int i = 0; i<n; i++){
    sumx2 += x[i]*x[i];
    sumy2 += y[i]*y[i];
    sumxy += x[i]*y[i];
  }

  double a = 0.5*atan2((2*sumx*sumy-2*(double)n*sumxy), pow(sumx,2)-pow(sumy,2)-(double)n*sumx2+(double)n*sumy2);
  double r = xmean*cos(a) + ymean*sin(a);

  if (r<0){
    r = abs(r);
    if (a<0){
      a += PI;
    } else {
      a -= PI;
    }
  }

  vector<double> line;
  line.push_back(a);
  line.push_back(r);

  return line;
}

vector<double> UFunczoneobst::transline(vector<double> lineL, vector <double> poseW){
  /*
  * lineL = [aL, rL]
  * poseW = [xL, yL, thL]^W
  */

  double a = lineL[0] + poseW[2];
  double r = lineL[1] + (cos(a)*poseW[0] + sin(a)*poseW[1]);

  if (r<0){
    r = abs(r);
    if (a<0){
      a += PI;
    } else {
      a -= PI;
    }
  }

  vector<double> lineW;
  lineW.push_back(a);
  lineW.push_back(r);

  return lineW;
}

vector<double> UFunczoneobst::transform(vector<double> poseR){
  vector<double> poseW;

  poseW.push_back(cos(poseR[2]) * 0.26 + poseR[0]);
  poseW.push_back(sin(poseR[2]) * 0.26 + poseR[1]);
  poseW.push_back(poseR[2]);

  return poseW;
}

void UFunczoneobst::printVec(vector<double> & result){
    for (unsigned int i = 0; i < result.size(); i++) {
        cout << result.at(i) << ' ';
    }
  cout << endl;
}

void UFunczoneobst::printMat(vector<vector<double>> &result){
  for ( const vector<double> &v : result )
  {
    for ( double x : v ) cout << x << ' ';
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