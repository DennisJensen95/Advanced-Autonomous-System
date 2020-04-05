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
      vector<double> line;
      bool state = DoLsqLineProcessing(x,y,line);
      
      if (state){
	      vector<double> poseR, poseW;
	      poseR.push_back(xo);
	      poseR.push_back(yo);
	      poseR.push_back(tho);

	      poseW = transform(poseR);

	      vector<double> lineW;
	      lineW = transline(line, poseW);

	      printf("Robot pose in world:\t(%.2f,%.2f,%.2f)\n", poseR[0], poseR[1], poseR[2]);
	      printf("Laser pose in world:\t(%.2f,%.2f,%.2f)\n", poseW[0], poseW[1], poseW[2]);

	      printf("Line parameters (World):\n");
	      printVec(lineW);
      }
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

void UFunczoneobst::printVec(vector<double> & result){
    for (unsigned int i = 0; i < result.size(); i++) {
        cout << result.at(i) << ' ';
    }
  cout << endl;
}

bool UFunczoneobst::DoLsqLineProcessing(vector<double> x, vector<double> y, vector<double> &line){
  int n = x.size();
  int parts = 5;
  int delta = n/parts;

  if (n > parts*2){
    vector<vector<double>> lineMat;

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

    
    for(int i = 0; i<parts; i++){
      printf("Line %d:\t\talpha=%.2f\tr=%.2f\n", i, lineMat[i][0], lineMat[i][1]);
    }

    vector<double> temp = lsqline(x,y);
    line.push_back(temp[0]);
    line.push_back(temp[1]);
    return true;
  }
  return false;
}

vector<double> UFunczoneobst::lsqline(vector<double> x, vector<double> y){
  int n = x.size();
  double xmean, ymean, sumx, sumy, sumx2, sumy2, sumxy;

  /*printf("x:\n\t");
  printVec(x);
  printf("\ny:\n\t");
  printVec(y);
  printf("\n\n");*/

  for (int j = 0; j < n; j++) 
  {
    sumx += x[j];
    sumy += y[j];
  }

  /*printf("sumx:\n\t");
  printf("%.2f\n", sumx);
  printf("sumy:\n\t");
  printf("%.2f\n", sumy);
  printf("\n\n");*/

  xmean = sumx/(double)n;
  ymean = sumy/(double)n;
  
  /*printf("xmean:\n\t");
  printf("%.2f\n", xmean);
  printf("ymean:\n\t");
  printf("%.2f\n", ymean);
  printf("\n\n");*/

  sumx2 = 0;
  sumy2 = 0;
  sumxy = 0;
  for (int i = 0; i<n; i++){
    sumx2 += x[i]*x[i];
    sumy2 += y[i]*y[i];
    sumxy += x[i]*y[i];
  }

  /*printf("sumx2:\n\t");
  printf("%.2f\n", sumx2);
  printf("sumy2:\n\t");
  printf("%.2f\n", sumy2);
  printf("sumxy:\n\t");
  printf("%.2f\n", sumxy);
  printf("\n\n");*/

  double a = 1.0/2.0*atan2((2*sumx*sumy-2*(double)n*sumxy), pow(sumx,2)-pow(sumy,2)-(double)n*sumx2+(double)n*sumy2);
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
  vector<double> lineW;

  lineW.push_back(lineL[0] + poseW[2]);
  lineW.push_back(lineL[1] + ( cos(lineW[0])*poseW[0] + sin(lineW[0])*poseW[1] ));

  return lineW;
}

vector<double> UFunczoneobst::transform(vector<double> poseR){
  vector<double> poseW;

  poseW.push_back(cos(poseR[2]) * 0.26 + poseR[0]);
  poseW.push_back(sin(poseR[2]) * 0.26 + poseR[1]);
  poseW.push_back(poseR[2]);

  return poseW;
}
