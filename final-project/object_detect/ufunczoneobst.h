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
#include <ulms4/ufunclaserbase.h>

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
    setCommand("zoneobst detect x y th", "zoneobstif", "obstacle detect for MRC (Compiled " __DATE__ " " __TIME__ ")");
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
    void printVec(vector<double> & result);
    bool DoLsqLineProcessing(vector<double> x, vector<double> y, vector<double> &line);
    vector<double> lsqline(vector<double> x, vector<double> y);
    vector<double> transline(vector<double> lineL, vector <double> poseW);
    vector<double> transform(vector<double> poseR);
    float round(float var);
    vector<vector<double>> goodLineFitsLaserCoordinates;
};



#endif

