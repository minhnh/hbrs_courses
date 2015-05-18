
// list here *ALL* header files include from map-store.h
#include <exception>
#include <list>

// allow us access to internals of map-store classes
#define private protected
#include "map-store.h"
#include "map-store-cone.h"

// process other includes normally
#undef private

#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

using namespace MobileRobots;

struct FailInfo {
  FailInfo(int chk, int expected, const char *item) : found(chk), required(expected), info(0) {
    if (item)
      info = strdup(item);
  }

  FailInfo(const FailInfo &old) : found(old.found), required(old.required), info(0) {
    if (old.info)
      info = strdup(old.info);
  }

  ~FailInfo() {
    if (info)
      free(info);
  }

  int found;
  int required;
  char *info;

private:
  FailInfo &operator=(const FailInfo &old);
};

struct TestLine {
  TestLine(double xs, double ys, double xe, double ye) : startX(xs), startY(ys), endX(xe), endY(ye) {
    dx = xe-xs;
    dy = ye-ys;
    len = sqrt( dx*dx + dy*dy );
    dxnorm = dx/len;
    dynorm = dy/len;

  }

  double pointDist(double tx, double ty) {
    double tdx = tx - startX;
    double tdy = ty - startY;

    double project = vScal(tdx, tdy);
    /*
    fx = startX + dxnorm*project;
    fy = startY + dynorm*project;

    distx = tx - fx;  // = tdx + startX - fx
    disty = ty - fy;  // = tdy + startY - fy

    distx = tdx - dxnorm*project;
    disty = tdy - dynorm*project;
    */

    tdx -= dxnorm*project;
    tdy -= dynorm*project;

    return sqrt(tdx*tdx + tdy*tdy);
  }

  double pointProjection(double tx, double ty) {
    double tdx = tx - startX;
    double tdy = ty - startY;

    return vScal(tdx, tdy);
  }

  double vScal(double tx, double ty) {
    return (tx*dxnorm + ty*dynorm);
  }

  double vScal(double x1, double y1, double x2, double y2) {
    return (x1*x2 + y1*y2);
  }

  double startX, startY, endX, endY;
  double dx,dy,len;

  double dxnorm, dynorm;
};

using std::cout;
using std::endl;

class MapStoreTest : public MapStore {
public:
  MapStoreTest() : MapStore(10,10), okCnt(0), failCnt(0) {

    // checking sizes
    cout << "Checking map sizes ..." << endl;
    checkVal(minX(), -5, "minX");
    checkVal(maxX(), 5, "maxX");
    checkVal(minY(), -5, "minY");
    checkVal(maxY(), 5, "maxY");

    cout << "Checking limit tests ..." << endl;
    int x;
    for (x = -7; x < 7; x++) {
      if (x < -5) {
	// outside
	if (isInX(x))
	  reportFail(x, "false expected, but true found", "isInX");
      } else if (x > 5) {
	// outside
	if (isInX(x))
	  reportFail(x, "false expected, but true found", "isInX");
      } else {
	// inside
	if (!isInX(x))
	  reportFail(x, "true expected, but false found", "isInX");
      }
    }

    /*
    for(x = -7; x < 7; x++)  {
      for(dx = 1; dx < 5; dx++) {
	int xt = t;
	int dxt = dx;
	convXRange(xt, dxt);
	if (x < -5) {
	  // outside
	  if (isInX(x))
	    reportFail(x, "false expected, but true found", "isInX");
	} else if (x > 5) {
	  // outside
	  if (isInX(x))
	    reportFail(x, "false expected, but true found", "isInX");
	} else {
	  // inside
	  if (!isInX(x))
	    reportFail(x, "true expected, but false found", "isInX");
	}
      }
    }
    */

    cout << "Checkking fillRect() ..." << endl;
    fillRect(-2,-2,5,6, 1.5);

    cout << "Checking trace() ..." << endl;
    int xs = -4;
    int ys = -4;
    int xe = -4;
    int ye = -4;
    FILE *fh = fopen("beamdata.txt", "w");
    for (xe = -3; xe < 5; xe++) {
      fprintf(fh, "\n\n");
      checkBeam(xs, ys, xe, ye, -2, -2, -2+5-1, -2+6-1, 1.5, fh);
      fprintf(fh, "e\n");
    }
    xe = 4;
    for (ye = -3; ye < 5; ye++) {
      fprintf(fh, "\n\n");
      checkBeam(xs, ys, xe, ye, -2, -2, -2+5-1, -2+6-1, 1.5, fh);
      fprintf(fh, "e\n");
    }
    ye = 4;
    for (xe = 3; xe > -4; xe--) {
      fprintf(fh, "\n\n");
      checkBeam(xs, ys, xe, ye, -2, -2, -2+5-1, -2+6-1, 1.5, fh);
      fprintf(fh, "e\n");
    }
    xe = -4;
    for (ye = 3; ye > -4; ye--) {
      fprintf(fh, "\n\n");
      checkBeam(xs, ys, xe, ye, -2, -2, -2+5-1, -2+6-1, 1.5, fh);
      fprintf(fh, "e\n");
    }
    fclose(fh);
  }

  ~MapStoreTest() {
    cout << "Done" << endl;
  }

  bool checkBeam(int startX, int startY, int endX, int endY, int filledSX, int filledSY, int filledEX, int filledEY, double filledVal, FILE *fh) {
    cout << "Checking beam (" << startX << ", " << startY << ") --> (" << endX << ", " << endY << ") ... ";
    double dx = endX-startX;
    double dy = endY-startY;
    double dir = atan2(dy, dx);
    double len = sqrt( dx*dx + dy*dy );
    MapStoreTrace tr = trace(startX,startY, dir, len);
    MapStoreTrace::MapIt iter = tr.traceStart();

    TestLine myLine(startX, startY, endX, endY);

    bool res = true;
    const double sqrt2 = sqrt(2);
    MapStoreCell cell;
    int idx = 0;
    while (iter != tr.traceEnd()) {
      cell = *iter;
      iter++;
      double dist = myLine.pointDist(cell.x, cell.y);
      double linePart = myLine.pointProjection(cell.x, cell.y);
      //      cout << " # " << idx << ": (" << (startX+linePart*cos(dir)) << ", " << (startY+linePart*sin(dir)) << ") <-> (" << cell.x << ", " << cell.y << ")" << endl;
      if (fh != 0)
	fprintf(fh, "%d   %.3f  %.3f   %d  %d\n", idx, startX+linePart*cos(dir), startY+linePart*sin(dir), cell.x, cell.y);
      if (dist > sqrt2) {
	if (res)
	  cout << endl;
	cout << "point " << cell.x << " " << cell.y << " not on line: dist = " << dist << endl;
	res = false;
      }
      if ( (filledSX <= cell.x) && (cell.x <= filledEX) &&
	   (filledSY <= cell.y) && (cell.y <= filledEY) ) {
	// inside filled region
	if (cell.val != filledVal) {
	  cout << "Cell (" << cell.x << ", " << cell.y << ") value " << cell.val << ", expected " << filledVal << endl;
	  res = false;
	}
      } else {
	// outside filled region
	if (cell.val != 0) {
	  cout << "Cell (" << cell.x << ", " << cell.y << ") value " << cell.val << ", expected 0" << endl;
	  res = false;
	}
      }

      idx += 1;
    }
    if (cell.x != endX || cell.y != endY) {
      cout << "Endpoint " << endX << " " << endY << " not reached, last point is " << cell.x << " " << cell.y << endl;
      res = false;
    }
    if (res)
      cout << "OK" << endl;
    return res;
  }

  void reportFail(int arg, const char *explain, const char *item) {
    cout << item << ": arg " << arg << ", result: " << explain << endl;
    failCnt += 1;
  }

  bool checkVal(int chk, int expected, const char *item) {
    cout << item << ": req " << expected << ", is " << chk;
    if (chk == expected) {
      cout << " : OK" << endl;
      okCnt += 1;
      return true;
    } else {
      cout << " : FAIL" << endl;
      failCnt += 1;
      failureList.push_back(FailInfo(chk, expected, item));
      return false;
    }
  }

  bool growTest(void) {
    cout << "Checking map grow() ..." << endl;

    // first fill the map with data, to check if the content is not mixed
    int x1,y1;
    for (x1 = -5; x1 < 6; x1++) {
      for (y1 = -5; y1 < 6; y1++) {
	set(x1, y1, x1 + 6 + (y1 + 6) * 30);
      }
    }
 
    // should gow from -5..5 / -5..5 to -6..10 / -5..11
    grow(2, 3, 8);
    checkVal(minX(), -6, "minX");
    checkVal(maxX(), 10, "maxX");
    checkVal(minY(), -5, "minY");
    checkVal(maxY(), 11, "maxY");

    cout << "   checking range tests after grow() ..." << endl;

    bool testVal = false;
    bool bomb = false;
    for (x1 = -7; x1 < 12; x1++) {
      for (y1 = -7; y1 < 13; y1++) {
	testVal = false;
	if ( (-5 <= x1) && (x1 <= 5) ) {
	  if ( (-5 <= y1) && (y1 <= 5) ) {
	    // within original area
	    testVal = true;
	  }
	}
	bomb = true;
	if ( (-6 <= x1) && (x1 <= 10) ) {
	  if ( (-5 <= y1) && (y1 <= 11) ) {
	    // within original area
	    bomb = false;
	  }
	}
	try {
	  double val = get(x1, y1);
	  if (bomb) {
	    cout << "   error: expected 'Range' exception, but nothing happened: (" << x1 << ", " << y1 << ")" << endl;
	  }
	  char checkbuf[30];
	  snprintf(checkbuf, 29, " cell (%d, %d) ", x1, y1);
	  if (testVal) {
	    checkVal(val,  x1 + 6 + (y1 + 6) * 30, checkbuf);
	  }
	  /*
	    // new values are not initialized, so nothing to test here.
	  else {
	    checkVal(val, 0, checkbuf);
	  }
	  */
	} catch (MapStoreError er) {
	  if (bomb) {
	    if (er.getType() == MapStoreError::Range)
	      cout << "  OK received expected exception" << endl;
	    else
	      cout << "  error received exception as expected, but of wrong type '" << er.what() << "'" << endl;
	  } else {
	    cout << "   error: unexpected '" << er.what() << "' exception: (" << x1 << ", " << y1 << ")" << endl;
	  }
	}
      }
    }
    return (failCnt == 0);
  }

  void loadTest(void) {
    if (loadMap("maploadfile.txt")) {
      cout << "loaded map successfully.  New map size: " << sizeX << ", " << sizeY << endl;
      cout << "New origin: " << originX << ", " << originY << endl;
    } else {
      cout << "Failed to load map" << endl;
    }
  }
   
private:
  int okCnt;
  int failCnt;
  std::list<FailInfo> failureList;
};

int main(int argc, char *argv[]) {

  MapStoreTest myTest;

  double dir;
  for (dir = 0; dir <= 90; dir += 15) {
    cout << "Checking cone " << dir << "° ..." << endl;
    cout << "  preparing ..." << endl;
    myTest.eraseRect(-5,-5, 11,11);
    /*
    cout << "  running fillCone ..." << endl;
    myTest.fillCone(-4,-4, dir/180.0*M_PI, 30.0/180.0*M_PI, 8, 3);
    */
    int x,y;
    double val;

    cout << "  creating cone iterator ..." << endl;
    MapStoreCone myCone(-4,-4, dir/180.0*M_PI, 30.0/180.0*M_PI, 8);
    {
      MapStoreBeam beamInfoCenter(-4.0,-4.0, dir/180.0*M_PI, 8.0);
      MapStoreBeam beamInfoLeft(-4.0,-4.0, dir/180.0*M_PI + 30.0/180.0*M_PI/2, 8.0);
      MapStoreBeam beamInfoRight(-4.0,-4.0, dir/180.0*M_PI - 30.0/180.0*M_PI/2, 8.0);
      beamInfoCenter.getInitCell(x,y);
      do {
	if (!myTest.isInX(x))
	  continue;
	if (!myTest.isInY(y))
	  continue;
	myTest.set(x,y, 4);
      } while(beamInfoCenter.nextCell(x,y));

      beamInfoLeft.getInitCell(x,y);
      do {
	if (!myTest.isInX(x))
	  continue;
	if (!myTest.isInY(y))
	  continue;
	myTest.set(x,y, 4);
      } while(beamInfoLeft.nextCell(x,y));

      beamInfoRight.getInitCell(x,y);
      do {
	if (!myTest.isInX(x))
	  continue;
	if (!myTest.isInY(y))
	  continue;
	myTest.set(x,y, 4);
      } while(beamInfoRight.nextCell(x,y));
    }

    cout << "  running cone iterator ..." << endl;
    while(myCone.nextCell(x,y)) {
      if (!myTest.isInX(x))
	continue;
      if (!myTest.isInY(y))
	continue;
      val =  myTest.get(x,y);
      if (val != 3) {
	//	cout << "Read cone cell (" << x << ", " << y << "): wrong value " << val << endl;
      }
      if (val == 0)
	myTest.set(x,y,1);
    }

    cout << endl;
    for(y=-5; y < 6; y++) {
      for(x=-5; x < 6; x++) {
	val = myTest.get(x,y);
	if (val == 3) {
	  //	  cout << "Read non-cone cell (" << x << ", " << y << "): wrong value " << val << endl;
	  cout << " #";
	} else if (val == 1) {
	  cout << " +";
	} else if (val == 4) {
	  cout << " *";
	} else {
	  cout << " .";
	}
      }
      cout << endl;
    }
  }

  myTest.growTest();
  
  myTest.loadTest();
  
  // save it again
  FILE *savefile = fopen("mapsavefile.plot", "w");
  myTest.dumpGP(savefile, "dump of loaded map");

  return 0;
}

