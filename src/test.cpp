#include "Dstar.h"

int main() {
 Dstar dstar;
 list<state> mypath;

 dstar.init(0,0,10,5);         // set start to (0,0) and goal to (10,5)
 dstar.updateCell(3,4,-1);     // set cell (3,4) to be non traversable
 dstar.updateCell(2,2,42.432); // set set (2,2) to have cost 42.432

 dstar.replan();               // plan a path
 mypath = dstar.getPath();     // retrieve path

 dstar.updateStart(10,2);      // move start to (10,2)
 dstar.replan();               // plan a path
 mypath = dstar.getPath();     // retrieve path

 dstar.updateGoal(0,1);        // move goal to (0,1)
 dstar.replan();               // plan a path
 mypath = dstar.getPath();     // retrieve path

 for(state p : mypath)
{
    printf("%i %i \n",p.x,p.y);
}
 return 0;
}