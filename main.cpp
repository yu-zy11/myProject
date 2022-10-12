#include <qpOASES.hpp>
#include <iostream>
#include"./controller/mpc_controller/convexMPC.h"

/** Example for qpOASES main function using the SQProblem class. */
int main( )
{
	convexMPC mpc_runner(1,0.02);
	std::cout<<mpc_runner.test<<"\n";
	mpc_runner.initConvexMPC();
	for(int i=0;i<10;i++)
		mpc_runner.run();
	return 0; 
}


/*
 *	end of file
 */
