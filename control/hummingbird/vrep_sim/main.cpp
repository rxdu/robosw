/*
 * main.cpp
 *
 *  Created on: Jul 18, 2015
 *      Author: rdu
 */

// headers for standard library
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <math.h>
#include <ctime>

// headers for vrep remote api
extern "C" {
    #include "extApi.h"
/*	#include "extApiCustom.h" // custom remote API functions */
}

// headers for g3log
#include "library/g3log/g3log.hpp"
#include "library/g3log/logworker.hpp"
#include "library/g3log/std2_make_unique.hpp"
#include "library/g3log/datasink.hpp"

// headers for user code
#include <sim_process/quad_sim_process.h>

using namespace g3;
using namespace srcl_ctrl;

#define USE_FIXED_PORT_NUM

#ifdef USE_FIXED_PORT_NUM
#define EXP_PORT_NUM 29999
#endif

clock_t	last_time = 0;

int main(int argc,char* argv[])
{
	int portNb = 0;
	unsigned long loop_count = 0;

#ifdef USE_FIXED_PORT_NUM
	portNb = EXP_PORT_NUM;
#else
	if (argc>=1)
	{
		portNb=atoi(argv[1]);
	}
	else
	{
		printf("Please indicate following argument(s): 'portNumber'!\n");
		extApi_sleepMs(5000);
		return 0;
	}
#endif

#ifdef ENABLE_LOG
	// initialize logger
	std::unique_ptr<LogWorker> logworker{ LogWorker::createWithNoSink() };
	auto sinkHandle = logworker->addSink(std2::make_unique<DataSink>("carsim","/home/rdu/Workspace/robot_toolkit/simulator/vrep/ackermann/log"),
			&DataSink::fileWrite);
	initializeLogging(logworker.get());
#endif

	// initialize simulator
	simxInt clientID = simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);

	if (clientID!=-1)
	{
		std::cout << "INFO: Connected to server." << std::endl;

		// initialize a simulation process
		QuadSimProcess sim_process(clientID);
		simxInt ping_time = 0;

		std::cout << "INFO: Created a simulation client." << std::endl;

		simxSynchronous(clientID,true);
		simxStartSimulation(clientID, simx_opmode_oneshot_wait);

		std::cout << "INFO: Enabled synchronous mode." << std::endl;

		while (simxGetConnectionId(clientID)!=-1)
		{
			if(loop_count == 0)
				std::cout << "INFO: Entered control loop." << std::endl;

			if(sim_process.ReceiveDataFromSimulator())
			{
				// update simulated control loop
				sim_process.SimLoopUpdate();

				clock_t time_now = clock();
				double freq = 1.0/(double(time_now - last_time)/CLOCKS_PER_SEC);
				last_time = time_now;

//				std::cout << "Control loop frequency: " << freq << std::endl;
			}
//			else
//				std::cout<<"failed to fetch data from simulator"<<std::endl;

			// send command to robot
			sim_process.SendDataToSimulator();

//			extApi_sleepMs(1); 		// use usleep(1750) to get shorter delay
			usleep(50);

			loop_count++;
			simxSynchronousTrigger(clientID);

			// After this call, the first simulation step is finished (Blocking function call)
			simxGetPingTime(clientID, &ping_time);
		}

		std::cout << "INFO: Exited control loop." << std::endl;

		simxStopSimulation(clientID, simx_opmode_oneshot_wait);

		simxFinish(clientID);
	}
	else
	{
		std::cout << "ERROR: Failed to connect to server" << std::endl;
	}

	std::cout << "INFO: Simulation ends." << std::endl;

	return(0);
}


