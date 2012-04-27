################################################################
## CAN++ Library			   		      ##
################################################################
Author: Christian Gehring
Date: Mar 16, 2012


/*! @mainpage libCAN++

	This library provides an interface to send and receive PDO and 
	SDO CAN messages on several CAN buses.
	
	It is independent of any CAN driver.
	The idea is that a CAN driver is accessed within a thread, which
	only reads the CAN messages from the shared memory and writes them to 
	the CAN driver and vice versa.
	Therefore, a shared memory interface is also provided.
	The intention was to use it within the Simulation Laboratory software package (SL)
	of Stefan Schaal. Hence, the shared memory interface is based on the SL convention.
	The CAN library can be used without SL. 
	If it is used with SL, the CMAKE variable USE_SL_FUNCTIONS should be turned on.
	See below.
	The shared memory can also be used with XENOMAI instead with pThread.
    
    \section lib Required libraries:
    - boost (libboost-dev)
    
    
	\section build Build 
		
	Without SL:
	
	cd build
	cmake .. -DUSE_SL_FUNCTIONS=OFF
	make
	
	The library will be built in folder lib.
	There are also examples that will be built in folder bin.
	
	
	With SL:
	cd build
	cmake .. -DUSE_SL_FUNCTIONS=ON
	make install
	
	Note that this will copy the library to $LAB_ROOT/lib/$MACHTYPE.
	
	With Xenomai:
	Run cmake with: cmake .. -DCOMPILE_XENOMAI=ON
    
	Build examples:
	cmake .. -DCOMPILE_EXAMPLES=ON -DABS_PATH_TO_CMAKE_FOLDER=~/libcanplusplus/trunk/cmake
	make

	The executables are copied to the folder bin.
    
*/

/*! 
    @defgroup robotCAN  robotCAN
    
    
*/

/*! 
    @defgroup device  Devices
    
    These files implement interfaces to handle a device.
    
*/

/*! 
    @defgroup bus  CAN BUS
    
    
    
*/

/*! 
    @defgroup SharedMemory  Shared Memory
    These files provides functions to handle shared memory using binary semaphores.
    
    
*/


