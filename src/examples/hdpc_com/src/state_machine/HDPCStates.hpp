/*!
 * @file 	HDPCStates.hpp
 * @brief	States of the state machine for HDPC
 * @author 	Christian Gehring
 * @date 	Apr, 2012
 * @version 1.0
 * @ingroup robotCAN, device
 *
 */

#ifndef HDPCSTATES_HPP_
#define HDPCSTATES_HPP_

#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>

#include <boost/intrusive_ptr.hpp>
#include <boost/mpl/list.hpp>

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

//////////////////////////////////////////////////////////////////////////////
struct StInit;
struct StStop;
struct StTop : sc::state< StTop, HDPCStateMachine, StInit >
{
  public:
    typedef mpl::list<
      sc::custom_reaction< EvExecute >,
      sc::custom_reaction< EvEmergencyStop >,
      sc::custom_reaction< EvTerminateSM >
    > reactions;

    StTop( my_context ctx );
    virtual ~StTop();

    sc::result react( const EvExecute& );
    sc::result react( const EvTerminateSM& );
    sc::result react( const EvEmergencyStop& );

};

//////////////////////////////////////////////////////////////////////////////
struct StInit : sc::state< StInit, StTop >
{
  public:
    typedef mpl::list<
      sc::custom_reaction< EvExecute >,
      sc::custom_reaction< EvStateInfo >
    > reactions;

    StInit( my_context ctx );
    virtual ~StInit();

    sc::result react( const EvExecute& );
    sc::result react( const EvStateInfo& );
  private:
    int iDevice_;
    int waitForDeviceCount_;

};

//////////////////////////////////////////////////////////////////////////////
struct StHoming : sc::state< StHoming, StTop >
{
  public:
    typedef mpl::list<
      sc::custom_reaction< EvExecute >,
      sc::custom_reaction< EvStateInfo >
    > reactions;

    StHoming( my_context ctx );
    virtual ~StHoming();

    sc::result react( const EvExecute& );
    sc::result react( const EvStateInfo& );
  private:
    int waitCount_;


};



//////////////////////////////////////////////////////////////////////////////
struct StDriveTestDrivingMotor;
struct StDriveTestSteeringMotor;
struct StDriveTestAll;
struct StDrive;
struct StStop : sc::state< StStop, StTop >
{
  public:
    typedef mpl::list<
      sc::custom_reaction< EvExecute >,
      sc::custom_reaction< EvStateInfo >,
      sc::transition<EvStarting, StDriveTestAll>,
	  sc::transition<EvReseting, StInit>,
	  sc::custom_reaction<EvStopping>
    > reactions;

    StStop( my_context ctx );
    virtual ~StStop();

    sc::result react( const EvExecute& );
    sc::result react( const EvStateInfo& );
    sc::result react( const EvStopping& );

};

//////////////////////////////////////////////////////////////////////////////
struct StDrive;
struct StDriveTop : sc::state< StDriveTop, StTop, StDrive >
{
  public:
    typedef mpl::list<
      sc::custom_reaction< EvExecute >
    > reactions;

    StDriveTop( my_context ctx );
    virtual ~StDriveTop();

    sc::result react( const EvExecute& );
  private:
    int counter_;

};
//////////////////////////////////////////////////////////////////////////////
struct StDrive : sc::state< StDrive, StDriveTop >
{
  public:
    typedef mpl::list<
      sc::custom_reaction< EvExecute >,
      sc::custom_reaction< EvStateInfo>,
	  sc::transition<EvStopping, StStop>
    > reactions;

    StDrive( my_context ctx );
    virtual ~StDrive();

    sc::result react( const EvExecute& );;
    sc::result react( const EvStateInfo& );

};

//////////////////////////////////////////////////////////////////////////////
struct StDriveTestDrivingMotor : sc::state< StDriveTestDrivingMotor, StDriveTop >
{
  public:
    typedef mpl::list<
      sc::custom_reaction< EvExecute >,
      sc::custom_reaction< EvStateInfo>,
	  sc::transition<EvStopping, StStop>
    > reactions;

    StDriveTestDrivingMotor( my_context ctx );
    virtual ~StDriveTestDrivingMotor();

    sc::result react( const EvExecute& );
    sc::result react( const EvStateInfo& );

};

//////////////////////////////////////////////////////////////////////////////
struct StDriveTestSteeringMotor : sc::state< StDriveTestSteeringMotor, StDriveTop >
{
  public:
    typedef mpl::list<
      sc::custom_reaction< EvExecute >,
      sc::custom_reaction< EvStateInfo>,
	  sc::transition<EvStopping, StStop>
    > reactions;

    StDriveTestSteeringMotor( my_context ctx );
    virtual ~StDriveTestSteeringMotor();

    sc::result react( const EvExecute& );
    sc::result react( const EvStateInfo& );

};

//////////////////////////////////////////////////////////////////////////////
struct StDriveTestAll : sc::state< StDriveTestAll, StDriveTop >
{
  public:
    typedef mpl::list<
      sc::custom_reaction< EvExecute >,
      sc::custom_reaction< EvStateInfo>,
	  sc::transition<EvStopping, StStop>
    > reactions;

    StDriveTestAll( my_context ctx );
    virtual ~StDriveTestAll();

    sc::result react( const EvExecute& );
    sc::result react( const EvStateInfo& );

};

//////////////////////////////////////////////////////////////////////////////
struct StFault : sc::state< StFault, StTop >
{
  public:
    typedef mpl::list<
      sc::custom_reaction< EvExecute >,
      sc::custom_reaction< EvEmergencyStop >,
      sc::custom_reaction< EvStateInfo >,
      sc::transition<EvReseting, StInit>
    > reactions;

    StFault( my_context ctx );
    virtual ~StFault();

    sc::result react( const EvExecute& );
    sc::result react( const EvEmergencyStop& );
    sc::result react( const EvStateInfo& );

};

////////////////////////////////////////////////////////////////////////////////
//struct StStarting : sc::state< StStarting, StTop >
//{
//  public:
//    typedef mpl::list<
//      sc::custom_reaction< EvExecute >,
//      sc::custom_reaction< EvEmergencyStop >,
//      sc::custom_reaction< EvStateInfo >
//    > reactions;
//
//    StStarting( my_context ctx );
//    virtual ~StStarting();
//
//    sc::result react( const EvExecute& );
//    sc::result react( const EvEmergencyStop& );
//    sc::result react( const EvStateInfo& );
//  private:
//    int sv_enabling_counter_;
//    int iDevice_;
//    bool* isEnabling_;
//    bool* isReallyEnabled_;
//};

////////////////////////////////////////////////////////////////////////////////
//struct StStopping : sc::state< StStopping, StTop >
//{
//  public:
//    typedef mpl::list<
//      sc::custom_reaction< EvExecute >,
//      sc::custom_reaction< EvEmergencyStop >,
//      sc::custom_reaction< EvStateInfo >
//    > reactions;
//
//    StStopping( my_context ctx );
//    virtual ~StStopping();
//
//    sc::result react( const EvExecute& );
//    sc::result react( const EvEmergencyStop& );
//    sc::result react( const EvStateInfo& );
//  private:
//    int sv_disabling_counter_;
//    int iDevice_;
//    bool* isDisabling_;
//    bool* isReallyDisabled_;
//};


#endif /* HDPCSTATES_HPP_ */
