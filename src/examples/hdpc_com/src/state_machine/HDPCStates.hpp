/*
 * HDPCStates.hpp
 *
 *  Created on: Apr 16, 2012
 *      Author: gech
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
      sc::custom_reaction< EvEmergencyStop >,
      sc::custom_reaction< EvStateInfo >
    > reactions;

    StInit( my_context ctx );
    virtual ~StInit();

    sc::result react( const EvExecute& );
    sc::result react( const EvEmergencyStop& );
    sc::result react( const EvStateInfo& );

};

//////////////////////////////////////////////////////////////////////////////
struct StStopping : sc::state< StStopping, StTop >
{
  public:
    typedef mpl::list<
      sc::custom_reaction< EvExecute >,
      sc::custom_reaction< EvEmergencyStop >,
      sc::custom_reaction< EvStateInfo >
    > reactions;

    StStopping( my_context ctx );
    virtual ~StStopping();

    sc::result react( const EvExecute& );
    sc::result react( const EvEmergencyStop& );
    sc::result react( const EvStateInfo& );
  private:
    int sv_disabling_counter_;
    int iDevice_;
    bool* isDisabling_;
    bool* isReallyDisabled_;
};

//////////////////////////////////////////////////////////////////////////////
struct 	StStarting;
struct StStop : sc::state< StStop, StTop >
{
  public:
    typedef mpl::list<
      sc::custom_reaction< EvExecute >,
      sc::custom_reaction< EvEmergencyStop >,
      sc::custom_reaction< EvStateInfo >,
      sc::transition<EvStarting, StStarting>,
	  sc::transition<EvReseting, StInit>
    > reactions;

    StStop( my_context ctx );
    virtual ~StStop();

    sc::result react( const EvExecute& );
    sc::result react( const EvEmergencyStop& );
    sc::result react( const EvStateInfo& );

};

//////////////////////////////////////////////////////////////////////////////
struct StStarting : sc::state< StStarting, StTop >
{
  public:
    typedef mpl::list<
      sc::custom_reaction< EvExecute >,
      sc::custom_reaction< EvEmergencyStop >,
      sc::custom_reaction< EvStateInfo >
    > reactions;

    StStarting( my_context ctx );
    virtual ~StStarting();

    sc::result react( const EvExecute& );
    sc::result react( const EvEmergencyStop& );
    sc::result react( const EvStateInfo& );
  private:
    int sv_enabling_counter_;
    int iDevice_;
    bool* isEnabling_;
    bool* isReallyEnabled_;
};

//////////////////////////////////////////////////////////////////////////////
struct StDrive : sc::state< StDrive, StTop >
{
  public:
    typedef mpl::list<
      sc::custom_reaction< EvExecute >,
      sc::custom_reaction< EvEmergencyStop >,
      sc::custom_reaction< EvStateInfo>,
	  sc::transition<EvStopping, StStopping>
    > reactions;

    StDrive( my_context ctx );
    virtual ~StDrive();

    sc::result react( const EvExecute& );
    sc::result react( const EvEmergencyStop& );
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


#endif /* HDPCSTATES_HPP_ */
