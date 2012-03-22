/*!
 * @file 	SDOManger.hpp
 * @brief	Type definitions
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */

#ifndef SDOMANAGER_HPP_
#define SDOMANAGER_HPP_


#include "SDOMsg.hpp"
#include <list>


//!
/*!
 * @ingroup robotCAN
 */
class SDOManager {
public:

	SDOManager(int iBus);
	virtual ~SDOManager();

	void addSDO(SDOMsg* sdo);
	void addSDO(SDOMsgPtr sdo);
	SDOMsg*  getSDO(unsigned int index);
	int getSize();
	SDOMsg* operator[] (unsigned int index){return getSDO(index);}
	SDOMsg* getFirstSDO();
	SDOMsg* getSendSDO();
	SDOMsg* getReceiveSDO();
private:
	std::list<SDOMsgPtr>sdos_;
	SDOMsg* emptySDO_;
	int iBus_;
};

#endif /* SDOMANAGER_HPP_ */
