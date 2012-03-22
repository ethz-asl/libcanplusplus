/*!
 * @file 	SLDefines.hpp
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN, SharedMemory
 * @brief	some variables
 *
 */

#ifdef COMPILE_XENOMAI
#define __XENO__
#endif
#define DEBUG 0



#ifdef __cplusplus
extern "C" {
#endif
	extern char         *robot_name;
	extern int           parent_process_id;


#ifdef __cplusplus
}
#endif

