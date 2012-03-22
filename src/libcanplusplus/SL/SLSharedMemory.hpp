/*!
 * @file 	SLSharedMemory.hpp
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */

#ifndef SLSHAREDMEMORY_HPP_
#define SLSHAREDMEMORY_HPP_


#ifdef __cplusplus
extern "C" {
#endif

/* local functions */
int init_sm_object_can(char *smname, size_t structsize, size_t datasize,
			  SEM_ID *semptr, void **structptr);
int init_sm_sem_can(char *smname, int filltype, void **semptr);

#ifdef __cplusplus
}
#endif


#endif /* SLSHAREDMEMORY_HPP_ */
