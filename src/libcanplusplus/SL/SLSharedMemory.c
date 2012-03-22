/*!
 * @file 	SLSharedMemory.c
 * @brief
 * @author 	Christian Gehring
 * @date 	Jan, 2012
 * @version 1.0
 * @ingroup robotCAN
 *
 */

#include "LibCANConfig.h"
#ifdef COMPILE_XENOMAI
	#include "XenoHeaders.hpp"
#else
	#include "PThreadHeaders.hpp"
#endif
#ifdef USE_SL_FUNCTIONS
	#include "SL.h"
	#include "SL_vx_wrappers.h"
	#include "utility.h"
#else
	#include "SLVxWrapper.hpp"
	#include "SLUtility.h"
#endif

#include "CANSharedMemory.hpp"
#include "SLSharedMemory.hpp"



#define DEBUG FALSE

#define TIME_OUT_NS 1000000000


/* local variables */
static int         n_bytes_sm_allocated = 0;

#ifdef __cplusplus
extern "C" {
#endif


/*!***************************************************************************
******************************************************************************
\note   init_sm_object
\date   May 2000

\remarks

initializes a shared memory object with semaphore as first element and
data array

******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]  smname     :  name of the shared memory object
\param[in]  structsize :  size of base structure
\param[in]  datasize   :  size of data structure
\param[out] semptr     :  ptr to object's semaphore
\param[out] structptr  :  handle to object's base structure

*****************************************************************************/
int
init_sm_object_can(char *smname, size_t structsize, size_t datasize,
	       SEM_ID *semptr, void **structptr)

{
  int      mtype;
  SEM_ID  *semptr_sm;
//  STATUS   error;
  char     smn[100];
  char     semn[100];

  /* build the full name from both robot name and desired name */
#ifdef VX  /* vxworks has a 20 character limit */
  sprintf(smn,"%s",smname);
#else
  sprintf(smn,"%s.%s",robot_name,smname);
#endif

  if (smNameFind(smn, structptr,&mtype, NO_WAIT) == ERROR) {

    /* get the object's base structure and add datasize to memory chunk */
#ifdef VX
    *structptr = (void *)smMemCalloc(1,structsize+datasize);
#else
    *structptr = (void *)smMemCalloc(smn,parent_process_id,1,structsize+datasize);
#endif
    if (*structptr == NULL) {
      printf("Couldn't create shared memory object %s\n",smn);
      return FALSE;
    }

    /* get the object's semaphore */
    semptr_sm = *structptr;
#ifdef VX
    *semptr = semBSmCreate(SEM_Q_FIFO, SEM_FULL);
    if (*semptr == NULL) {
#else
    sprintf(semn,"%s.%s_sem",robot_name,smname);
    *semptr = semBSmCreate (semn,parent_process_id,SEM_Q_FIFO, SEM_FULL);
    if (*semptr == (SEM_ID) (-1)) {
#endif
      printf("Couldn't create shared semaphore for object %s -- sem_id=%ld\n",
	     semn,(long)*semptr);
      return FALSE;
    }

#ifdef __XENO__
    // xenomai does not have a global identifier for semaphores
    *semptr_sm = NULL;
#else
    // vxWorks and Unix use global identifiers for semaphores which
    // we can keep in shared memory
    *semptr_sm = *semptr;
#endif

#ifdef VX
    /* add the object to the name database */
    error = smNameAdd(smn,(void*)smObjLocalToGlobal(*structptr),T_SM_PART_ID);
    if (error == ERROR)
      return FALSE;
#endif

    } else {
      *structptr = smObjGlobalToLocal(*structptr);
  }

  if (DEBUG)
    printf("Shared memory for %s is set at struct: g=0x%lx l=0x%lx\n",
	   smn,
	   (unsigned long) smObjLocalToGlobal((void*)*structptr),
	   (unsigned long) *structptr);

  // keep statistics of allocated shared memory
  n_bytes_sm_allocated += datasize + structsize;

  return TRUE;

}

/*!***************************************************************************
******************************************************************************
\note    init_sm_sem
\date    May 2000

\remarks

initializes a shared semaphore

******************************************************************************
Function Parameters: [in]=input,[out]=output

\param[in]  smname     :  name of the shared memory object
\param[in]  filltype   :  SEM_FULL or SEM_EMPTY
\param[out] semptr     :  handle to semaphore

*****************************************************************************/
int
init_sm_sem_can(char *smname, int filltype, void **semptr)

{
  int      mtype;
//  STATUS   error;
  char     smn[100];

  /* build the full name from both robot name and desired name */
#ifdef VX  /* vxworks has a 20 character limit */
  sprintf(smn,"%s",smname);
  smn[19]='\0';
#else
  sprintf(smn,"%s.%s",robot_name,smname);
#endif

  if (smNameFind(smn, semptr, &mtype,NO_WAIT)== ERROR) {
#ifdef VX
    *semptr = (void *)semBSmCreate (SEM_Q_FIFO, filltype);
    if (*semptr == NULL) {
#else
      *semptr = (void *) semBSmCreate (smn,parent_process_id,SEM_Q_FIFO, filltype);
      if (*semptr == (void *)-1 ) {
#endif
	printf("Couldn't create shared semaphore %s\n",smn);
	return FALSE;
      }

#ifdef VX
      error = smNameAdd(smn, (void*)*semptr, T_SM_SEM_B);
      if (error == ERROR)
	return FALSE;
#endif

    } else {
      ;
    }

    if (DEBUG)
      printf("Shared semaphore for %s is set at 0x%lx\n",smn,(unsigned long) *semptr);

    return TRUE;

}


#ifdef __cplusplus
}
#endif
