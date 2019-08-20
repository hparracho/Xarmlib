// ----------------------------------------------------------------------------
// @file    syscall.cpp
// @brief   Dependent OS functions for FatFs.
// @date    25 July 2019
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2019 Helder Parracho (hparracho@gmail.com)
//
// See README.md file for additional credits and acknowledgments.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
// ----------------------------------------------------------------------------

#include "ffconf.h"

namespace xarmlib
{

extern "C"
{




// ----------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------------

#if FF_FS_REENTRANT
/*------------------------------------------------------------------------*/
/* Create a Synchronization Object                                        */
/*------------------------------------------------------------------------*/
/* This function is called by f_mount() function to create a new
/  synchronization object, such as semaphore and mutex. When a 0 is
/  returned, the f_mount() function fails with FR_INT_ERR.
*/

int ff_cre_syncobj (    /* 1:Function succeeded, 0:Could not create due to any error */
    BYTE vol,           /* Corresponding logical drive being processed */
    _SYNC_t* sobj       /* Pointer to return the created sync object */
)
{
    *sobj = xSemaphoreCreateMutex();    /* FreeRTOS */

    return (int)(*sobj != NULL);
}




/*------------------------------------------------------------------------*/
/* Delete a Synchronization Object                                        */
/*------------------------------------------------------------------------*/
/* This function is called in f_mount() function to delete a synchronization
/  object that created with ff_cre_syncobj() function. When a 0 is
/  returned, the f_mount() function fails with FR_INT_ERR.
*/

int ff_del_syncobj (    /* 1:Function succeeded, 0:Could not delete due to any error */
    _SYNC_t sobj        /* Sync object tied to the logical drive to be deleted */
)
{
    vSemaphoreDelete(sobj);     /* FreeRTOS */

    return 1;
}




/*------------------------------------------------------------------------*/
/* Request Grant to Access the Volume                                     */
/*------------------------------------------------------------------------*/
/* This function is called on entering file functions to lock the volume.
/  When a FALSE is returned, the file function fails with FR_TIMEOUT.
*/

int ff_req_grant (  /* TRUE:Got a grant to access the volume, FALSE:Could not get a grant */
    _SYNC_t sobj    /* Sync object to wait */
)
{
    return (int)(xSemaphoreTake(sobj, _FS_TIMEOUT) == pdTRUE);  /* FreeRTOS */
}




/*------------------------------------------------------------------------*/
/* Release Grant to Access the Volume                                     */
/*------------------------------------------------------------------------*/
/* This function is called on leaving file functions to unlock the volume.
*/

void ff_rel_grant (
    _SYNC_t sobj    /* Sync object to be signaled */
)
{
    xSemaphoreGive(sobj);   /* FreeRTOS */
}




#endif  // FF_FS_REENTRANT




} // extern "C"

} // namespace xarmlib
