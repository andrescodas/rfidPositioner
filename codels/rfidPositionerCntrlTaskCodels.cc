/**
 ** rfidPositionerCntrlTaskCodels.cc
 **
 ** Codels used by the control task rfidPositionerCntrlTask
 **
 ** Author: 
 ** Date: 
 **
 **/

#include <portLib.h>

#include "server/rfidPositionerHeader.h"


/*------------------------------------------------------------------------
 * rfidPositionerSetPositionCntrl  -  control codel of CONTROL request SetPosition
 *
 * Description:    
 * Report: OK
 *
 * Returns:    OK or ERROR
 */

STATUS
rfidPositionerSetPositionCntrl(POSITION *position, int *report)
{
  /* ... add your code here ... */
  return OK;
}

/*------------------------------------------------------------------------
 * rfidPositionerSetVarianceCntrl  -  control codel of CONTROL request SetVariance
 *
 * Description:    
 * Report: OK
 *                 S_rfidPositioner_NOT_POSITIVE_DEFINITE
 *
 * Returns:    OK or ERROR
 */

STATUS
rfidPositionerSetVarianceCntrl(POSITION_ERROR *estimationError, int *report)
{
  /* ... add your code here ... */
  return OK;
}


