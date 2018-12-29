/*
 * AC_Flowermeter.cpp
 *
 *  Created on: 2016-4-12
 *      Author: liwh
 */

#include "AC_Flowermeter_Backend.h"

/*
  base class constructor.
*/
AC_Flowermeter_Backend::AC_Flowermeter_Backend(AC_Flowermeter &ac_flower, AC_Flowermeter::FL_State &state):
	_ac_flower(ac_flower),
	_state(state)
{
}


