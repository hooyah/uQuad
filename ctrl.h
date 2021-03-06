//
// This code is distributed under the GNU General Public License
// which can be found at http://www.gnu.org/licenses/gpl.txt
// It is meant for information purposes only and is in no way guaranteed
// to be accurate, bug free or even working.
// Use it at your own risk.
// 

#ifndef _CTRL_GLOBAL_H
#define _CTRL_GLOBAL_H

#include "avrlibtypes.h"

extern volatile BOOL accu_sampling;


inline void startGyro() { accu_sampling = FALSE; }

u08 rpm_to_duty(u08 motorNo, s16 rpmPercent);

//void calc_tilt_roll(s16 ax, s16 ay, s16 az, s08 *t, s08 *r);
void calculate_balance(const u08 Thrust, const s08 Yaw, const s08 Pitch, const s08 Roll);





#endif
