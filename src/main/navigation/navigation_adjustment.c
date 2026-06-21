/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/vector.h"

#include "fc/runtime_config.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

bool navAdjustTargetAltitude(int32_t deltaCm)
{
    // Must be armed and altitude control must be active
    if (!ARMING_FLAG(ARMED) || !(navGetCurrentStateFlags() & NAV_CTL_ALT)) {
        return false;
    }

    const float currentZ = navGetCurrentActualPositionAndVelocity()->pos.z;
    const float newTargetZ = currentZ + deltaCm;

    if (FLIGHT_MODE(NAV_WP_MODE)) {
        // Adjust waypoint target altitude
        posControl.activeWaypoint.pos.z = newTargetZ;
        // Re-anchor climb/descent calculations to the current position to prevent sudden jumps
        posControl.wpInitialAltitude = currentZ;
        
        setDesiredPosition(&posControl.activeWaypoint.pos, 0, NAV_POS_UPDATE_Z);
    } else {
        // For other altitude holding modes (PosHold, RTH, Cruise, etc.)
        updateClimbRateToAltitudeController(0, newTargetZ, ROC_TO_ALT_TARGET);
    }

    return true;
}
