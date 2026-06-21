/*
 * This file is part of INAV.
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
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

extern "C" {
    #include "platform.h"
    #include "common/axis.h"
    #include "drivers/time.h"
    #include "io/rangefinder.h"
    #include "drivers/compass/compass.h"
    #include "drivers/compass/compass_msp.h"

    // Stub for millis
    uint32_t stubUptimeMs = 1000;
    uint32_t millis(void) {
        return stubUptimeMs;
    }

    uint32_t stateFlags = 0;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(BetalinkSensorsTest, TestRangefinderUpdate)
{
    // Initially, no data
    EXPECT_EQ(RANGEFINDER_NO_NEW_DATA, rangefinderMSPVtable.read());

    // Update with 1230 mm (should convert to 123 cm)
    mspRangefinderUpdateData(1230);
    EXPECT_EQ(123, rangefinderMSPVtable.read());

    // After read, it should return NO_NEW_DATA again
    EXPECT_EQ(RANGEFINDER_NO_NEW_DATA, rangefinderMSPVtable.read());
}

TEST(BetalinkSensorsTest, TestCompassUpdate)
{
    magDev_t magDev;

    // Detect compass driver
    bool detected = mspMagDetect(&magDev);
    EXPECT_TRUE(detected);

    // Update compass with values
    stubUptimeMs = 2000;
    mspMagUpdateData(100, 200, 300);

    // Read the values
    bool success = magDev.read(&magDev);
    EXPECT_TRUE(success);

    // Ensure raw variables are populated
    EXPECT_NE(0.0f, magDev.magADCRaw[X]);
    EXPECT_NE(0.0f, magDev.magADCRaw[Y]);
    EXPECT_NE(0.0f, magDev.magADCRaw[Z]);
}
