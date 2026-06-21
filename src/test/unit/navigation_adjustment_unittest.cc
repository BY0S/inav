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

extern "C" {
    #include "platform.h"
    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/vector.h"
    #include "config/parameter_group.h"
    #include "config/parameter_group_ids.h"
    #include "navigation/navigation.h"
    #include "navigation/navigation_private.h"
    #include "fc/runtime_config.h"

    // Define posControl globally so the code under test can access it
    navigationPosControl_t posControl;

    // Stubs for navigation and runtime functions
    static fpVector3_t stubActualPosition;
    static navEstimatedPosVel_t stubPosAndVel;
    const navEstimatedPosVel_t* navGetCurrentActualPositionAndVelocity(void) {
        stubPosAndVel.pos = stubActualPosition;
        return &stubPosAndVel;
    }

    static uint32_t stubStateFlags = 0;
    navigationFSMStateFlags_t navGetCurrentStateFlags(void) {
        return (navigationFSMStateFlags_t)stubStateFlags;
    }

    uint32_t armingFlags = 0;
    uint32_t flightModeFlags = 0;

    bool setDesiredPositionCalled = false;
    fpVector3_t setDesiredPositionPos;
    uint32_t setDesiredPositionMask;
    void setDesiredPosition(const fpVector3_t * pos, int32_t yaw, navSetWaypointFlags_t useMask) {
        (void)yaw;
        setDesiredPositionCalled = true;
        if (pos) {
            setDesiredPositionPos = *pos;
        }
        setDesiredPositionMask = (uint32_t)useMask;
    }

    bool updateClimbRateToAltitudeControllerCalled = false;
    float updateClimbRateToAltitudeControllerTarget = 0;
    climbRateToAltitudeControllerMode_e updateClimbRateToAltitudeControllerMode;
    void updateClimbRateToAltitudeController(float desiredClimbRate, float targetAltitude, climbRateToAltitudeControllerMode_e mode) {
        (void)desiredClimbRate;
        updateClimbRateToAltitudeControllerCalled = true;
        updateClimbRateToAltitudeControllerTarget = targetAltitude;
        updateClimbRateToAltitudeControllerMode = mode;
    }

    // Unused config parameter groups
    PG_REGISTER(navConfig_t, navConfig, PG_NAV_CONFIG, 0);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

class NavigationAdjustmentTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset stubs and globals before each test
        memset(&posControl, 0, sizeof(posControl));
        stubActualPosition.x = 0;
        stubActualPosition.y = 0;
        stubActualPosition.z = 0;
        stubStateFlags = 0;
        armingFlags = 0;
        flightModeFlags = 0;
        setDesiredPositionCalled = false;
        memset(&setDesiredPositionPos, 0, sizeof(setDesiredPositionPos));
        setDesiredPositionMask = 0;
        updateClimbRateToAltitudeControllerCalled = false;
        updateClimbRateToAltitudeControllerTarget = 0;
    }
};

TEST_F(NavigationAdjustmentTest, TestUnarmedReturnsFalse)
{
    // Disarmed, but altitude control active
    armingFlags = 0;
    stubStateFlags = NAV_CTL_ALT;

    EXPECT_FALSE(navAdjustTargetAltitude(100));
}

TEST_F(NavigationAdjustmentTest, TestNoAltControlReturnsFalse)
{
    // Armed, but altitude control inactive
    armingFlags = ARMED;
    stubStateFlags = 0;

    EXPECT_FALSE(navAdjustTargetAltitude(100));
}

TEST_F(NavigationAdjustmentTest, TestAdjustAltitudeInPosHold)
{
    // Armed and altitude control active
    armingFlags = ARMED;
    stubStateFlags = NAV_CTL_ALT;
    flightModeFlags = 0; // Not waypoint mode

    // Set current estimated altitude to 1000 cm
    stubActualPosition.z = 1000.0f;

    // Adjust up by 250 cm
    EXPECT_TRUE(navAdjustTargetAltitude(250));

    EXPECT_TRUE(updateClimbRateToAltitudeControllerCalled);
    EXPECT_FLOAT_EQ(1250.0f, updateClimbRateToAltitudeControllerTarget);
    EXPECT_EQ(ROC_TO_ALT_TARGET, updateClimbRateToAltitudeControllerMode);
}

TEST_F(NavigationAdjustmentTest, TestAdjustAltitudeInWaypointMode)
{
    // Armed, altitude control active, waypoint mode active
    armingFlags = ARMED;
    stubStateFlags = NAV_CTL_ALT;
    flightModeFlags = NAV_WP_MODE;

    // Set current estimated altitude to 1000 cm
    stubActualPosition.z = 1000.0f;
    posControl.activeWaypoint.pos.z = 1200.0f; // Target was 12 meters

    // Adjust down by 300 cm
    EXPECT_TRUE(navAdjustTargetAltitude(-300));

    // For WP mode, it should re-anchor target altitude and initial altitude relative to current Z
    EXPECT_FLOAT_EQ(700.0f, posControl.activeWaypoint.pos.z); // 1000 - 300
    EXPECT_FLOAT_EQ(1000.0f, posControl.wpInitialAltitude);

    EXPECT_TRUE(setDesiredPositionCalled);
    EXPECT_FLOAT_EQ(700.0f, setDesiredPositionPos.z);
    EXPECT_EQ(NAV_POS_UPDATE_Z, setDesiredPositionMask);
}
