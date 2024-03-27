package org.codeorange.frc2024.utility;

import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.codeorange.frc2024.robot.Constants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class WristMaxAngleTest {
    @BeforeEach
        // this method will run before each test
    void setup() {
    }

    @AfterEach
        // this method will run after each test
    void shutdown() throws Exception {
    }

    double findMaxRotationDegrees(double elevatorPositionInches, double armPositionDegrees) {
        double wristLengthInches = 11.191;
        double maxRotation = -0.5;
        double elevatorPivotToWristCarriageOffset = 10.5;

        // special case for ground intake
        if (elevatorPositionInches >= SS_MIDINTAKE_ELEVATOR && armPositionDegrees <= 0.0) {
            return SS_GROUNDINTAKE_WRIST * 360;
        }

        elevatorPositionInches += elevatorPivotToWristCarriageOffset;

        //checks if the wrist can rotate freely (its length is less than its distance from the base). if not, it does stuff
        double verticalOffsetFromRobot = elevatorPositionInches * Math.sin(Units.degreesToRadians(armPositionDegrees));

        if(verticalOffsetFromRobot > wristLengthInches){
            return maxRotation * 360;
        }

        // maxRotation is the degrees that wrist would have to go to to form a triangle with the ground
        // (so that it can't move into the ground). this is negative
        maxRotation = Units.radiansToDegrees(Math.acos(verticalOffsetFromRobot / wristLengthInches)) - 90;
        //clamps position to between the maximum positive and negative rotation
        return maxRotation;
    }

    @Test
    void testWithDegrees() {
        assertEquals(0, Math.round(findMaxRotationDegrees(0, 0)));
        assertEquals(-9, Math.round(findMaxRotationDegrees(0, 10)));
        assertEquals(-28, Math.round(findMaxRotationDegrees(0, 30)));
        assertEquals(-37, Math.round(findMaxRotationDegrees(0, 40)));
        assertEquals(-46, Math.round(findMaxRotationDegrees(0, 50)));
        assertEquals(-54, Math.round(findMaxRotationDegrees(0, 60)));
        assertEquals(-62, Math.round(findMaxRotationDegrees(0, 70)));
        assertEquals(-68, Math.round(findMaxRotationDegrees(0, 80)));
        assertEquals(-70, Math.round(findMaxRotationDegrees(0, 90)));

        // elevator extended 6 inches
        assertEquals(-47, Math.round(findMaxRotationDegrees(6, 30)));
        assertEquals(-71, Math.round(findMaxRotationDegrees(6, 40)));
        assertEquals(-180, Math.round(findMaxRotationDegrees(6, 50)));

        // ground intake special case
        assertEquals(0, Math.round(findMaxRotationDegrees(6, 0)));
        assertEquals(0, Math.round(findMaxRotationDegrees(10, 0)));
        assertEquals(0, Math.round(findMaxRotationDegrees(11, 0)));
        assertEquals(0, Math.round(findMaxRotationDegrees(12, 0)));
        assertEquals(Math.round(SS_GROUNDINTAKE_WRIST * 360), Math.round(findMaxRotationDegrees(13.5, 0)));
        assertEquals(Math.round(SS_GROUNDINTAKE_WRIST * 360), Math.round(findMaxRotationDegrees(15, 0)));
    }
}
