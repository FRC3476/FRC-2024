import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Demonstrate how WPILib class InterpolatingDoubleTreeMap works
 */
class InterpolationTest {

    static final InterpolatingDoubleTreeMap SHOOTER_ANGLE_LOW_FRONT = new InterpolatingDoubleTreeMap();
    static {
        // First number is the distance in feet. Second number is the shooter angle.
        SHOOTER_ANGLE_LOW_FRONT.put(0.0, 54.0);
        SHOOTER_ANGLE_LOW_FRONT.put(6.0, 27.0);
        SHOOTER_ANGLE_LOW_FRONT.put(12.0, 24.0);
        SHOOTER_ANGLE_LOW_FRONT.put(18.0, 22.0);
        SHOOTER_ANGLE_LOW_FRONT.put(24.0, 20.0);
    }

    static final InterpolatingDoubleTreeMap SHOOTER_ANGLE_LOW_BACK = new InterpolatingDoubleTreeMap();
    static {
        // First number is the distance in feet. Second number is the shooter angle.
        SHOOTER_ANGLE_LOW_BACK.put(0.0, 54.0);
        SHOOTER_ANGLE_LOW_BACK.put(6.0, 27.0);
        SHOOTER_ANGLE_LOW_BACK.put(12.0, 24.0);
        SHOOTER_ANGLE_LOW_BACK.put(18.0, 22.0);
        SHOOTER_ANGLE_LOW_BACK.put(24.0, 20.0);
    }

    @BeforeEach // this method will run before each test
    void setup() {
    }

    @AfterEach // this method will run after each test
    void shutdown() throws Exception {
    }

    @Test // marks this method as a test
    void valuesThatWereMeasured() {
        assertEquals(20.0, SHOOTER_ANGLE_LOW_FRONT.get(24.0));
        assertEquals(54.0, SHOOTER_ANGLE_LOW_FRONT.get(0.0));
    }

    @Test // marks this method as a test
    void valuesThatWereNotMeasured() {
        // Test the angle halfway between two measured points
        assertEquals(27.0 + (54.0 - 27.0)/2.0, SHOOTER_ANGLE_LOW_FRONT.get(3.0));
        assertEquals(24.0 + (27.0 - 24.0)/2.0, SHOOTER_ANGLE_LOW_FRONT.get(9.0));
    }
    @Test // marks this method as a test
    void valueGreaterThanLargestMeasurement() {
        // The angle never goes lower than the last measurement made
        assertEquals(20.0, SHOOTER_ANGLE_LOW_FRONT.get(90.0));
    }

    @Test
    void valueVeryCloseToMeasuredValue() {
        double shooterAngle = SHOOTER_ANGLE_LOW_FRONT.get(18.7);
        System.out.print(shooterAngle);
        assertNotEquals(22.0, shooterAngle);
        assertTrue(shooterAngle < 22.0);
        assertTrue(shooterAngle > 21.7);
    }
}