import org.codeorange.frc2024.utility.MathUtil;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class MathUtilChecker {
    @BeforeEach
        // this method will run before each test
    void setup() {
    }

    @AfterEach
        // this method will run after each test
    void shutdown() throws Exception {
    }

    @Test
    void normalize() {
        assertEquals(MathUtil.normalize(180, 0, 360), 180);
        assertEquals(MathUtil.normalize(-180, 0, 360), 180);
        assertEquals(MathUtil.normalize(-540, 0, 360), 180);
        assertEquals(MathUtil.normalize(540, 0, 360), 180);
        assertEquals(MathUtil.normalize(90, -180, 180), 90);
        assertEquals(MathUtil.normalize(270, -180, 180), -90);
        assertEquals(MathUtil.normalize(540, -180, 180), 180);
        assertEquals(MathUtil.normalize(-540, -180, 180), 180);
        assertEquals(MathUtil.normalize(-539, -180, 180), -179);
    }
}
