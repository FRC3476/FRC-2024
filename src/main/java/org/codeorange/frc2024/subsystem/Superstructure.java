package org.codeorange.frc2024.subsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.drive.Drive;
import org.codeorange.frc2024.utility.Alert;
import org.codeorange.frc2024.utility.MathUtil;
import org.codeorange.frc2024.utility.net.editing.LiveEditableValue;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static org.codeorange.frc2024.robot.Constants.*;

public class Superstructure extends AbstractSubsystem {
    private static Drive drive;
    private static Superstructure superstructure = new Superstructure();
    private static BlinkinLEDController blinkin;
    //private static Vision vision;
    @AutoLogOutput(key = "Superstructure/Goal State")
    public boolean isFlipped = false;
    public boolean climberOut = false;
    public boolean manualOverride = false;

    private final Alert wristAlert = new Alert("WRIST WILL BREAK ITSELF IF ENABLED!!", Alert.AlertType.ERROR);

    private Superstructure() {
        super();
        drive = Robot.getDrive();
        blinkin = Robot.getBlinkin();
    }

    public double wantedAngle = 52;

}