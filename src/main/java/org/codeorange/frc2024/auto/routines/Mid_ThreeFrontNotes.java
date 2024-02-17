package org.codeorange.frc2024.auto.routines;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import org.codeorange.frc2024.auto.actions.*;
import org.codeorange.frc2024.robot.Robot;
import org.codeorange.frc2024.subsystem.Superstructure;
import org.codeorange.frc2024.subsystem.drive.Drive;

import java.util.List;

public class Mid_ThreeFrontNotes extends BaseRoutine {
    private final Drive drive;

    final ChoreoTrajectory driveToFirstNote;
    final ChoreoTrajectory driveToSecondNote;
    final ChoreoTrajectory driveToThirdNote;


    public Mid_ThreeFrontNotes() {
        driveToFirstNote = Choreo.getTrajectory("Mid_ThreeFrontNotes.1");
        driveToSecondNote = Choreo.getTrajectory("Mid_ThreeFrontNotes.2");
        driveToThirdNote = Choreo.getTrajectory("Mid_ThreeFrontNotes.3");
        drive = Robot.getDrive();
    }
    @Override
    protected void routine() {
        runAction(new ResetOdometry(driveToFirstNote.sample(0, Robot.isRed())));
        runAction(new SeriesAction(
                new Shoot(),
                new ParallelAction(
                        new DrivePath(driveToFirstNote),
                        new GroundIntake()
                ),
                new Shoot(),
                new ParallelAction(
                        new DrivePath(driveToSecondNote),
                        new GroundIntake()
                ),
                new Shoot(),
                new ParallelAction(
                        new DrivePath(driveToThirdNote),
                        new GroundIntake()
                ))
        );
    }
}
