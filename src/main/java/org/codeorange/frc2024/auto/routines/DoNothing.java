package org.codeorange.frc2024.auto.routines;

import org.codeorange.frc2024.auto.AutoEndedException;
import org.codeorange.frc2024.auto.actions.ShootFromGround;

public class DoNothing extends BaseRoutine {
    @Override
    protected void routine() throws AutoEndedException {
        runAction(new ShootFromGround(54));
    }
}
