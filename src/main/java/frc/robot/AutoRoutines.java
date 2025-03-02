package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine lineyippee() {
        final AutoRoutine routine = m_factory.newRoutine("LineYippee");
        final AutoTrajectory lineyippee = routine.trajectory("LineYippee");

        routine.active().onTrue(
            lineyippee.resetOdometry()
                .andThen(lineyippee.cmd())
        );
        return routine;
    }
}