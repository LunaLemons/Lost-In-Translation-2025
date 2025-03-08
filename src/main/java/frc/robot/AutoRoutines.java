package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Gantry.Elevator;
import frc.robot.subsystems.Gantry.Hinge;
import frc.robot.subsystems.Gantry.Rollers;






public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine lineyippee() {
        final AutoRoutine routine = m_factory.newRoutine("line-yippee");
        final AutoTrajectory lineyippee = routine.trajectory("line-yippee");

        routine.active().onTrue(
            lineyippee.resetOdometry()
                .andThen(lineyippee.cmd())
        );
        return routine;
    }
    public AutoRoutine test1coral() {
        final AutoRoutine routine = m_factory.newRoutine("test1coral");
        final AutoTrajectory test1coral = routine.trajectory("ihatejacky",0);
        final AutoTrajectory test2coral = routine.trajectory("ihatejacky",1);



        final Elevator elevator = new Elevator();
        final Hinge hinge = new Hinge();
        final Rollers rollers = new Rollers();




        routine.active().onTrue(
        Commands.sequence(
            test1coral.resetOdometry(),
            test1coral.cmd()

        )
    );

    test1coral.atTime("L1").onTrue(elevator.Setpoints(20));
    test1coral.atTime("L1").onTrue(hinge.Setpoints(-135));
    test1coral.atTime("L1.1").onTrue(rollers.roller(-40.0));
    
    test1coral.atTime("Reset").onTrue(elevator.Setpoints(124));
    test1coral.atTime("Reset").onTrue(hinge.Setpoints(-30));
    test1coral.atTime("Reset").onTrue(rollers.roller(-3.0));

    (test1coral.done()).onTrue(Commands.sequence(
    Commands.waitSeconds(4),    
    test2coral.cmd()));

    test2coral.atTime("LiftScore2").onTrue(elevator.Setpoints(20));
    test2coral.atTime("LiftScore2").onTrue(hinge.Setpoints(-135));
    test2coral.atTime("RollerScore2").onTrue(rollers.roller(-20.0));




        return routine;
    }
}