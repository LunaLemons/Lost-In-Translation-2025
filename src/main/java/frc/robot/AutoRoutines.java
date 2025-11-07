package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Gantry.Elevator;
import frc.robot.subsystems.Gantry.Hinge;
import frc.robot.subsystems.Gantry.RollersEndEffector;
import frc.robot.subsystems.Gantry.RollersStatic;






public class AutoRoutines {
    private final AutoFactory m_factory;
    private final Elevator elevator;
    private final Hinge hinge;
    private final RollersStatic rollers;
    private final RollersEndEffector rollersendeffector;


    public AutoRoutines(AutoFactory factory, Elevator elevator, Hinge hinge, RollersStatic rollers, RollersEndEffector rollersendeffector) {
        m_factory = factory;
        this.elevator = elevator;
        this.hinge = hinge;
        this.rollers = rollers;
        this.rollersendeffector = rollersendeffector;

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



        routine.active().onTrue(
        Commands.sequence(
            test1coral.resetOdometry(),
            test1coral.cmd()

        )
    );
    
    // i feel as though there must be a better way to do this
    test1coral.atTime("L1").onTrue(elevator.Setpoints(45.0));
    test1coral.atTime("L1").onTrue(hinge.Setpoints(15));
    test1coral.atTime("L1").onTrue(new InstantCommand(() -> hinge.setTargetAngle(15), hinge));

    test1coral.atTime("L1.1").onTrue(rollers.roller(-7.0));
    test1coral.atTime("L1.1").onTrue(rollersendeffector.rollerendeffector(-20.0));

    
    test1coral.atTime("Reset").onTrue(elevator.Setpoints(0.0));
    test1coral.atTime("Reset").onTrue(hinge.Setpoints(-3));
    test1coral.atTime("Reset").onTrue(new InstantCommand(() -> hinge.setTargetAngle(-3), hinge));
    test1coral.atTime("Reset").onTrue(rollers.roller(-7.0));
    test1coral.atTime("Reset").onTrue(rollersendeffector.rollerendeffector(-7.0));


     (test1coral.done()).onTrue(Commands.sequence(

    // // alternatively, use thread.sleep(4)
    // // (this is a very funny joke please laugh)
     Commands.waitSeconds(4), 
      
     test2coral.cmd()));

     test2coral.atTime("LiftScore2").onTrue(elevator.Setpoints(45.0));
     test2coral.atTime("LiftScore2").onTrue(hinge.Setpoints(15));
     test1coral.atTime("LiftScore2").onTrue(new InstantCommand(() -> hinge.setTargetAngle(15), hinge));
     test2coral.atTime("RollerScore2").onTrue(rollers.roller(-7.0));
     test1coral.atTime("RollerScore2").onTrue(rollersendeffector.rollerendeffector(-20.0));





        return routine;
    }
}