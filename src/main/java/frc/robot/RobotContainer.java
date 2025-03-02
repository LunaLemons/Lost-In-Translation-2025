// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Gantry.Elevator;
import frc.robot.subsystems.Gantry.Hinge;
import frc.robot.subsystems.Gantry.Rollers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.AlgaeArm.AlgaeHinge;
import frc.robot.subsystems.AlgaeArm.AlgaeRollers;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTag;


public class RobotContainer {
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController drivercontrol = new CommandXboxController(0);
    private final CommandXboxController codrivercontrol = new CommandXboxController(1);


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Elevator elevator = new Elevator();
    private final Hinge hinge = new Hinge();
    private final Rollers rollers = new Rollers();
    private final AlgaeHinge algaeHinge = new AlgaeHinge();
    private final AlgaeRollers algaeRollers = new AlgaeRollers();





    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("lineyippee", autoRoutines::lineyippee);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-drivercontrol.getLeftY() * MaxSpeed * (1 - ((elevator.getElevatorHeight() + 1) / 501))  ) // Drive forward with negative Y (forward)
                    .withVelocityY(-drivercontrol.getLeftX() * MaxSpeed * (1 - ((elevator.getElevatorHeight() + 1) / 501))  ) // Drive left with negative X (left)
                    .withRotationalRate(-drivercontrol.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        drivercontrol.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-drivercontrol.getLeftY(), -drivercontrol.getLeftX()))
        ));

        //joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //    forwardStraight.withVelocityX(0.5).withVelocityY(0))
       // );
        //joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //    forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        //);

        // climber redundant
        //joystick.a().whileTrue(climber.Setpoints(0));
        //joystick.x().whileTrue(climber.Setpoints(-90));

        // Elevator Setpoints
        codrivercontrol.povUp().whileTrue(elevator.Setpoints(250));
        codrivercontrol.povLeft().whileTrue(elevator.Setpoints(119));
        codrivercontrol.povRight().whileTrue(elevator.Setpoints(100));


        codrivercontrol.povDown().whileTrue(elevator.Setpoints(18));

        // Intake Angle
        // codrivercontrol.leftBumper().whileTrue(hinge.Setpoints(50));
        codrivercontrol.leftBumper().whileTrue(hinge.Setpoints(-30));

        codrivercontrol.rightBumper().whileTrue(hinge.Setpoints(-100));

        codrivercontrol.leftBumper().and(codrivercontrol.rightBumper()).whileTrue(hinge.Setpoints(-135));




        // Intake / Output Coral
        
        codrivercontrol.leftTrigger().whileTrue(rollers.roller(-2.0));
        codrivercontrol.rightTrigger().whileTrue(rollers.roller(-20.0));


        // yeah this doesn't either
        codrivercontrol.leftTrigger().and(codrivercontrol.rightTrigger()).whileTrue(rollers.roller(0.0));
    

        // Algae Control

        codrivercontrol.a().whileTrue(algaeRollers.algaeSpin(-15.0));
        codrivercontrol.b().whileTrue(algaeRollers.algaeSpin(15.0));
        codrivercontrol.a().and(codrivercontrol.b()).whileTrue(algaeRollers.algaeSpin(0.0));

        codrivercontrol.x().whileTrue(algaeHinge.Setpoints(50));
        codrivercontrol.y().whileTrue(algaeHinge.Setpoints(40));

        drivercontrol.a().whileTrue(algaeHinge.Setpoints(5));
        drivercontrol.b().whileTrue(algaeHinge.Setpoints(140));

        drivercontrol.x().whileTrue(drivetrain.autoAlign());






        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        drivercontrol.back().and(drivercontrol.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        drivercontrol.back().and(drivercontrol.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        drivercontrol.start().and(drivercontrol.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        drivercontrol.start().and(drivercontrol.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on D-pad up
        drivercontrol.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        

    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }
    
}
