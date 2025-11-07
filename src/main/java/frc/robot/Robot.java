// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import java.util.concurrent.Flow.Publisher;

import org.photonvision.*;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.communication.NIRioStatus;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.CanBridge;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;










public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private final PhotonCamera camera;
  private final PhotonCamera camera2;
  private final PhotonCamera camera3;
  private final PhotonCamera camera4;


  private final boolean kUseLimelight = false;
  private Vision vision;
  private Vision vision2;
  private Vision vision3;
  private Vision vision4;

  public static final Transform3d kRobotToCam =
  new Transform3d(new Translation3d(-0.296516, -0.103341, 0.2075), new Rotation3d(0, 0, 2.5307));


  public static final Transform3d kRobotToCam2 =
  new Transform3d(new Translation3d(-0.296516, 0.103341, 0.2075), new Rotation3d(0, 0, -2.5307));

  public static final Transform3d kRobotToCam3 =
  new Transform3d(new Translation3d(0.115302, 0.29571, 0.303), new Rotation3d(0, 0, -0.3491));


  public static final Transform3d kRobotToCam4 =
  new Transform3d(new Translation3d(0.115170, -0.296294, 0.303), new Rotation3d(0, 0, 0.2618));
  private CommandSwerveDrivetrain drivetrain;

  private LaserCan lc;


  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
                            .getStructTopic("MyPose", Pose2d.struct).publish();
  StructPublisher<Pose2d> publisher2 = NetworkTableInstance.getDefault()
                            .getStructTopic("MyPose2", Pose2d.struct).publish();
  StructPublisher<Pose2d> publisher3 = NetworkTableInstance.getDefault()
                            .getStructTopic("MyPose3", Pose2d.struct).publish();
  StructPublisher<Pose2d> publisher4 = NetworkTableInstance.getDefault()
                            .getStructTopic("MyPose4", Pose2d.struct).publish();



  public Robot() {
    CanBridge.runTCP();
    m_robotContainer = new RobotContainer();
    camera = new PhotonCamera("uwu");
    camera2 = new PhotonCamera("rawr");
    camera3 = new PhotonCamera("Arducam_OV9782_USB_Camera (1)");
    camera4 = new PhotonCamera("Arducam_OV9782_USB_Camera");
    //alpha
    String cameraname = "uwu";

    //beta
    String camera2name = "rawr";

    //delta
    String camera3name = "Arducam_OV9782_USB_Camera (1)";

    //omega
    String camera4name = "Arducam_OV9782_USB_Camera";

    drivetrain = m_robotContainer.drivetrain;
    CameraServer.startAutomaticCapture();
    vision = new Vision(cameraname, kRobotToCam);
    vision2 = new Vision(camera2name, kRobotToCam2);
    vision3 = new Vision(camera3name, kRobotToCam3);
    vision4 = new Vision(camera4name, kRobotToCam4);

    PortForwarder.add(5800, "wobot.local", 2005);
    PortForwarder.add(5800, "lobot.local", 2006);

  }

  @Override
  public void robotInit() {
    lc = new LaserCan(19);

    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run();

    // Currently has the two cameras just fighting each other - not ideal :/ - pls remember to fix
     
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
       // Correct pose estimate with vision measurements
       if (vision != null) {

        var visionEst = vision.getEstimatedGlobalPose();
        visionEst.ifPresent(
                est -> {
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = vision.getEstimationStdDevs();
                    drivetrain.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);
                            publisher.set(est.estimatedPose.toPose2d());
                            System.out.println("vision (alpha) nominal :3");

                });

                
        }else{
         System.out.println("vision (alpha) offline :(");
         
        }

      

        if (vision2 != null) {
        
          var visionEst2 = vision2.getEstimatedGlobalPose();
          visionEst2.ifPresent(
                  est -> {
                      // Change our trust in the measurement based on the tags we can see
                      var estStdDevs = vision2.getEstimationStdDevs();
                      drivetrain.addVisionMeasurement(
                              est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);
                              publisher2.set(est.estimatedPose.toPose2d());
                              System.out.println("vision (beta) nominal :3");
        
                });
  
                  
          }else{
           System.out.println("vision (beta) offline :(");
           
          }

          if (vision3 != null) {
        
            var visionEst3 = vision3.getEstimatedGlobalPose();
            visionEst3.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = vision3.getEstimationStdDevs();
                        drivetrain.addVisionMeasurement(
                                est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);
                                publisher3.set(est.estimatedPose.toPose2d());
                                System.out.println("vision (delta) nominal :3");
          
                  });
    
                    
            }else{
             System.out.println("vision (delta) offline :(");
             
            }

            if (vision4 != null) {
        
              var visionEst4 = vision4.getEstimatedGlobalPose();
              visionEst4.ifPresent(
                      est -> {
                          // Change our trust in the measurement based on the tags we can see
                          var estStdDevs = vision4.getEstimationStdDevs();
                          drivetrain.addVisionMeasurement(
                                  est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);
                                  publisher4.set(est.estimatedPose.toPose2d());
                                  System.out.println("vision (omega) nominal :3");
            
                    });
      
                      
              }else{
               System.out.println("vision (omega) offline :(");
               
              }
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
