package frc.robot.subsystems;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;

public class AutoAlign extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
    PoseReader poseReader = new PoseReader();



    

    // Load official AprilTag layout
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // PID controllers for smooth movement
    private final PIDController xController = new PIDController(0.1, 0, 0);
    private final PIDController yController = new PIDController(0.1, 0, 0);
    private final PIDController thetaController = new PIDController(0.05, 0, 0);

    public AutoAlign(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Command alignToNearestTag() {
        return drivetrain.applyRequest(() -> {
            Pose2d robotPose = poseReader.getRobotPose();; // Get the robot's current position
            Pose2d nearestTag = findClosestAprilTag(robotPose);

            if (nearestTag == null) {
                return drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0); // Stop if no tag is found
            }

            // Calculate errors
            Translation2d translationError = nearestTag.getTranslation().minus(robotPose.getTranslation());
            double angleError = nearestTag.getRotation().getRadians() - robotPose.getRotation().getRadians();

            // PID outputs
            double xSpeed = xController.calculate(0, translationError.getX());
            double ySpeed = yController.calculate(0, translationError.getY());
            double thetaSpeed = thetaController.calculate(0, angleError);

            // Generate field-centric drive request
            return drive.withVelocityX(xSpeed)
                        .withVelocityY(ySpeed)
                        .withRotationalRate(thetaSpeed);
        });
    }

    private Pose2d findClosestAprilTag(Pose2d robotPose) {
        return kTagLayout.getTags().stream()
            .map(tag -> kTagLayout.getTagPose(tag.ID))
            .filter(Optional::isPresent)
            .map(Optional::get)
            .map(Pose3d::toPose2d) // Convert 3D to 2D
            .min((tag1, tag2) -> Double.compare(
                tag1.getTranslation().getDistance(robotPose.getTranslation()),
                tag2.getTranslation().getDistance(robotPose.getTranslation())
            ))
            .orElse(null);
    }

    public class PoseReader {
        private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
        private final NetworkTable driveStateTable = inst.getTable("DriveState");
        private final StructSubscriber<Pose2d> drivePoseSubscriber =
            driveStateTable.getStructTopic("Pose", Pose2d.struct).subscribe(new Pose2d());
    
        public Pose2d getRobotPose() {
            return drivePoseSubscriber.get();
        }
    }
}
