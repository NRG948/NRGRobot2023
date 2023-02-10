package frc.robot.commands;

//import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ChaseTagCommand extends CommandBase {
    private static final int TAG_TO_CHASE = 3;
    private static final Transform3d TAG_TO_GOAL = new Transform3d(new Translation3d(1, 0, 0),
            new Rotation3d(0.0, 0.0, Math.PI));

    // Physical location of the camera on the robot, relative to the center of the
    // robot.
    public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
            new Translation3d(Units.inchesToMeters(-28.0), Units.inchesToMeters(2.0), Units.inchesToMeters(-25.6)),
            new Rotation3d()); // change the x distance based on robot
    public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();

    private final PhotonVisionSubsystem photonVision;
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Pose2d> poseProvider;

    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController omegaController;

    private Pose2d goalPose;
    private PhotonTrackedTarget lastTarget;

    public ChaseTagCommand(
            PhotonVisionSubsystem photonCamera,
            SwerveSubsystem swerveSubsystem,
            Supplier<Pose2d> poseProvider) {
        this.photonVision = photonCamera;
        this.swerveSubsystem = swerveSubsystem;
        this.poseProvider = poseProvider;

        
        addRequirements(swerveSubsystem, photonCamera);
    }
    
    @Override
    public void initialize() {
        goalPose = null;
        lastTarget = null;
        var robotPose = poseProvider.get();
        xController = new ProfiledPIDController(1, 0, 0, swerveSubsystem.getDriveConstraints());
        yController = new ProfiledPIDController(1, 0, 0, swerveSubsystem.getDriveConstraints());
        omegaController = new ProfiledPIDController(1, 0, 0, swerveSubsystem.getRotationalConstraints());
        xController.setTolerance(0.1);
        yController.setTolerance(0.1);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-1, 1);
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }
    
    @Override
    public void execute() {
        var robotPose2d = poseProvider.get();
        var robotPose = new Pose3d(
                robotPose2d.getX(),
                robotPose2d.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        if (photonVision.hasTargets()) {
            // Find the tag we want to chase
            var targetOpt = photonVision.getTargets().stream()
                    .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
                    .findFirst();
            if (targetOpt.isPresent()) {
                var target = targetOpt.get();

                if (!target.equals(lastTarget)) {
                    // This is new target data, so recalculate the goal
                    lastTarget = target;

                    // Get the transformation from the camera to the tag (in 2d)
                    var camToTarget = target.getBestCameraToTarget();

                    // Transform the robot's pose to find the tag's pose
                    var cameraPose = robotPose.transformBy(CAMERA_TO_ROBOT);
                    var targetPose = cameraPose.transformBy(camToTarget);

                    // Transform the tag's pose to set our goal
                    goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

                    // Drive
                    xController.setGoal(goalPose.getX());
                    yController.setGoal(goalPose.getY());
                    omegaController.setGoal(goalPose.getRotation().getRadians());
                }
            }
        }

        if (lastTarget == null) {
            swerveSubsystem.stopMotors();

            return;
        }

        var xSpeed = xController.calculate(robotPose.getX());
        if (xController.atGoal()) {
            xSpeed = 0;
        }

        var ySpeed = yController.calculate(robotPose.getY());
        if (yController.atGoal()) {
            ySpeed = 0;
        }

        var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        }

        swerveSubsystem.setChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopMotors();
    }

}
