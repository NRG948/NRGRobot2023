package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ChaseTagCommand extends CommandBase {
    private static final int TAG_TO_CHASE = 3;

    /**
     * A transform from the AprilTag to the goal position. This should place the
     * robot 1m in front of the tag.
     */
    private static final Transform3d TAG_TO_GOAL = new Transform3d(
            new Translation3d(1, 0, 0),
            new Rotation3d(0.0, 0.0, Math.PI));

    private final AprilTagSubsystem photonVision;
    private final SwerveSubsystem swerveSubsystem;

    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController omegaController;

    private Pose2d goalPose;
    private PhotonTrackedTarget lastTarget;

    /**
     * Creates a new ChaseTagCommand.
     * 
     * @param photonVision    The PhotonVision subsystem.
     * @param swerveSubsystem The swerve drive subsystem.
     * @param poseProvider    Supplies the pose of the robot.
     */
    public ChaseTagCommand(AprilTagSubsystem photonVision, SwerveSubsystem swerveSubsystem) {
        this.photonVision = photonVision;
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem, photonVision);
    }

    @Override
    public void initialize() {
        goalPose = null;
        lastTarget = null;

        Constraints driveConstraints = swerveSubsystem.getDriveConstraints();

        xController = new ProfiledPIDController(1, 0, 0, driveConstraints);
        yController = new ProfiledPIDController(1, 0, 0, driveConstraints);
        omegaController = new ProfiledPIDController(1, 0, 0, swerveSubsystem.getRotationalConstraints());

        xController.setTolerance(0.1);
        yController.setTolerance(0.1);
        omegaController.setTolerance(Units.degreesToRadians(3));

        omegaController.enableContinuousInput(-1, 1);

        var robotPose = swerveSubsystem.getPosition();

        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        // Get the current robot pose and map it to a 3-dimensional coordinate space.
        var robotPose3d = swerveSubsystem.getPosition3d();

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

                    // Get the transformation from the camera to the tag
                    var camToTarget = target.getBestCameraToTarget();

                    // Transform the robot's pose to find the tag's pose
                    var cameraPose = robotPose3d.transformBy(RobotConstants.ROBOT_TO_BACK_CAMERA);
                    var targetPose = cameraPose.transformBy(camToTarget);

                    // Transform the tag's pose to set our goal
                    goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

                    // Set the PID controller goal states
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

        var xSpeed = xController.calculate(robotPose3d.getX());
        if (xController.atGoal()) {
            xSpeed = 0;
        }

        var ySpeed = yController.calculate(robotPose3d.getY());
        if (yController.atGoal()) {
            ySpeed = 0;
        }

        var omegaSpeed = omegaController.calculate(robotPose3d.toPose2d().getRotation().getRadians());
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        }

        swerveSubsystem.setChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose3d.toPose2d().getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopMotors();
    }

}
