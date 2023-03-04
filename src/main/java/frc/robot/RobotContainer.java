// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants.XboxControllerPort;
import frc.robot.Constants.RobotConstants.PWMPort;
import frc.robot.commands.AutoBalanceOnChargeStation;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveWithController;
import frc.robot.commands.IntakeByController;
import frc.robot.commands.RaiseElevatorWithController;
import frc.robot.commands.Scoring;
import frc.robot.commands.TiltElevatorWithController;
import frc.robot.subsystems.ElevatorAngleSubsystem.ElevatorAngle;
import frc.robot.subsystems.ElevatorSubsystem.GoalState;
import frc.robot.subsystems.Subsystems;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@RobotPreferencesLayout(groupName = "Preferences", column = 0, row = 0, width = 2, height = 1)
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    private final Subsystems subsystems = new Subsystems();

    private static AddressableLEDs leds = new AddressableLEDs(PWMPort.LED, 51);

    // Operator Xbox controllers.
    private final CommandXboxController driveController = new CommandXboxController(XboxControllerPort.DRIVER);
    private final CommandXboxController manipulatorController = new CommandXboxController(
            XboxControllerPort.MANIPULATOR);

    private final RobotAutonomous autonomous = new RobotAutonomous(subsystems);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        subsystems.drivetrain.setDefaultCommand(new DriveWithController(subsystems.drivetrain, driveController));
        // Manual commands for elevator testing. Not to be used by drivers.
        subsystems.elevator
                .setDefaultCommand(new RaiseElevatorWithController(subsystems.elevator, manipulatorController));
        subsystems.elevatorAngle
                .setDefaultCommand(new TiltElevatorWithController(subsystems.elevatorAngle, manipulatorController));
        subsystems.intake
                .setDefaultCommand(new IntakeByController(subsystems.intake, manipulatorController));

        initShuffleboard();

        configureCommandBindings();

        leds.start();
        leds.setColor(new Color8Bit(255, 0, 0));
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureCommandBindings() {

        driveController.a().onTrue(
                new DriveStraight(subsystems.drivetrain, new Translation2d(3.0, Math.toRadians(0)), 1.0)
                        .until(() -> Math.abs(subsystems.drivetrain.getTilt().getDegrees()) > 9.0)
                        .andThen(new AutoBalanceOnChargeStation(subsystems.drivetrain)));
        driveController.b().onTrue(
                new DriveStraight(subsystems.drivetrain, new Translation2d(-3.0, Math.toRadians(0)), 1.0)
                        .until(() -> Math.abs(subsystems.drivetrain.getTilt().getDegrees()) > 9.0)
                        .andThen(new AutoBalanceOnChargeStation(subsystems.drivetrain)));
        driveController.x().whileTrue(new AutoBalanceOnChargeStation(subsystems.drivetrain));

        driveController.y().onTrue(Commands.runOnce(() -> {
            int red = (int) (255 * Math.random());
            int green = (int) (255 * Math.random());
            int blue = (int) (255 * Math.random());
            leds.setColor(new Color8Bit(red, green, blue));
        }));

        driveController.rightBumper().whileTrue(new ChaseTagCommand(subsystems.photonVision, subsystems.drivetrain));

        // TODO: Once we're done with testing the autonomous motion commands, change
        // this to call resetOrientation().
        driveController.start().onTrue(Commands.runOnce(() -> subsystems.drivetrain.resetPosition(new Pose2d())));

        manipulatorController.x()
                .onTrue(Commands.runOnce(() -> subsystems.elevatorAngle.setGoalAngle(ElevatorAngle.ACQUIRING)));
        manipulatorController.b()
                .onTrue(Commands.runOnce(() -> subsystems.elevatorAngle.setGoalAngle(ElevatorAngle.SCORING)));
        manipulatorController.a().onTrue(Commands.runOnce(() -> subsystems.elevator.setGoal(GoalState.ACQUIRE)));
        manipulatorController.y().onTrue(Commands.runOnce(() -> subsystems.elevator.setGoal(GoalState.SCORE_MID)));
        manipulatorController.rightBumper()
                .onTrue(Commands.runOnce(() -> subsystems.elevator.setGoal(GoalState.SCORE_HIGH)));
        // manipulatorController.rightTrigger().onTrue(Commands.runOnce(() ->
        // subsystems.elevator.setGoal(GoalState.SCORE_HIGH)));
        manipulatorController.leftBumper().whileTrue(Commands.sequence(
                Commands.waitUntil(() -> subsystems.photonVision.hasTargets()),
                new ProxyCommand(() -> Scoring.scoreToGrid(subsystems, manipulatorController.getHID()))));
    }

    /**
     * Returns the autonomous command selected in the chooser.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonomous.getAutonomousCommand();
    }

    /** Adds the Shuffleboard tabs for the robot. */
    private void initShuffleboard() {
        // The "Operator" tab contains UI elements that enable the drive team to set up
        // and operate the robot.
        ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator");

        autonomous.addShuffleboardLayout(operatorTab)
                .withPosition(0, 0)
                .withSize(2, 3);

        VideoSource video = new HttpCamera(
                "photonvision_Port_1182_MJPEG_Server", "http://10.9.48.11:1182/?action=stream",
                HttpCameraKind.kMJPGStreamer);
        operatorTab.add("PhotonVision", video)
                .withWidget(BuiltInWidgets.kCameraStream)
                .withPosition(2, 0)
                .withSize(4, 3);

        ShuffleboardLayout gridLayout = operatorTab.getLayout("Grid", BuiltInLayouts.kGrid)
                .withPosition(6, 0)
                .withSize(3, 3);

        gridLayout.addBoolean("Left High", () -> manipulatorController.getHID().getPOV() == 315)
                .withPosition(0, 0);
        gridLayout.addBoolean("Left Mid", () -> manipulatorController.getHID().getPOV() == 270)
                .withPosition(0, 1);
        gridLayout.addBoolean("Left Low", () -> manipulatorController.getHID().getPOV() == 225)
                .withPosition(0, 2);
        gridLayout.addBoolean("Center High", () -> manipulatorController.getHID().getPOV() == 0)
                .withPosition(1, 0);
        gridLayout.addBoolean("Center Mid", () -> manipulatorController.getHID().getPOV() == -1)
                .withPosition(1, 1);
        gridLayout.addBoolean("Center Low", () -> manipulatorController.getHID().getPOV() == 180)
                .withPosition(1, 2);
        gridLayout.addBoolean("Right High", () -> manipulatorController.getHID().getPOV() == 45)
                .withPosition(2, 0);
        gridLayout.addBoolean("Right Mid", () -> manipulatorController.getHID().getPOV() == 90)
                .withPosition(2, 1);
        gridLayout.addBoolean("Right Low", () -> manipulatorController.getHID().getPOV() == 135)
                .withPosition(2, 2);

        // The "Preferences" tab UI elements that enable configuring robot-specific
        // settings.
        RobotPreferences.addShuffleBoardTab();

        // The subsystem-specific tabs are added for testing and should be disabled by
        // default.
        subsystems.drivetrain.addShuffleboardTab();
        subsystems.photonVision.addShuffleboardTab();
        subsystems.elevator.addShuffleBoardTab(
                subsystems.elevatorAngle::atAcquiringLimit,
                subsystems.elevatorAngle::atScoringLimit);
    }
}
