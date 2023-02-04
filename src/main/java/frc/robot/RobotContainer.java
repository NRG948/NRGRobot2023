// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.nrg948.autonomous.Autonomous;
import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants.XboxControllerPort;
import frc.robot.Constants.RobotConstants.PWMPort;
import frc.robot.commands.AssistedBalanceOnChargeStation;
import frc.robot.commands.AutoBalanceOnChargeStation;
import frc.robot.commands.DriveWithController;
import frc.robot.subsystems.ClawSubsystem.Position;
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
  private final CommandXboxController manipulatorController = new CommandXboxController(XboxControllerPort.MANIPULATOR);


  private final SendableChooser<Command> autonomousCommandChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    subsystems.drivetrain.setDefaultCommand(new DriveWithController(subsystems.drivetrain, driveController));

    autonomousCommandChooser = Autonomous.getChooser(subsystems, "frc.robot");

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

    driveController.a().onTrue(new AutoBalanceOnChargeStation(subsystems.drivetrain, true));
    driveController.b().onTrue(new AutoBalanceOnChargeStation(subsystems.drivetrain, false));
    driveController.x().whileTrue(new AssistedBalanceOnChargeStation(subsystems.drivetrain));

    driveController.y().onTrue(new InstantCommand(() -> {
      int red = (int) (255 * Math.random());
      int green = (int) (255 * Math.random());
      int blue = (int) (255 * Math.random());
      leds.setColor(new Color8Bit(red, green, blue));
    }));

    // TODO: Once we're done with testing the autonomous motion commands, change
    // this to call resetOrientation().
    driveController.start().onTrue(new InstantCommand(() -> subsystems.drivetrain.resetPosition(new Pose2d())));

    manipulatorController.a().onTrue(new InstantCommand(() -> subsystems.claw.set(Position.OPEN)));
    manipulatorController.b().onTrue(new InstantCommand(() -> subsystems.claw.set(Position.GRAB_CUBE)));
    manipulatorController.x().onTrue(new InstantCommand(() -> subsystems.claw.set(Position.GRAB_CONE)));

  }

  /**
   * Returns the autonomous command selected in the chooser.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonomousCommandChooser.getSelected();
  }

  /** Adds the Shuffleboard tabs for the robot. */
  private void initShuffleboard() {
    // The "Operator" tab contains UI elements that enable the drive team to set up
    // and operate the robot.
    ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator");
    ShuffleboardLayout autonomousLayout = operatorTab.getLayout("Autonomous", BuiltInLayouts.kList)
        .withPosition(0, 0)
        .withSize(2, 2);

    autonomousLayout.add("Routine", autonomousCommandChooser);

    // The "Preferences" tab UI elements that enable configuring robot-specific
    // settings.
    RobotPreferences.addShuffleBoardTab();

    // The subsystem-specific tabs are added for testing and should be disabled by
    // default.
    subsystems.drivetrain.addShuffleboardTab();
  }
}
