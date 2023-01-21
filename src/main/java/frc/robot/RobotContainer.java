// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveWithController;
import frc.robot.subsystems.Subsystems;
import com.nrg948.autonomous.Autonomous;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // new drive controller
  private final CommandXboxController driveController = new CommandXboxController(0);

  // The robot's subsystems and commands are defined here...
  private final Subsystems subsystems = new Subsystems();

  private DriveWithController driveWithController = new DriveWithController(subsystems.drivetrain, driveController);

  public static AddressableLEDs leds = new AddressableLEDs(2, 51);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public final SendableChooser<Command> autonomousCommandChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    subsystems.drivetrain.setDefaultCommand(driveWithController);
    autonomousCommandChooser = Autonomous.getChooser(subsystems, "frc.robot");
    initShuffleboard();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    // Configure the trigger bindings
    configureBindings();
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
  private void configureBindings() {

    m_driverController.y().onTrue(new InstantCommand(() -> {
      int red = (int) (255 * Math.random());
      int green = (int) (255 * Math.random());
      int blue = (int) (255 * Math.random());
      leds.setColor(new Color8Bit(red, green, blue));
    }));
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
    ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator");
    ShuffleboardLayout autonomousLayout = operatorTab.getLayout("Autonomous", BuiltInLayouts.kList)
        .withPosition(0, 0)
        .withSize(2, 2);

    autonomousLayout.add("Routine", autonomousCommandChooser);

    subsystems.drivetrain.addShuffleboardTab();
  }
}
