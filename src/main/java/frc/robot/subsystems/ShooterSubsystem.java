// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.CAN;

@RobotPreferencesLayout(groupName = "Shooter", row = 0, column = 6, width = 2, height = 4)
public class ShooterSubsystem extends SubsystemBase {
  @RobotPreferencesValue
  public static RobotPreferences.BooleanValue ENABLE_SHOOTER_TAB = new RobotPreferences.BooleanValue("Shooter",
    "Enable Shooter Tab", false);

  private static final double MAX_RPM = 5880.0; // change name if reed doesnt like it
  private static final double RPM_PER_VOLT = 493.9; // Provided by systems, the change in RPM per change in volt. Could
                                                    // be useful.
  private static final double KS = 1.0; // guess

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue HYBRID_RPM = new RobotPreferences.DoubleValue("Shooter", "Hybrid RPM", 250);
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue MID_RPM = new RobotPreferences.DoubleValue("Shooter", "Mid RPM", 1000);
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue HIGH_RPM = new RobotPreferences.DoubleValue("Shooter", "High RPM", 1500);

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue BACKSPIN_FACTOR = new RobotPreferences.DoubleValue("Shooter", "Backspin constant", 0.66); // TODO: determine real backspin factor

  public enum GoalShooterRPM {
    // TODO: get real RPMs
    STOP(new RobotPreferences.DoubleValue("", "", 0.0)),
    HYBRID(HYBRID_RPM),
    MID(MID_RPM), // 1000 rpm as proposed by Taiga
    HIGH(HIGH_RPM);

    private final RobotPreferences.DoubleValue rpm;

    GoalShooterRPM(RobotPreferences.DoubleValue rpm) {
      this.rpm = rpm;
    }

    /**
     * Returns the RPM corresponding to the desired goal.
     * 
     * @return The RPM corresponding to the desired goal.
     */
    private double getRPM() {
      return rpm.getValue();
    }
  }

  private final CANSparkMax topShooterMotor = new CANSparkMax(CAN.SparkMax.TOP_SHOOTER, MotorType.kBrushless);
  private final CANSparkMax bottomShooterMotor = new CANSparkMax(CAN.SparkMax.BOTTOM_SHOOTER, MotorType.kBrushless);
  private final RelativeEncoder topEncoder = topShooterMotor.getEncoder();
  private final RelativeEncoder bottomEncoder = bottomShooterMotor.getEncoder();

  private GoalShooterRPM currentGoalRPM = GoalShooterRPM.STOP;

  private double currentTopRPM;
  private double currentBottomRPM;
  private double currentVoltage;
  private boolean isEnabled = false;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    topShooterMotor.setIdleMode(IdleMode.kCoast);
    bottomShooterMotor.setIdleMode(IdleMode.kCoast);
    bottomShooterMotor.setInverted(true);
  }

  /**
   * Sets the goal RPM of the shooter.
   * 
   * @param goalRPM The goal RPM.
   */
  public void setGoalRPM(GoalShooterRPM goalRPM) {
    currentGoalRPM = goalRPM;
  }

  /**
   * Enables the shooter.
   * 
   * @param goalShooterRPM The desired shooter RPM.
   */
  public void enable(GoalShooterRPM goalShooterRPM) {
    isEnabled = true;
    setGoalRPM(goalShooterRPM);
  }

  /**
   * Calculates the motor voltage needed for a given RPM.
   * 
   * @param goalShooterRPM The desired RPM.
   * @return The motor voltage needed.
   */
  public double calculateMotorVoltage(GoalShooterRPM goalShooterRPM) {
    return goalShooterRPM.getRPM() / RPM_PER_VOLT + KS; // not sure
  }

  /**
   * Stops the motors.
   */
  public void stopMotor() {
    topShooterMotor.stopMotor();
    bottomShooterMotor.stopMotor();
  }

  /**
   * Sets the motor speed for the shooter motors.
   * 
   * @param speed The desired motor speed.
   */
  public void setMotorVoltage(double voltage) {
    topShooterMotor.setVoltage(voltage * BACKSPIN_FACTOR.getValue());
    bottomShooterMotor.setVoltage(voltage);
  }

  /**
   * Disable the shooter.
   */
  public void disable() {
    isEnabled = false;
    stopMotor();
  }

  /**
   * Returns the current RPM.
   * 
   * @return the current RPM.
   */
  public double getCurrentTopRPM() {
    return currentTopRPM;
  }

  /**
   * Returns the current goal RPM.
   * 
   * @return the current goal RPM.
   */
  public GoalShooterRPM getCurrentGoalRPM() {
    return currentGoalRPM;
  }

  /**
   * Returns whether the shooter is enabled.
   * 
   * @return whether the shooter is enabled.
   */
  public boolean isEnabled() {
    return isEnabled;
  }

  public void periodic() {
    currentTopRPM = topEncoder.getVelocity();
    currentBottomRPM = bottomEncoder.getVelocity();

    if (isEnabled) {
      currentVoltage = calculateMotorVoltage(currentGoalRPM);
      setMotorVoltage(currentVoltage);
    }
  }

  /**
   * Adds the Shuffleboard Tab for elevator debugging.
   * 
   * @param acquiringLimit Supplies the acquiring limit switch value.
   * @param scoringLimit   Supplies the scoring limit switch value.
   */
  public void addShuffleBoardTab(BooleanSupplier isCubeDetected) {
    if (!ENABLE_SHOOTER_TAB.getValue()) {
      return;
    }

    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    ShuffleboardLayout layout = tab.getLayout("Shooter", BuiltInLayouts.kList)
        .withPosition(0, 0)
        .withSize(1, 3);
    
    layout.addNumber("Current top RPM", () -> currentTopRPM);
    layout.addNumber("Current bottom RPM", () -> currentBottomRPM);
    layout.addNumber("Current goal RPM", () -> currentGoalRPM.getRPM());
    layout.addBoolean("Has Cube", isCubeDetected);
  }
}
