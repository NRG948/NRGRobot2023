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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.CAN;
import frc.robot.parameters.MotorParameters;

@RobotPreferencesLayout(groupName = "Shooter", row = 0, column = 6, width = 2, height = 4)
public class ShooterSubsystem extends SubsystemBase {
  @RobotPreferencesValue
  public static RobotPreferences.BooleanValue ENABLE_SHOOTER_TAB = new RobotPreferences.BooleanValue("Shooter",
    "Enable Shooter Tab", false);

  private static final double KS = 0.15;
  private static final double KV = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) * 3 / MotorParameters.NeoV1_1.getFreeSpeedRPM();

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue HYBRID_RPM = new RobotPreferences.DoubleValue("Shooter", "Hybrid RPM", 300);
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue MID_RPM = new RobotPreferences.DoubleValue("Shooter", "Mid RPM", 800);
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue HIGH_RPM = new RobotPreferences.DoubleValue("Shooter", "High RPM", 1225);
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue MID_CHARGE_STATION_RPM = new RobotPreferences.DoubleValue("Shooter", "Mid Charge Station RPM", 1420);
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue HYBRID_BACKSPIN_FACTOR = new RobotPreferences.DoubleValue("Shooter", "Hybrid Backspin", 1);
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue MID_BACKSPIN_FACTOR = new RobotPreferences.DoubleValue("Shooter", "Mid Backspin", 0.5);
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue HIGH_BACKSPIN_FACTOR = new RobotPreferences.DoubleValue("Shooter", "High Backspin", 0.5);
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue MID_CHARGE_STATION_BACKSPIN_FACTOR = new RobotPreferences.DoubleValue("Shooter", "Mid Charge Station Backspin", 0.5);

  public enum GoalShooterRPM {
    // TODO: get real RPMs
    STOP(new RobotPreferences.DoubleValue("", "", 0.0), new RobotPreferences.DoubleValue("", "", 0)),
    HYBRID(HYBRID_RPM, HYBRID_BACKSPIN_FACTOR),
    MID(MID_RPM, MID_BACKSPIN_FACTOR),
    HIGH(HIGH_RPM, HIGH_BACKSPIN_FACTOR),
    MID_CHARGE_STATION(MID_CHARGE_STATION_RPM, MID_CHARGE_STATION_BACKSPIN_FACTOR);

    private final RobotPreferences.DoubleValue rpm;
    private final RobotPreferences.DoubleValue backspinFactor;

    GoalShooterRPM(RobotPreferences.DoubleValue rpm, RobotPreferences.DoubleValue backspinFactor) {
      this.rpm = rpm;
      this.backspinFactor = backspinFactor;
    }

    /**
     * Returns the RPM corresponding to the desired goal.
     * 
     * @return The RPM corresponding to the desired goal.
     */
    private double getRPM() {
      return rpm.getValue();
    }

    /**
     * Returns the percent of the bottom RPM for top motor.
     * 
     * @return The percent of the bottom RPM for top motor.
     */
    private double getBackspinFactor() {
      return backspinFactor.getValue();
    }
  }

  private final CANSparkMax topMotor = new CANSparkMax(CAN.SparkMax.TOP_SHOOTER, MotorType.kBrushless);
  private final CANSparkMax bottomMotor = new CANSparkMax(CAN.SparkMax.BOTTOM_SHOOTER, MotorType.kBrushless);
  private final RelativeEncoder topEncoder = topMotor.getEncoder();
  private final RelativeEncoder bottomEncoder = bottomMotor.getEncoder();

  private PIDController topPIDController = new PIDController(0.002088, 0, 0);
  private PIDController bottomPIDController = new PIDController(0.002088, 0, 0);
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV);

  private GoalShooterRPM currentGoalRPM = GoalShooterRPM.STOP;

  private double currentTopRPM;
  private double currentBottomRPM;
  private double topVoltage;
  private double bottomVoltage;
  private boolean isEnabled = false;
  
  private DoubleLogEntry goalRPMLogger = new DoubleLogEntry(DataLogManager.getLog(), "Shooter/Goal RPM");
  private DoubleLogEntry topRPMLogger = new DoubleLogEntry(DataLogManager.getLog(), "Shooter/Top RPM");
  private DoubleLogEntry bottomRPMLogger = new DoubleLogEntry(DataLogManager.getLog(), "Shooter/Bottom RPM");

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    topMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setInverted(true);
    topEncoder.setVelocityConversionFactor(0.333);
    bottomEncoder.setVelocityConversionFactor(0.333);
  }

  /**
   * Sets the goal RPM of the shooter.
   * 
   * @param goalRPM The goal RPM.
   */
  public void setGoalRPM(GoalShooterRPM goalRPM) {
    currentGoalRPM = goalRPM;
    topPIDController.setSetpoint(goalRPM.getRPM() * goalRPM.getBackspinFactor());
    bottomPIDController.setSetpoint(goalRPM.getRPM());
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
   * Stops the motors.
   */
  public void stopMotor() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  /**
   * Sets the motor voltages for both top and bottom motor.
   * 
   * @param topVoltage The voltage for the top motor.
   * @param bottomVoltage The voltage for the bottom motor.
   */
  public void setMotorVoltages(double topVoltage, double bottomVoltage) {
    topMotor.setVoltage(topVoltage);
    bottomMotor.setVoltage(bottomVoltage);
  }

  /**
   * Disable the shooter.
   */
  public void disable() {
    isEnabled = false;
    stopMotor();
    topPIDController.reset();
    bottomPIDController.reset();
  }

  /**
   * Returns the current RPM.
   * 
   * @return the current RPM.
   */
  public double getCurrentTopRPM() {
    return currentTopRPM;
  }

  public double getCurrentBottomRPM() {
    return currentBottomRPM;
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
      double bottomFeedforwardVoltage = feedforward.calculate(currentGoalRPM.getRPM());
      double topFeedForwardVoltage = feedforward.calculate(currentGoalRPM.getRPM() * currentGoalRPM.getBackspinFactor());

      topVoltage = topFeedForwardVoltage + topPIDController.calculate(currentTopRPM);
      bottomVoltage = bottomFeedforwardVoltage + bottomPIDController.calculate(currentBottomRPM);
      
      setMotorVoltages(topVoltage, bottomVoltage);
    }
    
    topRPMLogger.append(currentTopRPM);
    bottomRPMLogger.append(currentBottomRPM);
    goalRPMLogger.append(currentGoalRPM.getRPM());

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
