// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.CAN;
import frc.robot.Constants.RobotConstants.DigitalIO;
import frc.robot.parameters.MotorParameters;

/**
 * The elevator angle subsystem is responsible for controlling the elevator
 * angle.
 */
public class ElevatorAngleSubsystem extends SubsystemBase {

  // AQUIRING and SCORING are out of the frame perimeter
  public enum ElevatorAngle {
    ACQUIRING(30),
    SCORING(120);

    private final double angle;

    private ElevatorAngle(double angle) {
      this.angle = angle;
    }

    /** Returns elevator angle in degrees. */
    private double getDegrees() {
      return angle;
    }

    /** Returns elevator angle in radians. */
    private double getRadians() {
      return Math.toRadians(angle);
    }
  }

  // CONSTANTS
  private static final double ANGLE_RANGE = 90;
  private static final double GEAR_RATIO = 100 / 1;
  private static final double MOTOR_POWER = 0.3;
  public static final double MASS = 9.97903; //TODO: update mass when claw change.
  private static final MotorParameters MOTOR = MotorParameters.NeoV1_1;

  // Calculate degrees per pulse
  private static final double RADIANS_PER_REVOLUTION = 2 * Math.PI / (GEAR_RATIO);

  private static final double MAX_ANGULAR_SPEED = (MOTOR.getFreeSpeedRPM() * 2 * Math.PI) / (60 * GEAR_RATIO);
  private static final double MAX_ANGULAR_ACCELERATION = (2 * MOTOR.getStallTorque() * GEAR_RATIO) / MASS;
  private static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED * MOTOR_POWER, MAX_ANGULAR_ACCELERATION);
  private static final double KS = 0.15;
  private static final double KV = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ANGULAR_SPEED;
  private static final double KA = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ANGULAR_ACCELERATION;
  private static final double KG = 9.81 * KA;

  private final CANSparkMax motor = new CANSparkMax(CAN.SparkMax.ELEVATOR_ANGLE, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final DigitalInput acquiringLimit = new DigitalInput(DigitalIO.ELEVATOR_ANGLE_ACQUIRE_LIMIT);
  private final DigitalInput scoringLimit = new DigitalInput(DigitalIO.ELEVATOR_ANGLE_SCORING_LIMIT);
  private final ArmFeedforward feedforward = new ArmFeedforward(KS, KV, KA, KG);
  private final ProfiledPIDController controller = new ProfiledPIDController(1.0, 0.0, 0.0, CONSTRAINTS);
  private final Timer timer = new Timer();

  private double angleOffset; // record encoder's current position
  private ElevatorAngle goalAngle = ElevatorAngle.ACQUIRING;
  private TrapezoidProfile profile = new TrapezoidProfile(CONSTRAINTS, new TrapezoidProfile.State(goalAngle.getDegrees(), 0));
  private double currentAngle = ElevatorAngle.ACQUIRING.getRadians(); // start at acquiring
  private double currentVelocity = 0;
  private boolean isPeriodicControlEnabled = false;
  private boolean currentAcquiringLimit;
  private boolean currentScoringLimit;

  /** Creates a new ElevatorAngleSubsystem. */
  public ElevatorAngleSubsystem() {
    motor.setInverted(true);
    // convert encoder ticks to angle
    encoder.setPositionConversionFactor(RADIANS_PER_REVOLUTION);
    encoder.setVelocityConversionFactor(RADIANS_PER_REVOLUTION);
    
    angleOffset = encoder.getPosition() - ElevatorAngle.ACQUIRING.getRadians();
    motor.setIdleMode(IdleMode.kBrake);
  }

  public void setMotor(double power) {
    if (power == 0) {
      motor.stopMotor();
    } else {
      motor.set(power);
    }
  }

  /**
   * Sets the goal elevator angle.
   * 
   * @param goalAngle the goal elevator angle.
   */
  public void setGoalAngle(ElevatorAngle goalAngle) {
    this.goalAngle = goalAngle;
    TrapezoidProfile.State goalState = new TrapezoidProfile.State(goalAngle.getRadians(), 0);
    profile = new TrapezoidProfile(CONSTRAINTS, goalState);
    controller.setGoal(goalState);
    timer.reset();
    timer.start();
    enablePeriodicControl(true);
  }

  /**
   * Returns whether the elevator is at the goal angle.
   * 
   * @return True if the elevator is at the goal angle.
   */
  public boolean atGoalAngle() {
    return controller.atGoal() || (goalAngle == ElevatorAngle.ACQUIRING ? atAcquiringLimit() : atScoringLimit());
  }

  /**
   * Gets the current elevator angle in degrees.
   * 
   * @return the current elevator angle.
   */
  public double getAngle() {
    return currentAngle;
  }

  /** Enables periodic control. */
  public void enablePeriodicControl(boolean isEnabled) {
    isPeriodicControlEnabled = isEnabled;
    if (!isPeriodicControlEnabled) {
      motor.stopMotor();
    }
  }

  /**
   * Returns if the elevator angle reaches the scoring limit.
   * 
   * @return True if the elevator angle reaches the scoring limit.
   */
  public boolean atScoringLimit() {
    return currentScoringLimit;
  }

  /**
   * Returns if the elevator angle reaches the acquiring limit.
   * 
   * @return True if the elevator angle reaches the acquiring limit.
   */
  public boolean atAcquiringLimit() {
    return currentAcquiringLimit;
  }

  @Override
  public void periodic() {
    currentAcquiringLimit = acquiringLimit.get();
    currentScoringLimit = scoringLimit.get();
    currentAngle = encoder.getPosition() - angleOffset;
    currentVelocity = encoder.getVelocity();

    if (!isPeriodicControlEnabled) {
      return;
    }

    if (atGoalAngle()) {
      enablePeriodicControl(false);
      return;
    }

    TrapezoidProfile.State state = profile.calculate(timer.get());
    double feedbackVolts = controller.calculate(currentVelocity, state.velocity);
    double feedforwardVolts = feedforward.calculate(currentAngle, currentVelocity);

    motor.setVoltage(feedforwardVolts + feedbackVolts);
  }

}
