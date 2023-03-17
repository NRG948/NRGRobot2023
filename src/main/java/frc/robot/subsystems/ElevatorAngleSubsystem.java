// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
    ACQUIRING(58), // TO-DO: Change this angle so the carriage aligns with the intake
    // SCORING(134.6);
    SCORING(123);

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

  // Loggers for sensor data, current state, calculated feedback & feedforward
  // voltages
  private DoubleLogEntry angleLogger = new DoubleLogEntry(DataLogManager.getLog(), "ElevatorAngle/Angle");
  private DoubleLogEntry velocityLogger = new DoubleLogEntry(DataLogManager.getLog(), "ElevatorAngle/Velocity");
  private DoubleLogEntry statePositionLogger = new DoubleLogEntry(DataLogManager.getLog(),
      "ElevatorAngle/StatePosition");
  private DoubleLogEntry stateVelocityLogger = new DoubleLogEntry(DataLogManager.getLog(),
      "ElevatorAngle/StateVelocity");
  private DoubleLogEntry motorVoltageLogger = new DoubleLogEntry(DataLogManager.getLog(), "ElevatorAngle/MotorVoltage");
  private DoubleLogEntry feedbackLogger = new DoubleLogEntry(DataLogManager.getLog(), "ElevatorAngle/Feedback");
  private DoubleLogEntry feedfowardLogger = new DoubleLogEntry(DataLogManager.getLog(), "ElevatorAngle/Feedforward");
  // CONSTANTS
  private static final double GEAR_RATIO = (100 * 32) / 12; //Change to compensate for new sprocket
  private static final double MOTOR_POWER = 0.7;
  public static final double MASS = 9.97903; // TODO: update mass when claw change.
  private static final MotorParameters MOTOR = MotorParameters.NeoV1_1;

  // Calculate degrees per pulse
  private static final double RADIANS_PER_REVOLUTION = 2 * Math.PI / (GEAR_RATIO);

  private static final double MAX_ANGULAR_SPEED = MOTOR.getFreeSpeedRPM() * RADIANS_PER_REVOLUTION / 60;
  private static final double MAX_ANGULAR_ACCELERATION = (2 * MOTOR.getStallTorque() * GEAR_RATIO) / MASS;
  private static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ANGULAR_SPEED * MOTOR_POWER, MAX_ANGULAR_ACCELERATION);
  private static final double KS = 0.21654; // 5.0
  private static final double KV = 0.25197; // (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ANGULAR_SPEED;
  private static final double KA = 0.065149; // (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ANGULAR_ACCELERATION;
  private static final double KG = 9.81 * KA;

  private static final double ENCODER_MINIMUM_DUTY_CYCLE = 1.0 / 1025.0;
  private static final double ENCODER_MAXIMUM_DUTY_CYCLE = 1024.0 / 1025.0;
  private static final double ENCODER_DISTANCE_PER_ROTATION = 2.0 * Math.PI;

  private final CANSparkMax motor = new CANSparkMax(CAN.SparkMax.ELEVATOR_ANGLE, MotorType.kBrushless);
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(DigitalIO.ELEVATOR_ANGLE_ENCODER);
  private final DigitalInput acquiringLimit = new DigitalInput(DigitalIO.ELEVATOR_ANGLE_ACQUIRE_LIMIT);
  private final DigitalInput scoringLimit = new DigitalInput(DigitalIO.ELEVATOR_ANGLE_SCORING_LIMIT);
  private final ArmFeedforward feedforward = new ArmFeedforward(KS, KV, KA, KG);
  private final ProfiledPIDController controller = new ProfiledPIDController(5.0, 0.0, 0.0, CONSTRAINTS);
  private final Timer timer = new Timer();

  private ElevatorAngle goalAngle = ElevatorAngle.ACQUIRING;
  private TrapezoidProfile profile = new TrapezoidProfile(CONSTRAINTS,
      new TrapezoidProfile.State(goalAngle.getDegrees(), 0));
  private double currentAngle = ElevatorAngle.ACQUIRING.getRadians(); // start at acquiring
  private double angleOffset;
  private boolean angleOffsetInitialize = false;
  private double currentVelocity = 0;
  private double lastAngleTime;
  private double lastAngle;
  private boolean isPeriodicControlEnabled = false;
  private boolean currentAcquiringLimit;
  private boolean currentScoringLimit;

  /** Creates a new ElevatorAngleSubsystem. */
  public ElevatorAngleSubsystem() {
    motor.setInverted(true);
    motor.setIdleMode(IdleMode.kBrake);
    encoder.setDistancePerRotation(ENCODER_DISTANCE_PER_ROTATION);
    encoder.setDutyCycleRange(ENCODER_MINIMUM_DUTY_CYCLE, ENCODER_MAXIMUM_DUTY_CYCLE);
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
    System.out.println("GOAL ANGLE: " + goalAngle);
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
   * Gets the current elevator angle in radians.
   * 
   * @return the current elevator angle in radians.
   */
  public double getAngle() {
    return currentAngle;
  }

  /**
   * Gets the current elevator angular velocity in radians per second.
   * 
   * @return the current velocity
   */
  public double getVelocity() {
    return currentVelocity;
  }

  /** Enables pr disables periodic control. */
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
    currentScoringLimit = !scoringLimit.get(); // Negate the sensor because it's a hall effect sensor
    currentAcquiringLimit = acquiringLimit.get();

    if (!angleOffsetInitialize){
      angleOffset = encoder.getAbsolutePosition();
      angleOffsetInitialize = true;
    }

    currentAngle = (((1.0 - encoder.getAbsolutePosition() + angleOffset) % 1.0) * ENCODER_DISTANCE_PER_ROTATION 
        + ElevatorAngle.ACQUIRING.getRadians()) % (2 * Math.PI);
    
    double currentTime = Timer.getFPGATimestamp();
    
    if (lastAngleTime != 0) {
      currentVelocity = (currentAngle - lastAngle) / (currentTime - lastAngleTime);
    }
    
    lastAngleTime = currentTime;
    lastAngle = currentAngle;

    if (!isPeriodicControlEnabled) {
      return;
    }

    if (atGoalAngle()) {
      enablePeriodicControl(false);
      return;
    }

    TrapezoidProfile.State state = profile.calculate(timer.get());
    double feedbackVolts = controller.calculate(currentAngle, state.position);
    double feedforwardVolts = feedforward.calculate(state.position, state.velocity);
    double voltage = feedbackVolts + feedforwardVolts;

    setMotorVoltage(voltage);
    angleLogger.append(currentAngle);
    velocityLogger.append(currentVelocity);
    statePositionLogger.append(state.position);
    stateVelocityLogger.append(state.velocity);
    motorVoltageLogger.append(feedbackVolts + feedforwardVolts);
    feedbackLogger.append(feedbackVolts);
    feedfowardLogger.append(feedforwardVolts);

  }

  /**
   * Sets the motor voltage.
   * 
   * @param voltage The motor voltage.
   */
  public void setMotorVoltage(double voltage) {
    if (voltage > 0 ? !atScoringLimit() : !atAcquiringLimit()) {
      motor.setVoltage(voltage);
    } else {
      motor.stopMotor();
    }
  }

  /**
   * Stops the motor.
   */
  public void stopMotor() {
    motor.stopMotor();
  }

}
