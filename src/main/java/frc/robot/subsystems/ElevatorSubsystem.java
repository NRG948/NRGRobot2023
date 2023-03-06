package frc.robot.subsystems;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.CAN;
import frc.robot.Constants.RobotConstants.DigitalIO;
import frc.robot.parameters.MotorParameters;

/**
 * The elevator subsystem is responsible for setting the claw position for
 * acquiring or scoring game elements.
 */
@RobotPreferencesLayout(groupName = "Elevator", row = 1, column = 4, height = 1, width = 2)
public class ElevatorSubsystem extends SubsystemBase {
  @RobotPreferencesValue
  public static RobotPreferences.BooleanValue ENABLE_ELEVATOR_TAB = new RobotPreferences.BooleanValue("Elevator",
      "Enable Elevator Tab", false);

  // Constants representing the physical parameters of the elevator.
  private static final MotorParameters MOTOR = MotorParameters.NeoV1_1;
  private static final double GEAR_RATIO = 5.0 * 42.0 / 48.0;
  private static final double PULLEY_DIAMETER = Units.inchesToMeters(1.25);
  private static final double ELEVATOR_MASS = 3.63; // 3.63 kilograms, including the cone, trapdoor system, and the carrige of the elevator

  // Trapezoidal profile constants.
  private static final double MAX_SPEED = (MOTOR.getFreeSpeedRPM() * PULLEY_DIAMETER * Math.PI)
      / (60 * GEAR_RATIO); // meters/sec
  private static final double MAX_ACCELERATION = (2 * MOTOR.getStallTorque() * GEAR_RATIO)
      / (PULLEY_DIAMETER * ELEVATOR_MASS);

  private static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_SPEED, MAX_ACCELERATION);

  // Feedfoward constants.
  private static final double KS = 0.15;
  private static final double KV = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_SPEED;
  private static final double KA = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ACCELERATION;
  private static final double KG = 9.81 * KA;
  private static final double METERS_PER_REVOLUTION = (Units.inchesToMeters (0.955) *Math.PI)/GEAR_RATIO;

  private final CANSparkMax motor = new CANSparkMax(CAN.SparkMax.ELEVATOR, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  private final Supplier<Rotation2d> angle;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV, KA);
  private final ProfiledPIDController pidController = new ProfiledPIDController(1.0, 0, 0, CONSTRAINTS);
  private final Timer timer = new Timer();

  private final DigitalInput bottomLimit = new DigitalInput(DigitalIO.ELEVATOR_BOTTOM_LIMIT);
  private final DigitalInput topLimit = new DigitalInput(DigitalIO.ELEVATOR_TOP_LIMIT);

  private boolean enabled = false;
  private GoalState goalState = GoalState.ACQUIRE;
  private TrapezoidProfile profile = new TrapezoidProfile(
      CONSTRAINTS, new TrapezoidProfile.State(goalState.getPosition(), 0));
  private double currentPosition;
  private double currentVelocity;
  private Rotation2d currentAngle;
  private boolean currentBottomLimit;
  private boolean currentTopLimit;

  /** Encapsulates various goal heights we want to raise the elevator to. */
  public enum GoalState { // TODO: get real values
    ACQUIRE(0),
    SCORE_LOW(100),
    SCORE_MID(200),
    SCORE_HIGH(300);

    private final double position;

    GoalState(double position) {
      this.position = position;
    }

    /**
     * Returns the current position of elevator.
     * 
     * @return The current position of the elevator.
     */
    private double getPosition() {
      return position;
    }
  }

  /**
   * Creates a new ElevatorSubsystem.
   * 
   * @param angleSupplier Supplier that returns the current elevator angle.
   */
  public ElevatorSubsystem(Supplier<Rotation2d> angleSupplier) {
    motor.setIdleMode(IdleMode.kBrake);

    encoder.setPositionConversionFactor(METERS_PER_REVOLUTION);
    encoder.setVelocityConversionFactor(METERS_PER_REVOLUTION);

    angle = angleSupplier;
    currentAngle = angle.get();
  }

  /**
   * Sets the desired claw position.
   * 
   * @param goalState The desired claw position.
   */
  public void setGoal(GoalState goalState) {
    this.goalState = goalState;

    TrapezoidProfile.State state = new TrapezoidProfile.State(goalState.getPosition(), 0);

    profile = new TrapezoidProfile(CONSTRAINTS, state);
    pidController.setGoal(state);
    timer.reset();
    timer.start();
    enabled = true;
  }

  /**
   * Disables autonomous goal seeking.
   */
  public void disableGoalSeeking() {
    enabled = false;
    motor.stopMotor();
  }

  /**
   * Returns whether the elevator is at the goal state.
   * 
   * @return true if the elevator is at the goal state.
   */
  public boolean atGoal() {
    return pidController.atGoal();
  }

  /**
   * Returns if the elevator reaches the bootom limit.
   * 
   * @return True if the elevator reaches the bottom limit.
   */
  public boolean atBottomLimit() {
    return currentBottomLimit;
  }

  /**
   * Returns if the elevator reaches the top limit.
   * 
   * @return True if the elevator reaches the top limit.
   */
  public boolean atTopLimit() {
    return currentTopLimit;
  }

  /**
   * Sets the motor voltage.
   * 
   * @param voltage The desired voltage.
   */
  public void setMotorVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Stops the motor.
   */
  public void stopMotor() {
    motor.stopMotor();
  }

  /**
   * Returns the current position.
   * 
   * @return The current position.
   */
  public double getPosition() {
    return currentPosition;
  }

  /**
   * Returns whether the elevator is at the specified state.
   * 
   * @param state The state to check.
   * @return True if the elevator is at the specified state.
   */
  public boolean atPosition(GoalState state) {
    return Math.abs(currentPosition - state.getPosition()) <= 25; // TODO: determine tolerance with real units
  }

  /**
   * Returns the current velocity.
   * 
   * @return The current velocity.
   */
  public double getVelocity() {
    return currentVelocity;
  }

  @Override
  public void periodic() {
    // Update the sensor state.
    currentPosition = encoder.getPosition();
    currentVelocity = encoder.getVelocity();
    currentAngle = angle.get();
    currentBottomLimit = bottomLimit.get();
    currentTopLimit = topLimit.get();

    if (!enabled) {
      return;
    }

    // If the elevator has been moved to the lowest position, stop the motor.
    if (goalState == GoalState.ACQUIRE && currentPosition <= GoalState.ACQUIRE.getPosition() || atBottomLimit()) {
      disableGoalSeeking();
      return;
    }

    // Find the desired elevator state (position and velocity) using the trapezoid
    // profile and use feedback (PID) and feedforward to calculate the required
    // voltage to apply to the motor.
    TrapezoidProfile.State state = profile.calculate(timer.get());
    double feedbackVolts = pidController.calculate(currentVelocity, state.velocity);
    double feedforwardVolts = Math.sin(currentAngle.getRadians()) * KG;

    if (goalState != GoalState.SCORE_HIGH || !atTopLimit()) {
      feedforwardVolts += feedforward.calculate(currentVelocity, state.velocity);
    }

    motor.setVoltage(feedbackVolts + feedforwardVolts);
  }

  /**
   * Adds the Shuffleboard Tab for elevator debugging.
   * 
   * @param acquiringLimit Supplies the acquiring limit switch value.
   * @param scoringLimit   Supplies the scoring limit switch value.
   */
  public void addShuffleBoardTab(BooleanSupplier acquiringLimit, BooleanSupplier scoringLimit) {
    if (!ENABLE_ELEVATOR_TAB.getValue()) {
      return;
    }

    ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    ShuffleboardLayout layout = tab.getLayout("Elevator", BuiltInLayouts.kGrid)
        .withPosition(0, 0)
        .withSize(3, 3)
        .withProperties(Map.of("Number of columns", 2, "Number of rows", 1));
    ShuffleboardLayout positionLayout = layout.getLayout("Position", BuiltInLayouts.kList)
        .withPosition(0, 0);
    
    positionLayout.addNumber("Angle", () -> angle.get().getDegrees());
    positionLayout.addNumber("Position", () -> currentPosition);
    positionLayout.addNumber("Velocity", () -> currentVelocity);
    
    ShuffleboardLayout switchLayout = layout.getLayout("Limit Switches", BuiltInLayouts.kList)
        .withPosition(1, 0);
    switchLayout.addBoolean("Top Limit Switch", this::atTopLimit);
    switchLayout.addBoolean("Bottom Limit Switch", this::atBottomLimit);
    switchLayout.addBoolean("Acquiring Limit Switch", acquiringLimit);
    switchLayout.addBoolean("Scoring Limit Switch", scoringLimit);
  }

}
