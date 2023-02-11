package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.CAN;
import frc.robot.parameters.MotorParameters;

/**
 * The elevator subsystem is responsible for setting the claw position for
 * acquiring or scoring game elements.
 */
public class ElevatorSubsystem extends SubsystemBase {

  private static final MotorParameters MOTOR = MotorParameters.NeoV1_1;
  private static final double GEAR_RATIO = 4; // TODO: Get real ratio
  private static final double PULLEY_DIAMETER = Units.inchesToMeters(2); // TODO: Get real value
  private static final double ELEVATOR_MASS = 1; // TODO: Get real mass
  private static final double MAX_SPEED = (MOTOR.getFreeSpeedRPM() * PULLEY_DIAMETER * Math.PI)
      / (60 * GEAR_RATIO); // meters/sec
  private static final double MAX_ACCELERATION = (2 * MOTOR.getStallTorque() * GEAR_RATIO)
      / (PULLEY_DIAMETER * ELEVATOR_MASS);
  private static final double KS = 0.15;
  private static final double KV = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_SPEED;
  private static final double KA = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ACCELERATION;
  private static final double KG = 9.81 * KA;
  private static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_SPEED,
      MAX_ACCELERATION);

  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final Supplier<Rotation2d> angle;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV, KA);
  private final ProfiledPIDController pidController = new ProfiledPIDController(1.0, 0, 0, CONSTRAINTS);
  private final Timer timer = new Timer();

  private GoalState goalState = GoalState.ACQUIRE;
  private TrapezoidProfile profile = 
      new TrapezoidProfile(CONSTRAINTS, new TrapezoidProfile.State(goalState.getPosition(), 0));
  private double currentPosition;
  private double currentVelocity;
  private Rotation2d currentAngle;

  /** Encapsulates various goal heights we want to raise the elevator to. */
  public enum GoalState {  // TODO: get real values
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
    motor = new CANSparkMax(CAN.SparkMax.ELEVATOR, MotorType.kBrushless);
    encoder = motor.getAlternateEncoder(MOTOR.getPulsesPerRevolution());
    angle = angleSupplier;
  }
  
  /**
   * Sets the desired claw position.
   * 
   * @param goalState The desired claw position.
   */
  public void setGoal(GoalState goalState) {
    this.goalState = goalState;
    pidController.setGoal(new TrapezoidProfile.State(goalState.getPosition(), 0));
    timer.reset();
    timer.start();
  }

  @Override
  public void periodic() {
    currentPosition = encoder.getPosition();
    currentVelocity = encoder.getVelocity();
    currentAngle = angle.get();

    if (goalState == GoalState.ACQUIRE && currentPosition <= GoalState.ACQUIRE.getPosition()) {
      motor.stopMotor();
      return;
    }

    TrapezoidProfile.State state = profile.calculate(timer.get());
    double outputVolts = pidController.calculate(currentVelocity, state.velocity);
    double feedforwardVolts = feedforward.calculate(currentVelocity, state.velocity)
        + (Math.sin(currentAngle.getRadians()) * KG);
    motor.setVoltage(outputVolts + feedforwardVolts);
  }
}
