// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Manages the drive and steering motors of a single swerve drive module.
 */
public class SwerveModule {
    private static final double kScalingFactor = 0.8;
    private static final double kFreeSpeedRPM = 6380;
    private static final double kDriveGearRatio = 8.14;
    private static final double kSteeringGearRatio = 12.8;
    private static final double kWheelRadius = 0.047625;
    private static final double kMotorStallTorque = 4.69; // Nm
    private static final double kRobotMass = 67.5853; // Kg

    // temp theoretical constants
    public static final double kMaxDriveSpeed = kScalingFactor * ((kFreeSpeedRPM * 2 * kWheelRadius * Math.PI) / (60 * kDriveGearRatio)); // meters
                                                                                                                 // per
                                                                                                                 // second
    public static final double kMaxDriveAcceleration = kScalingFactor * ((2 * 4 * kMotorStallTorque) / (2 * kWheelRadius * kRobotMass)); // meters
                                                                                                                // per
                                                                                                                // second
                                                                                                                // per
                                                                                                                // second
    private static final double kDriveS = 1.0; // voltage needed to overcome friction
    private static final double kDriveV = (12.0 - kDriveS) / kMaxDriveSpeed; // voltage needed to maintain constant velocity
    private static final double kDriveA = (12.0 - kDriveS) / kMaxDriveAcceleration; // voltate needed to maintain constant
                                                                              // acceleration

    public static final double kMaxSteeringSpeed = kScalingFactor * ((kFreeSpeedRPM * 2 * Math.PI) / (60 * kSteeringGearRatio));
    public static final double kMaxSteeringAcceleration = kScalingFactor * ((2 * 4 * kMotorStallTorque) / (2 * kRobotMass));
    private static final double kSteeringS = 1.0; // voltage needed to overcome friction
    private static final double kSteeringV = (12.0 - kSteeringS) / kMaxSteeringSpeed; // voltage needed to maintain constant
                                                                                // rotational velocity
    private static final double kSteeringA = (12.0 - kSteeringS) / kMaxSteeringAcceleration; // voltate needed to mantain
                                                                                       // constant rotational
                                                                                       // acceleration

    private final MotorController driveMotor;
    private final DoubleSupplier position;
    private final DoubleSupplier velocity;
    private final MotorController steeringMotor;
    private final DoubleSupplier wheelAngle;

    private final PIDController drivePID = new PIDController(1.0, 0, 0);
    private final ProfiledPIDController steeringPID = new ProfiledPIDController(1.0, 0, 0,
            new TrapezoidProfile.Constraints(kMaxSteeringSpeed, kMaxSteeringAcceleration));

    // models motors mathematically, calculates voltage needed
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(kDriveS, kDriveV, kDriveA);
    private final SimpleMotorFeedforward steeringFeedForward = new SimpleMotorFeedforward(kSteeringS, kSteeringV, kSteeringA);

    private final String name;

    /**
     * Constructs the swerve module.
     * 
     * @param driveMotor    The drive motor controller.
     * @param velocity      Supplies velocity in meters per second.
     * @param steeringMotor The steering motor controller.
     * @param wheelAngle    Supplies the wheel angle in degrees.
     */
    public SwerveModule(MotorController driveMotor, DoubleSupplier position, DoubleSupplier velocity,
            MotorController steeringMotor,
            DoubleSupplier wheelAngle, String name) {
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.wheelAngle = wheelAngle;
        this.position = position;
        this.velocity = velocity;
        this.name = name;

        steeringPID.enableContinuousInput(-Math.PI, Math.PI);
        steeringPID.reset(position.getAsDouble(), velocity.getAsDouble());
    }

    /**
     * Sets the desired state for the module.
     * 
     * @param state The desired state w/ speed and angle
     */
    public void setModuleState(SwerveModuleState state) {
        // Optimize the state to avoid spinning further than 90 degrees
        Rotation2d currentAngle = getWheelRotation2d();
        state = SwerveModuleState.optimize(state, currentAngle);

        // Calculate the drive motor voltage using PID and FeedForward
        double driveOutput = drivePID.calculate(velocity.getAsDouble(), state.speedMetersPerSecond);
        double driveFeedForward = this.driveFeedForward.calculate(state.speedMetersPerSecond);

        // Calculate the steering motor voltage using PID and FeedForward
        double steeringOutput = steeringPID.calculate(currentAngle.getRadians(), state.angle.getRadians());
        double steeringFeedForward = this.steeringFeedForward.calculate(steeringPID.getSetpoint().velocity);

        // Sets voltages of motors
        this.driveMotor.setVoltage(driveOutput + driveFeedForward);
        this.steeringMotor.setVoltage(steeringOutput + steeringFeedForward);
    }

    /**
     * Stops the drive and steering motors.
     */
    public void stopMotors() {
        driveMotor.stopMotor();
        steeringMotor.stopMotor();
    }

    /**
     * The position of the wheel on its axis of travel.
     * 
     * @return The position of the swerve module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(position.getAsDouble(), getWheelRotation2d());
    }

    /**
     * Returns the current wheel orientation.
     * 
     * @return The current wheel orientation.
     */
    public Rotation2d getWheelRotation2d() {
        return Rotation2d.fromDegrees(position.getAsDouble());
    }

    /**
     * Adds the SwerveModule layout to the Shuffleboard tab.
     * 
     * @param tab The Shuffleboard tab to add the layout.
     * @return The SwerveModule layout.
     */
    public ShuffleboardLayout addShuffleboardLayout(ShuffleboardTab tab) {
        ShuffleboardLayout layout = tab.getLayout(name, BuiltInLayouts.kList);
        layout.add("Rotation", new Sendable() {

            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Gyro");
                builder.addDoubleProperty("Value", wheelAngle, null);
            }
        }).withWidget(BuiltInWidgets.kGyro).withPosition(0, 0);
        return layout;
    }
}
