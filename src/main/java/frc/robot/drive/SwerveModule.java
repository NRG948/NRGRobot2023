// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.parameters.SwerveDriveParameters;

/**
 * Manages the drive and steering motors of a single swerve drive module.
 * 
 * This class uses a combination feedback (i.e. PID) and feedforward control to
 * achieve desired translational (i.e. drive) and rotational (i.e. steering)
 * velocities. For more information, see the <a href=
 * "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html">
 * Introduction to PID</a> and <a href=
 * "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html">
 * Introduction to DC Motor Feedforward</a> articles of the WPILib
 * documentation.
 */
public class SwerveModule {
    private final MotorController driveMotor;
    private final DoubleSupplier positionSupplier;
    private final DoubleSupplier velocitySupplier;
    private final MotorController steeringMotor;
    private final Supplier<Rotation2d> wheelAngleSupplier;
    private final String name;

    // models motors mathematically, calculates voltage needed
    private final SimpleMotorFeedforward driveFeedForward;
    private final SimpleMotorFeedforward steeringFeedForward;

    private final PIDController drivePID;
    private final ProfiledPIDController steeringPID;

    // The current supplied state updated by the periodic method.
    private SwerveModulePosition position;
    private double velocity;

    /**
     * Constructs the swerve module.
     * 
     * @param parameters    A {@link SwerveDriveParameters} object providing
     *                      information on the physical swerve drive
     *                      characteristics.
     * @param driveMotor    The drive motor controller.
     * @param position      Supplies the position in meters.
     * @param velocity      Supplies velocity in meters per second.
     * @param steeringMotor The steering motor controller.
     * @param wheelAngle    Supplies the wheel angle in degrees.
     * @param name          The name of the module.
     */
    public SwerveModule(
            SwerveDriveParameters parameters,
            MotorController driveMotor,
            DoubleSupplier position,
            DoubleSupplier velocity,
            MotorController steeringMotor,
            Supplier<Rotation2d> wheelAngle,
            String name) {
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.wheelAngleSupplier = wheelAngle;
        this.positionSupplier = position;
        this.velocitySupplier = velocity;
        this.name = name;

        initializeSuppliedState();

        this.driveFeedForward = new SimpleMotorFeedforward(
                parameters.getDriveKs(), parameters.getDriveKv(), parameters.getDriveKa());
        this.steeringFeedForward = new SimpleMotorFeedforward(
                parameters.getSteeringKs(), parameters.getSteeringKv(), parameters.getSteeringKa());

        this.drivePID = new PIDController(5.0, 0, 0.0);

        this.steeringPID = new ProfiledPIDController(7.0, 0, 0.0, parameters.getSteeringConstraints());
        this.steeringPID.enableContinuousInput(-Math.PI, Math.PI);
        this.steeringPID.setTolerance(Math.toRadians(1.0));
        this.steeringPID.reset(getPosition().angle.getRadians());
    }

    /**
     * Initializes the supplied state.
     */
    private void initializeSuppliedState() {
        updateSuppliedState();
    }

    /**
     * Updates the supplied state.
     * <p>
     * This method **MUST* be called by the {@link #periodic()} method to ensure the
     * supplied state is up to date for subsequent use.
     */
    private void updateSuppliedState() {
        position = new SwerveModulePosition(positionSupplier.getAsDouble(), wheelAngleSupplier.get());
        velocity = velocitySupplier.getAsDouble();
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
        double driveOutput = drivePID.calculate(velocity, state.speedMetersPerSecond);
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
        return position;
    }

    /**
     * Returns the current wheel orientation.
     * 
     * @return The current wheel orientation.
     */
    public Rotation2d getWheelRotation2d() {
        return position.angle;
    }

    /**
     * This method is called periodically by the {@link SwerveSubsystem}. It is used
     * to update module-specific state.
     */
    public void periodic() {
        updateSuppliedState();
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
                builder.addDoubleProperty("Value", () -> position.angle.getRadians(), null);
            }
        }).withWidget(BuiltInWidgets.kGyro);
        return layout;
    }
}
