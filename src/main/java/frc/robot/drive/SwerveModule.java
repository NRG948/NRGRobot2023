// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Manages the drive and steering motors of a single swerve drive module.
 */
public class SwerveModule {
    private static double kFreeSpeedRPM = 6380;
    private static double kDriveGearRatio = 8.14;
    private static double kSteeringGearRatio = 12.8;
    private static double kWheelRadius = 0.047625;
    private static double kMotorStallTorque = 4.69; // Nm
    private static double kRobotMass = 67.5853; // Kg

    // temp theoretical constants
    public static double kMaxDriveSpeed = (kFreeSpeedRPM * 2 * kWheelRadius * Math.PI) / (60 * kDriveGearRatio); // meters per second
    private static double kDriveS = 1.0; // voltage needed to overcome friction
    private static double kDriveV = (12.0 - kDriveS) / kMaxDriveSpeed; // voltage needed to maintain constant velocity
    private static double kDriveA = (2 * 4 * kMotorStallTorque) / (2 * kWheelRadius * kRobotMass);

    public static double kMaxSteeringSpeed = (kFreeSpeedRPM * 2 * Math.PI) / (60 * kSteeringGearRatio);
    private static double kSteeringS = 1.0; // voltage needed to overcome friction
    private static double kSteeringV = (12.0 - kSteeringS) / kMaxSteeringSpeed; // voltage needed to maintain constant
                                                                                // rotational velocity
    private static double kSteeringA = (2 * 4 * kMotorStallTorque) / (2 * kRobotMass);

    private MotorController driveMotor;
    private DoubleSupplier velocity;
    private MotorController steeringMotor;
    private DoubleSupplier wheelAngle;

    private PIDController drivePID = new PIDController(1.0, 0, 0);
    private PIDController steeringPID = new PIDController(1.0, 0, 0);

    // models motors mathematically, calculates voltage needed
    private SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(kDriveS, kDriveV, kDriveA);
    private SimpleMotorFeedforward steeringFeedForward = new SimpleMotorFeedforward(kSteeringS, kSteeringV, kSteeringA);

    private final String name;

    /**
     * Constructs the swerve module.
     * 
     * @param driveMotor    The drive motor controller.
     * @param velocity      Supplies velocity in meters per second.
     * @param steeringMotor The steering motor controller.
     * @param wheelAngle    Supplies the wheel angle in degrees.
     */
    public SwerveModule(MotorController driveMotor, DoubleSupplier velocity, MotorController steeringMotor,
            DoubleSupplier wheelAngle, String name) {
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.wheelAngle = wheelAngle;
        this.velocity = velocity;
        this.name = name;

        steeringPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Sets the desired state for the module.
     * 
     * @param state The desired state w/ speed and angle
     */
    public void setModuleState(SwerveModuleState state) {
        // Optimize the state to avoid spinning further than 90 degrees
        Rotation2d currentAngle = Rotation2d.fromDegrees(wheelAngle.getAsDouble());
        state = SwerveModuleState.optimize(state, currentAngle);

        // Calculate the drive motor voltage using PID and FeedForward
        double driveOutput = drivePID.calculate(velocity.getAsDouble(), state.speedMetersPerSecond);
        double driveFeedForward = this.driveFeedForward.calculate(state.speedMetersPerSecond);

        // Calculate the steering motor voltage using PID and FeedForward
        double steeringOutput = steeringPID.calculate(currentAngle.getRadians(), state.angle.getRadians());
        double steeringFeedForward = this.steeringFeedForward.calculate(steeringPID.getSetpoint());

        // Sets voltages of motors
        this.driveMotor.setVoltage(driveOutput + driveFeedForward);
        this.steeringMotor.setVoltage(steeringOutput + steeringFeedForward);

    }

    /**
     * stops the drive and steering motors
     */
    public void stopMotors() {
        driveMotor.stopMotor();
        steeringMotor.stopMotor();
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
