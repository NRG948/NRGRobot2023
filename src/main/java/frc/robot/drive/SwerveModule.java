// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/** SwerveModule manages the drive and steering motors of a single swerve drive module. */
public class SwerveModule {
    //temp theoretical constants
    private static double kMaxDriveSpeed = 4.9; //meters per second
    private static double kDriveS = 0.2; //voltage needed to overcome friction
    private static double kDriveV = (12.0 - kDriveS)/kMaxDriveSpeed; //max voltage divided by max speed, voltage needed to maintain constant velocity
    private static double kDriveA = 0;

    private static double kMaxSteeringSpeed = 2*Math.PI;
    private static double kSteeringS = 0.2; //voltage needed to overcome friction
    private static double kSteeringV = (12.0 - kSteeringS)/kMaxSteeringSpeed; //max voltage divided by max speed, voltage needed to maintain constant velocity
    private static double kSteeringA = 0;


    private MotorController driveMotor; //
    private DoubleSupplier velocity;
    private MotorController steeringMotor;
    private DoubleSupplier angle; //gives a double as angle
    
    private PIDController drivePID = new PIDController(1.0, 0, 0);
    private PIDController steeringPID = new PIDController(1.0, 0, 0);

    private SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(kDriveS, kDriveV, kDriveA); //models motor mathematically, calculates voltage needed
    private SimpleMotorFeedforward steeringFeedForward = new SimpleMotorFeedforward(kSteeringS, kSteeringV, kSteeringA);

    /**
     * Constucts the swerve module
     * @param driveMotor Drive motor controller
     * @param velocity Supplies velocity
     * @param steeringMotor Steering motor controller
     * @param angle Supplies angle
     */
    public SwerveModule(MotorController driveMotor, DoubleSupplier velocity, MotorController steeringMotor, DoubleSupplier angle) {
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.angle = angle;
        this.velocity = velocity;
    }

    /**
     * Sets the desired state for the module
     * 
     * @param state desired state w/ speed and angle
     */
    public void setModuleState(SwerveModuleState state) {
        // Optimize the state to avoid spinning further than 90 degrees
        Rotation2d currentAngle = Rotation2d.fromDegrees(angle.getAsDouble());
        state = SwerveModuleState.optimize(state, currentAngle);

        // Calculate the drive motor voltage using PID and FeedForward
        double driveOutput = drivePID.calculate(velocity.getAsDouble(),state.speedMetersPerSecond);
        double driveFeedForward = this.driveFeedForward.calculate(state.speedMetersPerSecond);

        // Calculate the steering motor voltage using PID and FeedForward
        double steeringOutput = steeringPID.calculate(currentAngle.getRadians(),state.angle.getRadians());
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
    
}
