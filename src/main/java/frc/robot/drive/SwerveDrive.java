// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;

/** SwerveDrive implements service drive control. */
public class SwerveDrive extends RobotDriveBase {

    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final DoubleSupplier fieldOrientation;

    /**
     * constructs the swerve drive
     * 
     * @param modules An array of four SwerveModule objects in the order: front left, front right, back left, back right
     */
    public SwerveDrive(SwerveModule[] modules, SwerveDriveKinematics kinematics, DoubleSupplier fieldOrientation) {
        this.modules = modules;
        this.kinematics = kinematics;
        this.fieldOrientation = fieldOrientation;
    }

    /**
     * Sets the swerve module states.
     * 
     * @param states array of 4 SwerveModuleStates objects
     */
    public void setModuleStates(SwerveModuleState[] states) {
        for(int i=0; i<modules.length;++i){
            modules[i].setModuleState(states[i]);
        }

        // Reset the motor watchdog timer.
        feedWatchdog();
    }
    
    @Override
    public void stopMotor() {
        for (SwerveModule module : modules) {
            module.stopMotors();
        }
    }

    @Override
    public String getDescription() {
        return "SwerveDrive";
    }
    
    /**
     * Drives the robot based on joystick inputs.
     * 
     * @param xSpeed Speed of the robot in the x direction
     * @param ySpeed Speed of the robot in the y direction
     * @param rSpeed Rotation speed of the robot
     * @param fieldRelative Whether the x and y values are relative to field
     * @param squareInputs Decreases sensitivity at low speeds (Good for rookie drivers)
     */
    public void drive(double xSpeed, double ySpeed, double rSpeed, boolean fieldRelative, boolean squareInputs) {
        //Applies deadbands to x, y, and rotation speed and multiples all values with max 
        xSpeed = MathUtil.applyDeadband(xSpeed, m_deadband) * SwerveModule.kMaxDriveSpeed;
        ySpeed = MathUtil.applyDeadband(ySpeed, m_deadband) * SwerveModule.kMaxDriveSpeed;
        rSpeed = MathUtil.applyDeadband(rSpeed, m_deadband)* SwerveModule.kMaxSteeringSpeed;
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rSpeed, Rotation2d.fromDegrees(fieldOrientation.getAsDouble()))
                : new ChassisSpeeds(xSpeed, ySpeed, rSpeed));
        setModuleStates(states);
    }

}
