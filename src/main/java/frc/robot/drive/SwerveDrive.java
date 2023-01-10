// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;

/** SwerveDrive implements service drive control. */
public class SwerveDrive extends RobotDriveBase {

    private SwerveModule[] modules;

    /**
     * constructs the swerve drive
     * 
     * @param modules An array of four SwerveModule objects in the order: front left, front right, back left, back right
     */
    public SwerveDrive(SwerveModule[] modules) {
        this.modules = modules;
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

}
