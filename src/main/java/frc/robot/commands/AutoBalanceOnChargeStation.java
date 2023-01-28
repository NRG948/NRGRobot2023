// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Command which balances the robot on the charge station at the end of auto.
 */
public class AutoBalanceOnChargeStation extends CommandBase {

  private static final double CLIMB_SPEED = 0.25;
  private static final double APPROACH_SPEED = 0.5;
  private static final double BALANCE_THRESHOLD = 2.0; // degrees, "balanced" if within +/- TILT_EPSILON.
  private static final double MAX_TILT = 15; // maxmimum incline of the charge station

  private boolean isDrivingForward;
  private boolean wasTiltedUp;
  private double previousSpeed;
  private int pauseCounter;
  private SwerveSubsystem drivetrain;
  Rotation2d tiltAngle;

  /** Creates a new BalanceOnChargeStation command. */
  public AutoBalanceOnChargeStation(SwerveSubsystem drivetrain,boolean isDrivingForward, boolean alreadyOnChargeStation) {
    this.drivetrain = drivetrain;
    this.isDrivingForward = isDrivingForward;
    this.wasTiltedUp = alreadyOnChargeStation;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wasTiltedUp = false;
    previousSpeed = 0;
    pauseCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tiltAngle = drivetrain.getTilt();
    double currentSpeed;
    
    if (!wasTiltedUp) {
      //If the robot has never gotten onto the charge station, the robot drives foward
      currentSpeed = APPROACH_SPEED;
      wasTiltedUp = tiltAngle.getDegrees() > 9;
    } else {
      currentSpeed = calculateSpeed(tiltAngle.getDegrees());
    }

    if (previousSpeed != 0 && currentSpeed == 0) {
      pauseCounter = 50;
    } 
    if (pauseCounter > 0) {
      currentSpeed = 0;
      pauseCounter--;
  }

    currentSpeed = isDrivingForward ? currentSpeed : -currentSpeed;
    drivetrain.drive(currentSpeed, 0, 0, false, false);
    previousSpeed = currentSpeed;
  }

  private double calculateSpeed(double tiltAngle) {
    if (Math.abs(tiltAngle) <= BALANCE_THRESHOLD) {
      return 0;
    }
    return tiltAngle / MAX_TILT * CLIMB_SPEED;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pauseCounter == 1 && Math.abs(tiltAngle.getDegrees()) <= BALANCE_THRESHOLD;
  }
}