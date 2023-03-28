// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.nrg948.autonomous.AutonomousCommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Subsystems;

@AutonomousCommand(name="[TEST] Measure Shooter kS")
public class MeasureShooterKS extends CommandBase {

  private final ShooterSubsystem shooter;
  private final Timer timer = new Timer();

  private static final double VOLTS_PER_SECOND = 0.1;
  private double voltage;

  /** Creates a new MeasureShooterKS. */
  public MeasureShooterKS(Subsystems subsystems) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = subsystems.shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.disable();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    voltage = timer.get() * VOLTS_PER_SECOND;
    shooter.setMotorVoltages(voltage, voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
    timer.stop();
    System.out.println("Shooter kS = " + voltage);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.getCurrentBottomRPM() > 1;
  }
}
