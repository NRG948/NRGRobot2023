// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;

import com.nrg948.autonomous.AutonomousCommandMethod;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;

public class PlayMusic extends CommandBase {
  private static File SONG_DIR = new File(Filesystem.getDeployDirectory(), "songs");
  private final SwerveSubsystem drivetrain;
  private final String filePath;

  @AutonomousCommandMethod(name = "Play Song No. 9")
  public static Command playSongNo9(Subsystems subsystems) {
    return new PlayMusic(subsystems.drivetrain, new File(SONG_DIR, "song9.chrp").toString());
  }

  /** Creates a new PlayMusic. */
  public PlayMusic(SwerveSubsystem drivetrain, String filePath) {
    this.drivetrain = drivetrain;
    this.filePath = filePath;

    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("BEGIN: Playing " + filePath);
    drivetrain.playMusic(filePath);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMusic();
    System.out.println("END: Playing " + filePath);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !drivetrain.isMusicPlaying();
  }
}
