// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** Container class that manages robot subsystems. */
public class Subsystems {
  public final SwerveSubsystem drivetrain = new SwerveSubsystem();
  public final CubeVisionSubsystem cubeVision = new CubeVisionSubsystem();
  public final AprilTagSubsystem aprilTag = new AprilTagSubsystem();
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final AdressableLEDSubsystem leds = new AdressableLEDSubsystem();
  public final ShooterSubsystem shooter = new ShooterSubsystem();
  public final IndexerSubsystem indexer = new IndexerSubsystem();

  public final Subsystem[] all = new Subsystem[] {
    drivetrain,
    cubeVision,
    aprilTag,
    shooter,
    indexer,
    intake,
    leds,
  };
}
