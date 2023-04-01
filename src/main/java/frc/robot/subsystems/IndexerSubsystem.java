// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.CAN;
import frc.robot.Constants.RobotConstants.DigitalIO;

public class IndexerSubsystem extends SubsystemBase {

  private static final double MAX_RPM = 2000;
  private static final double RPM_PER_VOLT = 493.9;

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue INDEXER_SHOOT_RPM = new RobotPreferences.DoubleValue("Shooter",
      "Indexer Shoot RPM", 1000.0); // Estimates
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue INDEXER_INTAKE_RPM = new RobotPreferences.DoubleValue("Shooter",
      "Indexer Intake RPM", 250.0);

  private static final double KS = 0.5;
  private static final double GEAR_RATIO = 5.0 / 1;

  private double goalRPM = 0;
  private boolean isEnabled = false;
  private boolean cubeDetected = false;

  private final CANSparkMax indexerMotor = new CANSparkMax(CAN.SparkMax.INDEXER, MotorType.kBrushless);
  private final DigitalInput beamBreak = new DigitalInput(DigitalIO.INDEXER_BEAM_BREAK);

  private final BooleanLogEntry cubeDetectedLogger = new BooleanLogEntry(DataLogManager.getLog(), "Indexer/Cube Detected");

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {
    indexerMotor.setIdleMode(IdleMode.kBrake);
    indexerMotor.setInverted(true);
  }

  /**
   * Returns whether the cube is detected by the beam break.
   * 
   * @return whether the cube is detected by the beam break.
   */
  public boolean isCubeDetected() {
    return cubeDetected;
  }

  /** Sets the goal RPM of the indexer */
  public void setShootRPM() {
    isEnabled = true;
    goalRPM = INDEXER_SHOOT_RPM.getValue();
  }

  public void setIntakeRPM() {
    isEnabled = true;
    goalRPM = INDEXER_INTAKE_RPM.getValue();
  }

  public void disable() {
    isEnabled = false;
    goalRPM = 0;
    stopMotor();
  }

  public void stopMotor() {
    indexerMotor.stopMotor();
  }

  public void runMotor(double power) {
    indexerMotor.set(power);
  }

  @Override
  public void periodic() {
    cubeDetected = !beamBreak.get();

    if (isEnabled) {
      double voltage = goalRPM / RPM_PER_VOLT + KS;
      indexerMotor.setVoltage(voltage);
    }

    cubeDetectedLogger.append(cubeDetected);
  }
}