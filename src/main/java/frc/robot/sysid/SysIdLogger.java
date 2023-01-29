// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sysid;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An abstract base class enabling SysId integration. */
public abstract class SysIdLogger {
  private static final String SYS_ID_TEST = "SysIdTest";
  private static final String SYS_ID_TEST_TYPE = "SysIdTestType";
  private static final String SYS_ID_ROTATE = "SysIdRotate";
  private static final String SYS_ID_VOLTAGE_COMMAND = "SysIdVoltageCommand";
  private static final String SYS_ID_ACK_NUMBER = "SysIdAckNumber";
  private static final String SYS_ID_WRONG_MECH = "SysIdWrongMech";
  private static final String SYS_ID_OVERFLOW = "SysIdOverflow";
  private static final String SYS_ID_TELEMETRY = "SysIdTelemetry";

  private static final String QUASISTATIC_TEST = "Quasistatic";
  private static final String DYNAMIC_TEST = "Dynamic";

  private static final int DATA_ARRAY_CAPACITY = 36000;

  private double voltageCommand;
  private double motorVoltage;
  private double timestamp;
  private double startTime;
  private boolean rotate;
  private String testType;
  private String mechanism;
  private double ackNumber;
  private ArrayList<Double> data = new ArrayList<Double>();

  /** Construct an instance of this class. */
  public SysIdLogger() {
    SmartDashboard.putNumber(SYS_ID_VOLTAGE_COMMAND, 0.0);
    SmartDashboard.putString(SYS_ID_TEST_TYPE, "");
    SmartDashboard.putString(SYS_ID_TEST, "");
    SmartDashboard.putBoolean(SYS_ID_ROTATE, false);
    SmartDashboard.putBoolean(SYS_ID_OVERFLOW, false);
    SmartDashboard.putBoolean(SYS_ID_WRONG_MECH, false);
    SmartDashboard.putNumber(SYS_ID_ACK_NUMBER, ackNumber);
  }

  /** Initializes the logger. */
  public void init() {
    mechanism = SmartDashboard.getString(SYS_ID_TEST, "");
    testType = SmartDashboard.getString(SYS_ID_TEST_TYPE, "");
    rotate = SmartDashboard.getBoolean(SYS_ID_ROTATE, false);
    voltageCommand = SmartDashboard.getNumber(SYS_ID_VOLTAGE_COMMAND, 0.0);
    ackNumber = SmartDashboard.getNumber(SYS_ID_ACK_NUMBER, 0.0);

    data.ensureCapacity(DATA_ARRAY_CAPACITY);

    SmartDashboard.putBoolean(SYS_ID_WRONG_MECH, isWrongMechanism());

    reset();

    startTime = Timer.getFPGATimestamp();
  }

  /**
   * Returns the mechanism to be characterized.
   * 
   * @return The mechanism to be characterized.
   */
  public String getMechanism() {
    return mechanism;
  }

  /**
   * Returns the current logging timestamp.
   * 
   * @return The current logging timestamp.
   */
  protected double getTimestamp() {
    return timestamp;
  }

  /**
   * Returns the voltage to apply to the mechanism's motor.
   * 
   * @return The motor voltage.
   */
  public double getMotorVoltage() {
    final double batteryVoltage = RobotController.getBatteryVoltage();

    return MathUtil.clamp(motorVoltage, -batteryVoltage, batteryVoltage);
  }

  /**
   * Returns whether rotational motion is being characterized.
   * 
   * @return
   */
  public boolean isRotating() {
    return rotate;
  }

  /** Updates the logger data from the current SysId values. */
  public void updateData() {
    timestamp = Timer.getFPGATimestamp();

    if (isWrongMechanism()) {
      motorVoltage = 0.0;
    } else if (testType.equals(QUASISTATIC_TEST)) {
      motorVoltage = voltageCommand * (timestamp - startTime);
    } else if (testType.equals(DYNAMIC_TEST)) {
      motorVoltage = voltageCommand;
    } else {
      motorVoltage = 0.0;
    }
  }

  /** Sends the data to the SysId tool. */
  public void sendData() {
    System.out.println(String.format("Collected %d data points.", data.size()));

    SmartDashboard.putBoolean(SYS_ID_OVERFLOW, data.size() >= DATA_ARRAY_CAPACITY);

    StringBuilder b = new StringBuilder();

    final String type = testType.equals(DYNAMIC_TEST) ? "fast" : "slow";
    final String direction = voltageCommand > 0 ? "forward" : "backward";

    b.append(String.format("%s-%s;", type, direction));

    for (int i = 0; i < data.size(); ++i) {
      if (i != 0) {
        b.append(",");
      }
      b.append(data.get(i));
    }

    SmartDashboard.putString(SYS_ID_TELEMETRY, b.toString());
    SmartDashboard.putNumber(SYS_ID_ACK_NUMBER, ++ackNumber);

    reset();
  }

  /**
   * Appends the specified data to the log.
   * 
   * @param dataValue The data to log.
   */
  protected void addData(Double... dataValue) {
    data.addAll(Arrays.asList(dataValue));
  }

  /** Resets the logger. */
  public void reset() {
    motorVoltage = 0.0;
    timestamp = 0.0;
    startTime = 0.0;
    data.clear();
  }

  /** Returns true if the wrong mechanism is being profiled. */
  protected abstract boolean isWrongMechanism();
}
