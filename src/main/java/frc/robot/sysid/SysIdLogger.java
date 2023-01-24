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
    private static final int DATA_ARRAY_CAPACITY = 36000;

    private double m_voltageCommand;
    private double m_motorVoltage;
    private double m_timestamp;
    private double m_startTime;
    boolean m_rotate;
    String m_testType;
    String m_mechanism;
    ArrayList<Double> m_data = new ArrayList<Double>();

    /** Construct an instance of this class. */
    public SysIdLogger() {

    }

    /** Initializes the logger. */
    public void init() {
        m_mechanism = SmartDashboard.getString("SysIdTest", "");

        SmartDashboard.putBoolean("SysIdWrongMech", isWrongMechanism());

        m_testType = SmartDashboard.getString("SysIdTestType", "");
        m_rotate = SmartDashboard.getBoolean("SysIdRotate", false);
        m_voltageCommand = SmartDashboard.getNumber("SysIdVoltageCommand", 0.0);
        m_data.ensureCapacity(DATA_ARRAY_CAPACITY);

        reset();

        m_startTime = Timer.getFPGATimestamp();
    }

    /**
     * Returns the mechanism to be characterized.
     * 
     * @return The mechanism to be characterized.
     */
    public String getMechanism() {
        return m_mechanism;
    }

    /**
     * Returns the current logging timestamp.
     * 
     * @return The current logging timestamp.
     */
    protected double getTimestamp() {
        return m_timestamp;
    }

    /**
     * Returns the voltage to apply to the mechanism's motor.
     * 
     * @return The motor voltage.
     */
    public double getMotorVoltage() {
        final double batteryVoltage = RobotController.getBatteryVoltage();

        return MathUtil.clamp(m_motorVoltage, -batteryVoltage, batteryVoltage);
    }

    /**
     * Returns whether rotational motion is being characterized.
     * 
     * @return
     */
    public boolean isRotating() {
        return m_rotate;
    }

    /** Updates the logger data from the current SysId values. */
    public void updateData() {
        m_timestamp = Timer.getFPGATimestamp();

        if (isWrongMechanism()) {
            m_motorVoltage = 0.0;
        } else if (m_testType.equals("Quasistatic")) {
            m_motorVoltage = m_voltageCommand * (m_timestamp - m_startTime);
        } else if (m_testType.equals("Dynamic")) {
            m_motorVoltage = m_voltageCommand;
        } else {
            m_motorVoltage = 0.0;
        }
    }

    /** Sends the data to the SysId tool. */
    public void sendData() {
        System.out.println(String.format("Collected %d data points.", m_data.size()));

        SmartDashboard.putBoolean("SysIdOverflow", m_data.size() >= DATA_ARRAY_CAPACITY);

        StringBuilder b = new StringBuilder();
        for (int i = 0; i < m_data.size(); ++i) {
            if (i != 0)
                b.append(",");
            b.append(m_data.get(i));
        }

        SmartDashboard.putString("SysIdTelemetry", b.toString());

        reset();
    }

    /** Appends the specified data to the log.
     * 
     * @param dataValue The data to log.
     */
    protected void addData(Double... dataValue) {
        m_data.addAll(Arrays.asList(dataValue));
    }

    /** Resets the logger. */
    public void reset() {
        m_motorVoltage = 0.0;
        m_timestamp = 0.0;
        m_startTime = 0.0;
        m_data.clear();
    }

    /** Returns true if the wrong mechanism is being profiled. */
    protected abstract boolean isWrongMechanism();
}

