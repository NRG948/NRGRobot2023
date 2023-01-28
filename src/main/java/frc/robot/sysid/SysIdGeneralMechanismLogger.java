// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sysid;

import java.util.Set;
import java.util.function.DoubleSupplier;

/**
 * A class for logging characterization data for Arm, Elevator and Simple
 * mechanisms to SysId.
 */
public class SysIdGeneralMechanismLogger extends SysIdLogger {
    private static final Set<String> SUPPORTED_MECHANISMS = Set.of("Arm", "Elevator", "Simple");

    private final DoubleSupplier positionSupplier;
    private final DoubleSupplier velocitySupplier;

    /**
     * Constructs an instance of this class.
     * 
     * @param positionSupplier Supplies the mechanism position.
     * @param velocitySupplier Supplies the mechanism velocity.
     */
    public SysIdGeneralMechanismLogger(DoubleSupplier positionSupplier, DoubleSupplier velocitySupplier) {
        this.positionSupplier = positionSupplier;
        this.velocitySupplier = velocitySupplier;
    }

    @Override
    protected boolean isWrongMechanism() {
        String mechanism = getMechanism();

        return !SUPPORTED_MECHANISMS.contains(mechanism);
    }

    /**
     * Records the mechanism data for the current timestamp.
     */
    public void logData() {
        updateData();

        double timestamp = getTimestamp();
        double motorVoltage = getMotorVoltage();
        double position = positionSupplier.getAsDouble();
        double velocity = velocitySupplier.getAsDouble();

        addData(timestamp, motorVoltage, position, velocity);
    }
}
