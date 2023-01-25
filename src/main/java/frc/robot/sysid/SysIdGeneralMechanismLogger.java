// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sysid;

import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class SysIdGeneralMechanismLogger extends SysIdLogger{
    private final DoubleSupplier positionSupplier;
    private final DoubleSupplier velocitySupplier;

    public SysIdGeneralMechanismLogger(DoubleSupplier positionSupplier, DoubleSupplier velocitySupplier){
        this.positionSupplier= positionSupplier;
        this.velocitySupplier = velocitySupplier;
    }
    
    @Override
    protected boolean isWrongMechanism(){
        String mechanism = getMechanism();

        return !mechanism.equals("Simple");
    }

    public void logData(){
        updateData();

        double timestamp = getTimestamp();
        double motorVoltage = getMotorVoltage();
        double position = positionSupplier.getAsDouble();
        double velocity = velocitySupplier.getAsDouble();
        
        addData(timestamp, motorVoltage, position, velocity);
    }
}
