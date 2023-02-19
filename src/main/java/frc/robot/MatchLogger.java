// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class MatchLogger {

    // TODO: Research if we need to create some sort of instance of the drivers
    // station...

    /**
     * Prints Tele-Op starts now
     */
    public void printTeleOpStart() {
        System.out.println("\n Tele-Op starts now!\n");
    }

    /**
     * Prints Autonomous starts now
     */
    public void printAutoStart() {
        System.out.println("\n Autonomous starts now!\n");
    }

    /**
     * Prints if the robot has been emergency-stopped.
     */
    public void printIfEStopped() {
        if (DriverStation.isEStopped()) {
            System.out.println("Robot has been E-Stopped!!! \n Robot has been E-Stopped \n");
        }
    }

    /**
     * Print match details -
     * ex. Qualification match number: 8
     * Red alliance drivers' station: 3
     */
    public void printMatchDetails() {
        System.out.println(
            "\nmatch type: " + DriverStation.getMatchType() + 
            "\nmatch number: " + DriverStation.getMatchNumber() + 
            "\nalliance: " + DriverStation.getAlliance() + 
            "\ndrivers' station: " + DriverStation.getLocation());
    }
}
