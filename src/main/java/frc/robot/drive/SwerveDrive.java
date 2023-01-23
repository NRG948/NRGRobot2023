// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.parameters.SwerveDriveParameters;
import frc.robot.subsystems.SwerveSubsystem;

/** SwerveDrive implements swerve drive control. */
public class SwerveDrive extends RobotDriveBase {
    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final Supplier<Rotation2d> orientationSupplier;
    private final double maxDriveSpeed;
    private final double maxRotationalSpeed;

    // The current supplied state updated by the periodic method.
    private Rotation2d orientation;

    /**
     * constructs the swerve drive
     * 
     * @param parameters          A {@link SwerveDriveParameters} object providing
     *                            information on the physical swerve drive
     *                            characteristics.
     * @param modules             An array of four {@link SwerveModule} objects in
     *                            the order: front left, front right, back left,
     *                            back right.
     * @param kinematics          A {@link SwerveDriveKinematics} object used to
     *                            convert chassis velocity into individual module
     *                            states.
     * @param orientationSupplier Supplies the robot orientation relative to the
     *                            field.
     */
    public SwerveDrive(
            SwerveDriveParameters parameters,
            SwerveModule[] modules,
            Supplier<Rotation2d> orientationSupplier) {
        this.modules = modules;
        this.kinematics = parameters.getKinematics();
        this.orientationSupplier = orientationSupplier;
        this.maxDriveSpeed = parameters.getMaxDriveSpeed();
        this.maxRotationalSpeed = parameters.getMaxRotationalSpeed();

        initializeSuppliedState();
    }

    /**
     * Initializes the supplied state.
     */
    private void initializeSuppliedState() {
        updateSuppliedState();
    }

    /**
     * Updates the supplied state.
     * <p>
     * This method **MUST* be called by the {@link #periodic()} method to ensure the
     * supplied state is up to date for subsequent use.
     */
    private void updateSuppliedState() {
        orientation = orientationSupplier.get();
    }

    /**
     * Sets the swerve module states.
     * 
     * @param states An array of four {@link SwerveModuleState} objects in the
     *               order: front left, front right, back left, back right
     */
    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxDriveSpeed);

        for (int i = 0; i < modules.length; ++i) {
            modules[i].setModuleState(states[i]);
        }

        // Reset the motor watchdog timer.
        feedWatchdog();
    }

    @Override
    public void stopMotor() {
        for (SwerveModule module : modules) {
            module.stopMotors();
        }
    }

    @Override
    public String getDescription() {
        return "SwerveDrive";
    }

    /**
     * Returns the orientation of the robot frame relative to the field.
     * 
     * @return The orientation of the robot frame.
     */
    public Rotation2d getOrientation() {
        return orientation;
    }

    /**
     * Drives the robot based on joystick inputs.
     * 
     * @param xSpeed        Speed of the robot in the x direction.
     * @param ySpeed        Speed of the robot in the y direction.
     * @param rSpeed        Rotation speed of the robot.
     * @param fieldRelative Whether the x and y values are relative to field.
     * @param squareInputs  Decreases sensitivity at low speeds.
     */
    public void drive(double xSpeed, double ySpeed, double rSpeed, boolean fieldRelative, boolean squareInputs) {
        // Applies deadbands to x, y, and rotation joystick values and multiples all
        // values with max speed.
        xSpeed = MathUtil.applyDeadband(xSpeed, m_deadband) * m_maxOutput * maxDriveSpeed;
        ySpeed = MathUtil.applyDeadband(ySpeed, m_deadband) * m_maxOutput * maxDriveSpeed;
        rSpeed = MathUtil.applyDeadband(rSpeed, m_deadband) * m_maxOutput * maxRotationalSpeed;

        if (squareInputs) {
            xSpeed *= xSpeed;
            ySpeed *= ySpeed;
            rSpeed *= rSpeed;
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rSpeed, orientation)
                        : new ChassisSpeeds(xSpeed, ySpeed, rSpeed));

        setModuleStates(states);
    }

    /**
     * Returns the swerve module positions.
     * 
     * @return Swerve module positions.
     */
    public SwerveModulePosition[] getModulesPositions() {
        SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];
        for (int i = 0; i < modules.length; i++) {
            modulePosition[i] = modules[i].getPosition();
        }
        return modulePosition;
    }

    /**
     * This method is called periodically by the {@link SwerveSubsystem}. It is used
     * to update drive-specific state.
     */
    public void periodic() {
        updateSuppliedState();

        for (SwerveModule module : modules) {
            module.periodic();
        }
    }

    /**
     * Adds the SwerveModule layouts to the shuffleboard tab.
     * 
     * @param tab The suffleboard tab to add layouts
     */
    public void addShuffleboardLayouts(ShuffleboardTab tab) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].addShuffleboardLayout(tab)
                    .withSize(3, 2)
                    .withPosition((i * 3) % 6, ((i / 2) * 2) % 4);
        }
    }
}
