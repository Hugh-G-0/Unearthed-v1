package com.gmail.frcteam1758.lib.swervedrive.control;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * interface defining a class that provides input to SwerveDrives in the
 * form of a {@code ChassisSpeeds} object, in robot-oriented speeds. If
 * field oriented speeds are desired, the implementing {@code class} is
 * responsible for performing the conversion
 * <p>
 * A lambda (<code>(...) -> {...}</code>) is a valid {@link SwerveDriveInput}
 * <p>
 * {@link ChassisSpeeds#fromFieldRelativeSpeeds()} is provded for this purpose
 * <p>
 * {@link SwerveChassis} uses {@link SwerveDriveInput} objects to
 * obtain input in both Teleop and Auton.
 */
@FunctionalInterface
public interface SwerveDriveInput {
    
    /**
     * gets the speads commanded by the driver
     * @return a {@code SwerveDriveState} object represnting the desired robot-oriented speeds
     */
    public SwerveDriveState getCommandedState();

    /**
     * always commands the robot to stop moving. Should be used instead of {@code null}
     * if a {@link SwerveDriveInput} is unavalible. For example, early in the season,
     * autonomous may not exist yet. If {@code null} is supplied to a {@link SwerveChassis}
     * as the auton info supplier, and auton is accedentally requested, a
     * {@link NullPointerException} could (and likely would) be thrown, resulting in a
     * crash (or worse).
     */
    public static final SwerveDriveInput NO_INPUT = () -> SwerveDriveState.LOCKED;
}