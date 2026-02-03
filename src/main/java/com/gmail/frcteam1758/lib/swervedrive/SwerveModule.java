
package com.gmail.frcteam1758.lib.swervedrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * {@link SwerveModule} defines a swerve module, including motorcontrollers AND encoders
 * <p>
 * extends {@link AutoCloseable} to allow cleanup code to be run if desired with
 * a try-resources block
 * 
 * @apiNote this interface offers no way to configure a module. Configuration is the
 * responsiblility of the implementing class. It is recomended that a reference to each
 * module be kept to allow for configuration after the module is passed to a
 * {@link BasicSwerveDrive}, a subclass of BasicSwerveDrive, or whatever class
 * coordinates your modules
 */
public interface SwerveModule extends AutoCloseable {
    
    /**
     * causes the motors owned by this {@link SwerveModule} object to
     * move the wheel in the commanded speed and direction
     * 
     * @param p_state the desired state (speed & direction)
     */
    public void run(SwerveModuleState p_state);

    /**
     * @return a {@link SwerveModulePosition} that can be used in odometry
     */
    public SwerveModulePosition getPosition();

    /**
     * @return the <i>current</i> state of this module, as a {@link SwerveModuleState}
     */
    public SwerveModuleState getState();

    /**
     * @return the module's position relative to the center of the robot, as a
     * {@link Translation2d}
     */
    public Translation2d getTranslation();

    /**
     * causes this module to point its wheel outward, such that the robot
     * is unable to roll (see below).
     * <p>
     * <code>
     * \\ //
     * <p>
     * // \\
     * </code>
     */
    public void lock();

    /**
     * releases any resources used by this object
     * <p>
     * satisfies {@link AutoCloseable}, meaning that if a {@link SwerveModule} object
     * is used as a resource in a {@code try-resources} block, this method will be
     * called automatically, even if an Exception is thrown.
     * <p>
     * Example:
     * <code>
     * try (SwerveModule sm = ...) {...}
     * </code>
     * <p>
     * At the closing <code>}</code>, {@code sm.close()} will have been called
     */
    @Override
    public void close() throws Exception;

    public void resetPosition();
}
