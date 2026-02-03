package com.gmail.frcteam1758.lib.swervedrive;

import java.util.function.Supplier;

import com.gmail.frcteam1758.lib.swervedrive.control.SwerveDriveInput;
import com.gmail.frcteam1758.lib.swervedrive.control.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * defines a fully-integrated swerve drive which o
 */
public class SwerveChassis implements AutoCloseable {
    
    protected SwerveDriveInput m_ctrl, m_auto;

    protected SwerveModule[] m_modules;

    protected SwerveDriveKinematics m_kinematics;

    protected SwerveDriveOdometry m_odometry;

    protected double m_maxSpeed;

    protected Supplier<Rotation2d> m_angleSupplier;

    public void run(SwerveDriveState p_state) {

        if (p_state.lock) {
            for (var i : m_modules) i.lock();
            return;
        }
        SwerveModuleState[] l_states = m_kinematics.toSwerveModuleStates(p_state.speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(l_states, m_maxSpeed);

        for (int i = 0; i < 4; ++i) {
            m_modules[i].run(l_states[i]);

            SmartDashboard.putNumber(
                "delta of " + Integer.toString(i),
                l_states[i].speedMetersPerSecond - getModuleStates()[i].speedMetersPerSecond
            );
        }
    }

    public void performTeleop() { run(m_ctrl.getCommandedState()); }

    public void performAuto() { run(m_auto.getCommandedState()); }

    /**
     * Constructs an {@link SwerveChassis}
     * @param p_ctrl a {@link SwerveDriveInputs} object to provide control during the teleop period
     * @param p_auto a {@link SwerveDriveInputs} object to provide control during the auton period
     * @param p_modules a {@link SwerveModule}{@code []} of the modules to be used
     * @param p_maxSpeed the maximum attainable/allowable translation speed
     * @param p_angleSupplier a {@link Supplier}{@code <}{@link Rotation2d}{@code >} which will be
     * used to obtain the robot's orientation. If {@code null} is given, any attemp to use odometry will
     * result in a {@link SwerveChassis.OdometryUnavalibleError}
     */
    public SwerveChassis(
        SwerveDriveInput p_ctrl,
        SwerveDriveInput p_auto,
        SwerveModule[] p_modules,
        double p_maxSpeed,
        Supplier<Rotation2d> p_angleSupplier
    ) {
        m_ctrl = p_ctrl;
        m_modules = p_modules;
        m_auto = p_auto;
        m_maxSpeed = p_maxSpeed;
        m_angleSupplier = p_angleSupplier;

        Translation2d[] l_translations = new Translation2d[m_modules.length];

        for (int i = 0; i < m_modules.length; ++i) {
            l_translations[i] = m_modules[i].getTranslation();
        }

        m_kinematics = new SwerveDriveKinematics(l_translations);

        if (p_angleSupplier != null) {
            m_odometry = new SwerveDriveOdometry(m_kinematics, p_angleSupplier.get(), getModulePositions());
        }
    }

    /**
     * releases any resoures used by this object
     * <p>
     * Satisfies {@link AutoCloseable}, meaning that if a {@link SwerveChassis} is
     * used as a resource in a {@code try-resources} block, this method will be called
     * automatically, even if an Exception is thrown
     * <p>
     * Example:
     * <code>
     * try (BasicSwerveDrive4x sd = ...) {...}
     * </code>
     * <p>
     * At the closing <code>}</code>, {@code sd.close()} will have been called
     */
    @Override
    public void close() throws Exception {
        for (var i : m_modules) i.close();
    }

    /**
     * gets one of this {@link BasicSwerveDrive}'s {@link SwerveModule}s
     * @param idx the module to get
     * @return the module identified by {@code idx}
     * @throws ArrayIndexOutOfBoundsException if the requested module does not exist
     */
    public SwerveModule getModule(int idx) {

        return m_modules[idx];
    }

    public SwerveModulePosition[] getModulePositions() {

        SwerveModulePosition[] l_positions = new SwerveModulePosition[m_modules.length];

        for (int i = 0; i < m_modules.length; ++i) {
            l_positions[i] = m_modules[i].getPosition();
        }

        return l_positions;
    }

    public SwerveModuleState[] getModuleStates() {

        SwerveModuleState[] l_states = new SwerveModuleState[m_modules.length];

        for (int i = 0; i < m_modules.length; ++i) {
            l_states[i] = m_modules[i].getState();
        }

        return l_states;
    }

    public Pose2d getPose() {

        if (m_odometry == null) {
            throw new OdometryUnavalibleError();
        }

        return m_odometry.update(m_angleSupplier.get(), getModulePositions());
    }

    public void resetPose(Pose2d p_pose) {
        m_odometry.resetPosition(m_angleSupplier.get(), getModulePositions(), p_pose);
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return m_kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Thrown when a {@link SwerveChassis}' {@link #getPose()} method is called but
     * no {@link Supplier}{@code <}{@link Rotation2d}{@code >} was provided to
     * enable Odometry
     */
    public static class OdometryUnavalibleError extends Error {
        public OdometryUnavalibleError() {
            super("This SwerveChassis is not equiped to provide odometry");
        }
    }
}
