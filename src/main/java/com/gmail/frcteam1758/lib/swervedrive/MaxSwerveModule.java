package com.gmail.frcteam1758.lib.swervedrive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MaxSwerveModule implements SwerveModule {

    protected final SparkMax m_drvMtr, m_strMtr;

    protected final RelativeEncoder m_drvEncoder;
    protected final AbsoluteEncoder m_strEncoder;

    protected final SparkClosedLoopController m_drvPID, m_strPID;

    protected final Translation2d m_pos;
    protected final Rotation2d m_angularOffset;
    protected final SwerveModuleState m_lockState;

    /**
     * constructs a {@link MaxSwerveModule}
     * <p>
     * The anglular offset will be determined based on the position of the module
     * @param p_drvCanId CAN id of the drive motor
     * @param dcfg drive config
     * @param p_strCanId CAN id of the steering motor
     * @param scfg steering config
     * @param p_pos Location of the module relative to the center of the robot. Positive x is forward and positve y is left
     */
    public MaxSwerveModule(int p_drvCanId, SparkMaxConfig dcfg, int p_strCanId, SparkMaxConfig scfg, Translation2d p_pos) {

        m_drvMtr = new SparkMax(p_drvCanId, SparkLowLevel.MotorType.kBrushless);
        m_strMtr = new SparkMax(p_strCanId, SparkLowLevel.MotorType.kBrushless);

        m_drvMtr.configure(dcfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_strMtr.configure(scfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_drvEncoder = m_drvMtr.getEncoder();
        m_strEncoder = m_strMtr.getAbsoluteEncoder();

        m_drvPID = m_drvMtr.getClosedLoopController();
        m_strPID = m_strMtr.getClosedLoopController();

        m_pos = p_pos;
        
        if (p_pos.getX() > 0 && p_pos.getY() > 0) {
            m_angularOffset = Rotation2d.fromDegrees(270);
        }
        else if (p_pos.getX() > 0 && p_pos.getY() < 0) {
            m_angularOffset = Rotation2d.fromDegrees(0);
        }
        else if (p_pos.getX() < 0 && p_pos.getY() < 0) {
            m_angularOffset = Rotation2d.fromDegrees(90);
        }
        else {
            m_angularOffset = Rotation2d.fromDegrees(180);
        }

        m_lockState = new SwerveModuleState(
            0,
            new Rotation2d(Math.atan(m_pos.getY() / m_pos.getX()))
        );

    }

    @Override
    public void run(SwerveModuleState p_state) {
        p_state.angle = p_state.angle.plus(m_angularOffset);

        p_state.optimize(new Rotation2d(m_strEncoder.getPosition()));

        m_drvPID.setSetpoint(p_state.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
        m_strPID.setSetpoint(p_state.angle.getRadians(), SparkMax.ControlType.kPosition);

        SmartDashboard.putNumber("setpoit", p_state.angle.getDegrees());
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(m_angularOffset);
    
        // Optimize the reference state to avoid spinning further than 90 degrees.
        desiredState.optimize(
            new Rotation2d(m_strEncoder.getPosition()));
    
        // Command driving and turning SPARKS MAX towards their respective setpoints.
        m_drvPID.setSetpoint(desiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
        m_strPID.setSetpoint(desiredState.angle.getRadians(), SparkMax.ControlType.kPosition);
      }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_drvEncoder.getPosition(),
            new Rotation2d(m_strEncoder.getPosition()).minus(m_angularOffset)
        );
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            m_drvEncoder.getVelocity(),
            new Rotation2d(m_strEncoder.getPosition()).minus(m_angularOffset)
        );
    }

    @Override
    public void lock() { run(m_lockState); }
    
    @Override
    public void close() throws Exception /* allows subclass to throw */ {
        m_strMtr.close();
        m_drvMtr.close();
    }

    @Override
    public void resetPosition() {
        m_drvEncoder.setPosition(0);
    }

    @Override
    public Translation2d getTranslation() { return m_pos; }
}
