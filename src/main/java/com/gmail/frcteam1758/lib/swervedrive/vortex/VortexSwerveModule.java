package com.gmail.frcteam1758.lib.swervedrive.vortex;

import com.gmail.frcteam1758.lib.swervedrive.SwerveModule;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.gmail.frcteam1758.lib.swervedrive.SwerveChassis;

/**
 * class representing a MaxSwerve Module equiped with Vortex motors.
 * 
 * Should be used with {@link SwerveChassis} or similar
 */
public class VortexSwerveModule implements SwerveModule{
    
    // driveing and steering mototrcontrollers
    protected final SparkFlex driveMC;
    protected final SparkMax  steerMC;

    //driving and steering encoders
    protected final RelativeEncoder driveEncoder;
    protected final AbsoluteEncoder steerEncoder;

    //driving and steering closed-loop (PID) controllers
    protected final SparkClosedLoopController drivePID;
    protected final SparkClosedLoopController steerPID;

    //fetchable position data
    protected final Translation2d position;

    //anglular offset from forward
    protected final Rotation2d    angularOffset;

    //orientation for locked mode
    protected final SwerveModuleState lockedState;

    /**
     * Constructs a {@link VortexSwerveModule}
     * 
     * @param pDriveCAN CAN id of driving (Vortex) motor
     * @param pDriveCfg {@link SparkFlexConfig} object to configure SparkFlex
     * @param pSteerCan CAN id of steering (Neo550) motor
     * @param pSteerConfig {@link SparkMaxConfig} object to configure SparkMax
     * @param pPosition position of module relative to center of rotation
     */
    public VortexSwerveModule(
        int pDriveCAN,
        SparkFlexConfig pDriveCfg,
        int pSteerCan,
        SparkMaxConfig pSteerConfig,
        Translation2d pPosition
    ) {
        VortexSwerveDefaults.prepareConfigs();

        this.driveMC = new SparkFlex(pDriveCAN, MotorType.kBrushless);
        this.steerMC = new SparkMax (pSteerCan, MotorType.kBrushless);

        REVLibError eD = this.driveMC.configure(pDriveCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        REVLibError eS = this.steerMC.configure(pDriveCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (eD != REVLibError.kOk || eS != REVLibError.kOk) {
            throw new RuntimeException("mc config error");
        }

        this.driveEncoder = this.driveMC.getEncoder        ();
        this.steerEncoder = this.steerMC.getAbsoluteEncoder();

        this.drivePID = this.driveMC.getClosedLoopController();
        this.steerPID = this.steerMC.getClosedLoopController();

        this.position = pPosition;

        //deduce angular offset based on position
        if (pPosition.getX() > 0 && pPosition.getY() > 0) {
            this.angularOffset = Rotation2d.fromDegrees(270);
        }
        else if (pPosition.getX() > 0 && pPosition.getY() < 0) {
            this.angularOffset = Rotation2d.fromDegrees(0);
        }
        else if (pPosition.getX() < 0 && pPosition.getY() < 0) {
            this.angularOffset = Rotation2d.fromDegrees(90);
        }
        else {
            this.angularOffset = Rotation2d.fromDegrees(180);
        }

        //compute appropriate lock angle from position
        this.lockedState = new SwerveModuleState(
            0,
            new Rotation2d(Math.atan2(pPosition.getY(),pPosition.getX()))
        );
    }

    /**
     * Constructs a {@link VortexSwerveModule} with default configs (usually fine)
     * 
     * @param pDriveCAN CAN id of driving (Vortex) motor
     * @param pSteerCan CAN id of steering (Neo550) motor
     * @param pPosition position of module relative to center of rotation
     */
    public VortexSwerveModule(int pDriveCAN, int pSteerCAN, Translation2d pPosition) {
        this(pDriveCAN, VortexSwerveDefaults.kDriveCfg, pSteerCAN, VortexSwerveDefaults.kSteerCfg, pPosition);
    }

    /**
     * reapplys configs in case of error. (shouldn't be used)
     */
    public void reConfig() {
        this.driveMC.configure(
            VortexSwerveDefaults.kDriveCfg,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        this.steerMC.configure(
            VortexSwerveDefaults.kSteerCfg,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    /**
     * causes this module's motors to approch the given {@link SwerveModuleState}
     * <p>
     * will usually be called by {@link SwerveChassis#run()}, not directly
     * <p>
     * editing some SmartDahboard calls may be helpful for debugging
     */
    @Override
    public void run(SwerveModuleState pState) {

        pState = new SwerveModuleState(pState.speedMetersPerSecond, pState.angle);

        //account for angular offset of module
        pState.angle = pState.angle.plus(this.angularOffset);

        //consider reversing direction to avoid >90deg turns
        pState.optimize(new Rotation2d(this.steerEncoder.getPosition()));

        SmartDashboard.putNumber(
            "m%d aVel".formatted(this.driveMC.getDeviceId()),
            Math.abs(this.getState().speedMetersPerSecond)
        );

        SmartDashboard.putNumber(
            "m%d tVel".formatted(this.driveMC.getDeviceId()),
            Math.abs(pState.speedMetersPerSecond)
        );

        SmartDashboard.putNumber(
            "m%d dVel".formatted(this.driveMC.getDeviceId()),
            Math.abs(this.getState().speedMetersPerSecond) - Math.abs(pState.speedMetersPerSecond)
        );

        SmartDashboard.putNumber(
            "m%d cVel".formatted(this.driveMC.getDeviceId()),
            Math.abs(pState.speedMetersPerSecond)
        );

        try  {
            SmartDashboard.putNumber(
                "m%d rVel".formatted(this.driveMC.getDeviceId()),
                Math.abs(pState.speedMetersPerSecond) / Math.abs(this.getState().speedMetersPerSecond)
            );
        } catch (Exception e) {}

        this.drivePID.setSetpoint(pState.speedMetersPerSecond, ControlType.kVelocity);
        this.steerPID.setSetpoint(pState.angle.getRadians()  , ControlType.kPosition);


    }

    @Override
    public SwerveModulePosition getPosition() {

        return new SwerveModulePosition(

            this.driveEncoder.getPosition(),
            new Rotation2d(this.steerEncoder.getPosition()).minus(this.angularOffset)
        );
    }

    @Override
    public SwerveModuleState getState() {

        return new SwerveModuleState(
            this.driveEncoder.getVelocity(),
            new Rotation2d(this.steerEncoder.getPosition()).minus(this.angularOffset)
        );
    }

    /**
     * causes this module to enter "locked"/"X" configuration.
     * <p>
     * Should generally be called by {@link SwerveChassis#run()}, not directly
     */
    @Override
    public void lock() { this.run(this.lockedState); }

    /**
     * resets the accumulation of {@link #getPosition()}
     */
    @Override
    public void resetPosition() {
        this.driveEncoder.setPosition(0);
    }

    @Override
    public Translation2d getTranslation() {

        return this.position;
    }

    /**
     * releases system resources on power-down
     * <p>
     * should generally be called by {@link SwerveChassis#close()}, not directly
     */
    @Override
    public void close() throws Exception /* allow subclass to throw exception*/ {
        this.driveMC.close();
        this.steerMC.close();
    }
}
