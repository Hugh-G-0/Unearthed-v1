package com.gmail.frcteam1758.lib.swervedrive.vortex;

import com.gmail.frcteam1758.lib.swervedrive.SwerveModule;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
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
        this.driveMC = new SparkFlex(pDriveCAN, MotorType.kBrushless);
        this.steerMC = new SparkMax (pSteerCan, MotorType.kBrushless);

        this.driveMC.configure(pDriveCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.steerMC.configure(pDriveCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

    public VortexSwerveModule(int pDriveCAN, int pSteerCAN, Translation2d pPosition) {
        this(pDriveCAN, VortexSwerveDefaults.kDriveCfg, pSteerCAN, VortexSwerveDefaults.kSteerCfg, pPosition);
    }

    @Override
    public void run(SwerveModuleState pState) {

        //account for angular offset of module
        pState.angle = pState.angle.plus(this.angularOffset);

        //consider reversing direction to avoid >90deg turns
        pState.optimize(new Rotation2d(this.steerEncoder.getPosition()));

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

    @Override
    public void lock() { this.run(this.lockedState); }

    @Override
    public void resetPosition() {
        this.driveEncoder.setPosition(0);
    }

    @Override
    public Translation2d getTranslation() {

        // avoid giving a reference
        return this.position.times(1);
    }

    @Override
    public void close() throws Exception /* allow subclass to throw exception*/ {
        this.driveMC.close();
        this.steerMC.close();
    }
}
