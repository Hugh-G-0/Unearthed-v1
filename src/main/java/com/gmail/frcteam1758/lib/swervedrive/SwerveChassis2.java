package com.gmail.frcteam1758.lib.swervedrive;

import java.util.function.Supplier;

import com.gmail.frcteam1758.lib.swervedrive.control.SwerveDriveInput;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.NavSubsystem;

public class SwerveChassis2 extends SwerveChassis {

    protected SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        this.m_kinematics, NavSubsystem.X.getAngle(), this.getModulePositions(), this.getPose()
    );
    
    /**
     * Constructs an {@link SwerveChassis}
     * @param pCtrl a {@link SwerveDriveInputs} object to provide control during the teleop period
     * @param pAuto a {@link SwerveDriveInputs} object to provide control during the auton period
     * @param pModules a {@link SwerveModule}{@code []} of the modules to be used
     * @param pMaxSpeed the maximum attainable/allowable translation speed
     * @param pAngleSupplier a {@link Supplier}{@code <}{@link Rotation2d}{@code >} which will be
     * used to obtain the robot's orientation. If {@code null} is given, any attempt to use odometry will
     * result in a {@link SwerveChassis.OdometryUnavalibleError}
     * <p>
     * {@link SwerveDriveInput.NO_INPUT} should be used if no {@link SwerveDriveInput} is availible/desired
     * <p>
     * {@code null} should be used if no gyro is availible
     */
    public SwerveChassis2(
        SwerveDriveInput pCtrl,
        SwerveDriveInput pAuto,
        SwerveModule[] pModules,
        double pMaxSpeed,
        Supplier<Rotation2d> pAngleSupplier
    ) {
        super(pCtrl, pAuto, pModules, pMaxSpeed, pAngleSupplier);
    }

    @Override
    public void resetPose(Pose2d newPose) {
        this.poseEstimator.resetPose(newPose);
    }

    @Override
    public Pose2d getPose() {
        return this.poseEstimator.update(this.m_angleSupplier.get(), this.getModulePositions());
    }

    public Pose2d getPose(Pose2d visionPose, double visionTimestamp) {

        if (NavSubsystem.X.hasVisionPose()) {
            this.poseEstimator.addVisionMeasurement(visionPose, visionTimestamp);
        }

        return this.poseEstimator.update(this.m_angleSupplier.get(), this.getModulePositions());
    }
}
