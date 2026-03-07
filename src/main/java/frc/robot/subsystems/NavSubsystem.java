package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.visiondeps.PhotonVisionWrapper;

public class NavSubsystem extends SubsystemBase {

    public static final NavSubsystem X = new NavSubsystem();

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private Rotation2d oldAngle = new Rotation2d();
    private Rotation2d newAngle = new Rotation2d();

    // Vision tracking fields
    private Pose2d visionPose = null;
    private double visionTimestamp = 0;

    private NavSubsystem() {}

    @Override
    public void periodic() {
        // periodic vision stuff
        SmartDashboard.putNumber("gyro (deg)", this.getAngle().getDegrees());

        this.oldAngle = this.newAngle;
        this.newAngle = this.getAngle();
    }

    public Pose2d getPose() {
        if (this.hasVisionPose()) {
            return this.getVisionPose();
        }
        return DriveSubsystem.X.getOdometricPose();
    }

    public void zeroAngle() {
        this.gyro.setAngleAdjustment(-this.getAngle().getDegrees());
    }

    /**
     * @return the orientation of the robot
     */
    public Rotation2d getAngle() {
        return this.gyro.getRotation2d().times(-1);
    }

    public AngularVelocity getAngVelocity() {
        return AngularVelocity.ofBaseUnits(
            this.newAngle.minus(this.oldAngle).times(50).getRadians(),
            Units.RadiansPerSecond
        );
    }

    /**
     * @return the position of the robot as determined by vision systems
     */
    private Pose2d getVisionPose() {
        return this.visionPose;
    }

    /**
     * @return whether or not a viable vision-based pose is available
     */
    public boolean hasVisionPose() {
        this.updateVisionPose(PhotonVisionWrapper.X);

        return this.visionPose != null;
    }

    /**
     * @return the timestamp of the available vision pose (if it exists)
     */
    public double getVisionTimestamp() {
        return this.visionTimestamp;
    }

    /**
     * Call this method periodically to update the vision pose.
     */
    private void updateVisionPose(PhotonVisionWrapper photonVision) {
        if (photonVision == null) {
            this.visionPose = null;
            this.visionTimestamp = 0;
            return;
        }

        Pose2d prevPose = DriveSubsystem.X.getOdometricPose();
        var result = photonVision.getLatestResult();
        var estimateOpt = photonVision.getEstimatedGlobalPose(prevPose, result);

        if (estimateOpt.isPresent()) {
            var estimate = estimateOpt.get();
            this.visionPose = estimate.estimatedPose.toPose2d();
            this.visionTimestamp = estimate.timestampSeconds;
        } else {
            this.visionPose = null;
            this.visionTimestamp = 0;
        }
    }
}
