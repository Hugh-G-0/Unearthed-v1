package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavSubsystem extends SubsystemBase {

    public static final NavSubsystem X = new NavSubsystem();

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private Rotation2d oldAngle = new Rotation2d();
    private Rotation2d newAngle = new Rotation2d();
    
    private NavSubsystem() {}

    @Override
    public void periodic() {
        // periodic vision stuff

        SmartDashboard.putNumber("gyro (deg)", this.getAngle().getDegrees());

        this.oldAngle = this.newAngle;
        this.newAngle = this.getAngle();
    }

    public Pose2d getPose() {

        if (this.hasVisionPose()) { return this.getVisionPose(); }

        return DriveSubsystem.X.getOdometricPose();
    }

    public void zeroAngle() {
        this.gyro.setAngleAdjustment(-this.getAngle().getDegrees());
    }

    /**
     * @return the orietation of the robot
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
        return null; //TODO(vision team): implement #getVisionPose()
    }

    /**
     * @return whether or not a viable vision-based pose is availible
     */
    public boolean hasVisionPose() {
        return false; //TODO(vision team): implement #hasVisionPose()
    }
}
