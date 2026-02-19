package frc.robot.subsystems;

import com.gmail.frcteam1758.lib.enums.SwerveDriveMode;
import com.gmail.frcteam1758.lib.swervedrive.SwerveChassis;
import com.gmail.frcteam1758.lib.swervedrive.control.SwerveDriveControls2023;
import com.gmail.frcteam1758.lib.swervedrive.control.SwerveDriveInput;
import com.gmail.frcteam1758.lib.swervedrive.control.SwerveDriveState;
import com.gmail.frcteam1758.lib.swervedrive.vortex.VortexSwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

    private final VortexSwerveModule[] modules = {

        new VortexSwerveModule(
            1, 5,
            new Translation2d( Constants.kWheelBaseX,  Constants.kWheelBaseY)
        ),
        new VortexSwerveModule(
            2, 6,
            new Translation2d(-Constants.kWheelBaseX,  Constants.kWheelBaseY)
        ),
        new VortexSwerveModule(
            3, 7,
            new Translation2d(-Constants.kWheelBaseX, -Constants.kWheelBaseY)
        ),
        new VortexSwerveModule(
            4, 8,
            new Translation2d( Constants.kWheelBaseX, -Constants.kWheelBaseY)
        )
    };

    private final SwerveDriveInput controls = new SwerveDriveControls2023(
        0,
        1,
        NavSubsystem.X::getAngle,
        SwerveDriveMode.FIELD_ORIENTED,
        4.8,
        4.0
    );

    private final SwerveChassis chassis = new SwerveChassis(
        controls,
        SwerveDriveInput.NO_INPUT,
        modules, 4.0, NavSubsystem.X::getAngle);

    public static final DriveSubsystem X = new DriveSubsystem();

    private DriveSubsystem() {}

    @Override
    public void periodic() {

        this.chassis.getPose();
        
        if (NavSubsystem.X.hasVisionPose()) { this.chassis.resetPose(NavSubsystem.X.getPose()); }

        SmartDashboard.putNumber("chassisSpeedX", this.controls.getCommandedState().speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("chassisSpeedY", this.controls.getCommandedState().speeds.vyMetersPerSecond);

        SmartDashboard.putNumber("chassisSpeedR (rad/s)", this.controls.getCommandedState().speeds.omegaRadiansPerSecond);
    }

    /**
     * causes all four swerve module to do as the controller indicates (teleop)
     */
    public void performTeleop() { this.chassis.performTeleop(); }

    /**
     * causes all four modules to do as the provided ChassisSpeeds indicates
     * 
     * mainly useful for pathplanner
     * @param x the chassisspeeds
     */
    public void acceptChassisSpeeds(ChassisSpeeds x) {

        this.chassis.run(new SwerveDriveState(x));
    }

    public ChassisSpeeds getSpeeds() {
        return this.chassis.getCurrentSpeeds();
    }

    public void resetOdometricPose(Pose2d pos) {

        this.chassis.resetPose(pos);
    }

    public Pose2d getOdometricPose() {
        return this.chassis.getPose();
    }

    public void reConfig() {

        for (var m: this.modules) {m.reConfig();}
    }
    
}
