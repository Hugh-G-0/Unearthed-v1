package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavSubsystem extends SubsystemBase {

    public static final NavSubsystem X = new NavSubsystem();
    
    private NavSubsystem() {}

    public Rotation2d getAngle() {

        return new Rotation2d(); //TODO:actual gyro stuff
    }
}
