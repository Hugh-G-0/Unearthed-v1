package frc.robot.subsystems.visiondeps;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {

    public static final Transform3d ROBOT_TO_CAM = new Transform3d(
        new Translation3d(0.3, 0.0, 0.5),
        new Rotation3d(0, Math.toRadians(-15), 0)
    );

    public static final String CAMERA_NAME = "Arducam_OV9782_USB_Camera";
}
