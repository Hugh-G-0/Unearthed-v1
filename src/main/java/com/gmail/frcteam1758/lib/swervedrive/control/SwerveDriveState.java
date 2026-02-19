package com.gmail.frcteam1758.lib.swervedrive.control;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * a {@link ChassisSpeeds but with the option to enter "locked"/"X" configuration}
 */
public class SwerveDriveState {
    
    public ChassisSpeeds speeds;

    public boolean lock;

    public static final SwerveDriveState LOCKED = new SwerveDriveState();

    public SwerveDriveState(ChassisSpeeds p_speeds) {
        
        speeds = p_speeds;

        lock = false;
    }

    public SwerveDriveState() {

        speeds = new ChassisSpeeds();

        lock = true;
    }
}
