package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDriveCommand extends Command {

    public static final TeleopDriveCommand X = new TeleopDriveCommand();

    private TeleopDriveCommand() {}
    
    @Override
    public void execute() {
        DriveSubsystem.X.performTeleop();
    }

    @Override
    public boolean isFinished() { return false; }
}
