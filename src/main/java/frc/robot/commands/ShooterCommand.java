package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command{

    private double s, f;

    public static final double
        SHOOT_VEL = 0,
        FEED_VEL  = 0
    ;

    public static final ShooterCommand
        SHOOT = new ShooterCommand(SHOOT_VEL, FEED_VEL),
        PREP  = new ShooterCommand(SHOOT_VEL, 0       ),
        IDLE  = new ShooterCommand(0        , 0       )
    ;

    private ShooterCommand(double s, double f) {
        this.s = s;
        this.f = f;
    }

    @Override
    public void initialize() {
        this.addRequirements(ShooterSubsystem.X);
    }

    @Override
    public void execute() {
        ShooterSubsystem.X.run(this.s, this.f);
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean wasInterrupted) {
        ShooterSubsystem.X.run(0, 0);
    }
    
}
