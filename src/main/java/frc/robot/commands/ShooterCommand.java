package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command{

    private double s, f, b;

    public static final double
        SHOOT_VOLT = 8,
        FEED_VOLT  = 7,
        BELT_VOLT = 2
    ;

    public static final Supplier<ShooterCommand>
        SHOOT = () -> new ShooterCommand(SHOOT_VOLT, FEED_VOLT, BELT_VOLT),
        PREP  = () -> new ShooterCommand(SHOOT_VOLT, 0, 0),
        IDLE  = () -> new ShooterCommand(0, 0, 0)
    ;

    private ShooterCommand(double s, double f, double b) {
        this.s = s;
        this.f = f;
        this.b = b;

        this.addRequirements(ShooterSubsystem.X);
    }

    @Override
    public void initialize() {
        this.addRequirements(ShooterSubsystem.X);
    }

    @Override
    public void execute() {
        ShooterSubsystem.X.run(this.s, this.f, this.b);
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean wasInterrupted) {
        ShooterSubsystem.X.run(0, 0, 0);
    }
    
}
