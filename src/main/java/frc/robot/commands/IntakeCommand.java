package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

    private final boolean d;
    private final double  v;

    public static final double RUN_VOLT = 4;

    public static final Supplier<IntakeCommand>
        RUN       = () -> new IntakeCommand(true , RUN_VOLT),
        IDLE_DOWN = () -> new IntakeCommand(true , 0     ),
        IDLE_UP   = () -> new IntakeCommand(false, 0     )
    ;

    private IntakeCommand(boolean d, double v) {
        this.d = d;
        this.v = v;

        this.addRequirements(IntakeSubsystem.X);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        IntakeSubsystem.X.run(this.d, this.v);
    }

    @Override
    public boolean isFinished() { return false; }
    
    @Override
    public void end(boolean wasInterupted) {
        IntakeSubsystem.X.run(this.d, 0);
    }
}
