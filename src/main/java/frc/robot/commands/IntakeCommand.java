package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

    private final boolean d;
    private final double  v;

    public static final double RUN_VEL = 0;

    public static final IntakeCommand
        RUN       = new IntakeCommand(true , RUN_VEL),
        IDLE_DOWN = new IntakeCommand(true , 0    ),
        IDLE_UP   = new IntakeCommand(false, 0    )
    ;

    private IntakeCommand(boolean d, double v) {
        this.d = d;
        this.v = v;
    }

    @Override
    public void initialize() {
        this.addRequirements(IntakeSubsystem.X);
    }

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
