// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NavSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Robot extends TimedRobot {

    @SuppressWarnings("unused")
    private final Joystick
        lctrl = new Joystick(0),
        rctrl = new Joystick(1)
    ;

    public Robot() {
        DriveSubsystem.X.register();
        ShooterSubsystem.X.register();
        IntakeSubsystem.X.register();
        NavSubsystem  .X.register();

        NavSubsystem.X.zeroAngle();
    }

    @Override
    public void robotInit() {
        DriveSubsystem.X.reConfig();

        Autonomous.initialize(this);
    }

    @Override
    public void robotPeriodic() {

        if (lctrl.getTriggerPressed()) {
            ShooterCommand.SHOOT.get().schedule();
        }
        else if (lctrl.getRawButtonPressed(2)) {
            ShooterCommand.PREP.get().schedule();
        }
        else if (lctrl.getTriggerReleased()) {
            ShooterCommand.IDLE.get().schedule();
        }
        
        if (rctrl.getTriggerPressed()) {
            IntakeCommand.RUN.get().schedule();
        }
        else if (rctrl.getTriggerReleased()) {
            IntakeCommand.IDLE_DOWN.get().schedule();
        }
        else if (rctrl.getRawButtonPressed(3)) {
            IntakeCommand.IDLE_UP.get().schedule();
        }

        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        Autonomous.autoChooser.getSelected().schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();

        TeleopDriveCommand.X.schedule();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        TeleopDriveCommand.X.cancel();
    }

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
