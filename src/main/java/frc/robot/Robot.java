// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavSubsystem;

public class Robot extends TimedRobot {

    public Robot() {
        DriveSubsystem.X.register();
        NavSubsystem  .X.register();

        NavSubsystem.X.zeroAngle();
    }

    @Override
    public void robotInit() {
        DriveSubsystem.X.reConfig();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
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
