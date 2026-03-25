package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.*;

public class Autonomous {

    public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    
    public static void initialize(Robot bot) {

        try {
        
            AutoBuilder.configure(
                DriveSubsystem.X::getOdometricPose,
                DriveSubsystem.X::resetOdometricPose,
                DriveSubsystem.X::getSpeeds,
                DriveSubsystem.X::drive,
                new PPHolonomicDriveController(new PIDConstants(5), new PIDConstants(5)),
                RobotConfig.fromGUISettings(),
                () -> DriverStation.getAlliance().get() == Alliance.Red,
                DriveSubsystem.X
            );
        }
        catch (Exception e) {
            e.printStackTrace();
        }

        NamedCommands.registerCommand("intake.idleup"  , IntakeCommand.IDLE_UP  .get());
        NamedCommands.registerCommand("intake.idledown", IntakeCommand.IDLE_DOWN.get());
        NamedCommands.registerCommand("intake.run"     , IntakeCommand.RUN      .get());

        NamedCommands.registerCommand("shooter.shoot", ShooterCommand.SHOOT.get());
        NamedCommands.registerCommand("shooter.prep" , ShooterCommand.PREP .get());

        SmartDashboard.putData("Auto:", autoChooser = AutoBuilder.buildAutoChooser());
    }
}