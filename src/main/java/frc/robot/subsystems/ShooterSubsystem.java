package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.*;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex
        shootMCL = new SparkFlex(23, MotorType.kBrushless),
        shootMCR = new SparkFlex(24, MotorType.kBrushless)
    ;
    private final SparkMax
        feedMCL = new SparkMax (21, MotorType.kBrushless),
        feedMCR = new SparkMax(22, MotorType.kBrushless),
        beltMC  = new SparkMax(25, MotorType.kBrushless)
    ;

    private final SparkClosedLoopController shootPidfL = this.shootMCL.getClosedLoopController();
    private final SparkClosedLoopController shootPidfR = this.shootMCR.getClosedLoopController();

    private final SparkClosedLoopController feedPidfL  = this.feedMCL .getClosedLoopController();
    private final SparkClosedLoopController feedPidfR  = this.feedMCR .getClosedLoopController();
    private final SparkClosedLoopController beltPidf   = this.beltMC  .getClosedLoopController();

    public static final ShooterSubsystem X = new ShooterSubsystem();

    private ShooterSubsystem() {

        SparkBaseConfig flexCfg = new SparkFlexConfig();
        SparkBaseConfig maxCFG  = new SparkMaxConfig ();

        shootMCL.configure(flexCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shootMCR.configure(flexCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        feedMCL.configure(maxCFG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        feedMCR.configure(maxCFG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        beltMC .configure(maxCFG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //this.setDefaultCommand(ShooterCommand.IDLE.get());
    }
    
    public void run(double shootVolts, double feedVolts, double beltVolts) {

        this.shootPidfL.setSetpoint(-shootVolts, ControlType.kVoltage);
        this.shootPidfR.setSetpoint( shootVolts, ControlType.kVoltage);

        this.feedPidfL.setSetpoint(-beltVolts, ControlType.kVoltage);
        this.feedPidfR.setSetpoint( beltVolts, ControlType.kVoltage);

        this.beltPidf.setSetpoint(beltVolts, ControlType.kVoltage);
    }
}
