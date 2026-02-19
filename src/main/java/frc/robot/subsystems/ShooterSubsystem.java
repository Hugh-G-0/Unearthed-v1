package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex shootMC = new SparkFlex(-1, MotorType.kBrushless);
    private final SparkMax  feedMC  = new SparkMax (-1, MotorType.kBrushless);

    private final SparkClosedLoopController shootPIDF = this.shootMC.getClosedLoopController();
    private final SparkClosedLoopController feedPIDF  = this.feedMC .getClosedLoopController();

    public static final ShooterSubsystem X = null;//new ShooterSubsystem();

    private ShooterSubsystem() {

        SparkBaseConfig shootCfg = new SparkFlexConfig();

        shootCfg
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(50)
        ;
        shootCfg.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0, 0, 0)
            .outputRange(-1, 1)
        ;
        shootCfg.closedLoop.feedForward.sva(0, 0, 0);

        SparkBaseConfig feedCfg  = new SparkMaxConfig ();
        
        feedCfg
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
        ;
        feedCfg.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0, 0, 0)
            .outputRange(-1, 1)
        ;
        feedCfg.closedLoop.feedForward.sva(0, 0, 0);
        
        this.shootMC.configure(shootCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.feedMC .configure(feedCfg , ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void run(double shootVel, double feedVel) {

        this.shootPIDF.setSetpoint(shootVel, ControlType.kVelocity);
        this.feedPIDF .setSetpoint(feedVel , ControlType.kVelocity);
    }
}
