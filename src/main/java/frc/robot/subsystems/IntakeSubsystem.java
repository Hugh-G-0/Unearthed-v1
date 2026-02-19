package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex driveMC = new SparkFlex(-1, MotorType.kBrushless);

    private final SparkClosedLoopController drivePIDF = this.driveMC.getClosedLoopController();

    public static final IntakeSubsystem X = null;//new IntakeSubsystem();
    
    private IntakeSubsystem() {

        SparkBaseConfig c = new SparkFlexConfig();

        c
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
        ;

        c.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0, 0, 0)
            .outputRange(-1, 1)
        ;

        c.closedLoop.feedForward.sva(0, 0, 0);

        this.driveMC.configure(c, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void run(boolean isDown, double v) {

        this.drivePIDF.setSetpoint(v, ControlType.kVelocity);
    }
}
