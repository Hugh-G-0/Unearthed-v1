package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.*;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex driveMC = new SparkFlex(20, MotorType.kBrushless);
    private final SparkClosedLoopController drivePIDF = this.driveMC.getClosedLoopController();

    private final SparkMax liftMC = new SparkMax(26, MotorType.kBrushless);
    private final SparkClosedLoopController liftPIDF = liftMC.getClosedLoopController();
    private final SparkAbsoluteEncoder liftEnc = liftMC.getAbsoluteEncoder();

    public static final IntakeSubsystem X = new IntakeSubsystem();
    
    private IntakeSubsystem() {

        this.driveMC.configure(new SparkFlexConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.liftMC.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //this.setDefaultCommand(IntakeCommand.IDLE_DOWN.get());
    }

    public void run(boolean isDown, double v) {

        this.drivePIDF.setSetpoint(v, ControlType.kVoltage);

        if (isDown && liftEnc.getPosition() < 0.8) {
            this.liftPIDF.setSetpoint(DOWN_VOLTS, ControlType.kVoltage);
        }
        else if (!isDown && liftEnc.getPosition() > 0.5) {
            this.liftPIDF.setSetpoint(UP_VOLTS, ControlType.kVoltage);
        }
        else {
            this.liftPIDF.setSetpoint(0, ControlType.kVoltage);
        }
    }

    private static final double UP_VOLTS = -2, DOWN_VOLTS = 3;
}
