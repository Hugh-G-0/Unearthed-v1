package com.gmail.frcteam1758.lib.swervedrive.vortex;

import com.gmail.frcteam1758.lib.swervedrive.MaxSwerveConstants;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * default configuration for Vortex-based MaxSwerveModules ({@link VortexSwerveModule}s)
 * 
 * you should not need to edit this file
 */
public class VortexSwerveDefaults {

    // prevent creation of useless instances
    private VortexSwerveDefaults() {}

    public static final SparkFlexConfig kDriveCfg = new SparkFlexConfig();
    public static final SparkMaxConfig  kSteerCfg = new SparkMaxConfig ();

    private static final double kDrivingCoefficient =
        MaxSwerveConstants.ModuleConstants.kWheelDiameterMeters * Math.PI
        /
        MaxSwerveConstants.ModuleConstants.kDrivingMotorReduction
    ;

    private static final double kSteeringCoefficient = 2 * Math.PI;

    private static final double kDrivingFFV = 12.3 / MaxSwerveConstants.ModuleConstants.kVortexDriveWheelFreeSpeedRps;

    private static boolean isReady = false;

    // apply configs
    public static void prepareConfigs() {

        if (isReady) return;

        kDriveCfg
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
        ;

        kDriveCfg.encoder
            .positionConversionFactor(kDrivingCoefficient)
            .velocityConversionFactor(kDrivingCoefficient / 60)
        ;

        kDriveCfg.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.04, 0, 0)
            .outputRange(-1, 1)
        ;

        kDriveCfg.closedLoop.feedForward.kV(kDrivingFFV);

        kSteerCfg
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)
        ;

        kSteerCfg.absoluteEncoder
            .inverted(true)
            .positionConversionFactor(kSteeringCoefficient)
            .velocityConversionFactor(kSteeringCoefficient / 60)
        ;

        kSteerCfg.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(1,0,0)
            .outputRange(-1, 1)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, kSteeringCoefficient)
        ;
    }
}