package frc.robot;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class CTREConfigs {
    final static Logger logger = LoggerFactory.getLogger(CTREConfigs.class);
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;
    public CANcoderConfiguration shoulderCanCoderConfig;
    public CANcoderConfiguration wristCanCoderConfig;


    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        CANcoderConfiguration
        swerveCanCoderConfig = new CANcoderConfiguration();

        /* Swerve Angle Motor Configurations */
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();

        angleSupplyLimit.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        angleSupplyLimit.SupplyCurrentThreshold = Constants.Swerve.anglePeakCurrentLimit;
        angleSupplyLimit.SupplyTimeThreshold = Constants.Swerve.anglePeakCurrentDuration;
        angleSupplyLimit.SupplyCurrentLimit = Constants.Swerve.angleContinuousCurrentLimit;

        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;
        swerveAngleFXConfig.Slot0.kV = Constants.Swerve.angleKF;
         
        // swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero; ********CHECK THIS*********


        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();

        driveSupplyLimit.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        driveSupplyLimit.SupplyCurrentThreshold = Constants.Swerve.drivePeakCurrentLimit;
        driveSupplyLimit.SupplyTimeThreshold = Constants.Swerve.drivePeakCurrentDuration;
        driveSupplyLimit.SupplyCurrentLimit = Constants.Swerve.driveContinuousCurrentLimit;

        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.Slot0.kV = Constants.Swerve.driveKF;

        swerveDriveFXConfig.CurrentLimits = driveSupplyLimit;

        swerveDriveFXConfig.OpenLoopRamps = new OpenLoopRampsConfigs();
        swerveDriveFXConfig.OpenLoopRamps.withVoltageOpenLoopRampPeriod(Constants.Swerve.openLoopRamp);

        swerveDriveFXConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs();
        swerveDriveFXConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(Constants.Swerve.closedLoopRamp);

        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        // swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

    }

}