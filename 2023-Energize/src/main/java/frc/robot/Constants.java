package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 25;
        public static final int pdhID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
        public static final String canbusString = "CANivore";

        public static final COTSFalconSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
                COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(25.86); // TODO: This must be tuned to specific
                                                                             // robot
        public static final double wheelBase = Units.inchesToMeters(26.125); // TODO: This must be tuned to specific
                                                                             // robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        public static final double inchesPerTick = 4.42 * Math.PI / 2048;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        public static final double distanceToTicks = chosenModule.wheelDiameter * Math.PI / 2048 / driveGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.0075;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.05;

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        public static final double driveKS = (0.10933 / 12); // divide by 12 to convert from volts to percent output for
                                                             // CTRE
        public static final double driveKV = (2.7472 / 12);
        public static final double driveKA = (0.24136 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 1; // meters per second (Max for robot is 4.5)
        public static final double maxTeleopSpeed = 7.5; // meters per second (Max for robot is 4.5)
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 7;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(274.69);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 0;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(73.56);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(314.56);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 18;
            public static final int angleMotorID = 19;
            public static final int canCoderID = 17;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(70.04);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 2.0;
        public static final double kPYController = 2.0;
        public static final double  kPThetaController = 1.0;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static final class IntakeConstants {

        public static final double degreesToCancoder = 4096.0 / 360.0;

        public static final double shoulderGearRatio = 308.33;
        public static final double wristGearRatio = 200.0;

        public static final int handMotorID = 4;
        public static final int wristMotorID = 7;
        public static final int shoulderMotorID = 6;

        public static final int wristCanCoderID = 9;
        public static final int ShoulderCanCoderID = 8;

        public static final boolean shoulderCanCoderInvert = true;
        public static final boolean wristCanCoderInvert = false;
        public static final boolean handMotorInvert = true;

        public static final double intakeSpeedPercent = 1;
        public static final double ejectSpeedPercent = -0.6;

        public static final double wristMoveSpeedPercentage = 0.25;
        public static final double shoulderMoveSpeedPercentage = 0.15;

        //main bot
        public static final double shoulderCanCoderOffset = 4.57;
        public static final double wristCanCoderOffset = -77.60;

        //woody
        // public static final double shoulderCanCoderOffset = -54.84;
        // public static final double wristCanCoderOffset = 151.87;

        //MAIN BOT
        public static final int handCurrentLimit = 22;
        public static final int secondHandCurrentLimit = 8;
        public static final double handCurrentThreshold = 20;

        public static final int shoulderUpperLimit = 0;
        public static final int shoulderLowerLimit = -85;

        public static final int wristUpperLimit = 115;
        public static final int wristLowerLimit = -62;

        public static final double shoulderkP = 0.2;
        public static final double shoulderkI = 0.0;
        public static final double shoulderkD = 0.0;

        public static final double wristkP = 0.1;
        public static final double wristkI = 0.0;
        public static final double wristkD = 0.0;

        //Volts
        public static final double shoulderkS = 0.1;
        public static final double shoulderkG = 0.73;
        public static final double shoulderkV = 5.54;
        public static final double shoulderkA = 0.6;

        public static final double wristkS = 0.0;
        public static final double wristkG = 0.07;
        public static final double wristkV = 4.50;
        public static final double wristkA = 0.01;

        public static final double shouldermaxVelo = Math.toRadians(180); // degrees / seconds
        public static final double shouldermaxAccel = Math.toRadians(220); // degrees / seconds^2

        public static final double wristmaxVelo = Math.toRadians(190); // degrees / seconds
        public static final double wristmaxAccel = Math.toRadians(190); // degrees / seconds^2

        public static final double speed = 0.2;
        public static final int canPause = 100;
    }

    public static final class IntakeAngles {

        public static final double stowedWristAngle = Math.toRadians(35.0);
        public static final double stowedShoulderAngle = Math.toRadians(-85.0);

        public static final double pickUpWristAngle = Math.toRadians(0.0);
        public static final double pickUpShoulderAngle = Math.toRadians(-77.25);

        public static final double lowWristAngle = Math.toRadians(35.0);
        public static final double lowShoulderAngle = Math.toRadians(-85.0);

        public static final double midWristAngle = Math.toRadians(38.33);
        public static final double midShoulderAngle = Math.toRadians(-21.0);

        public static final double highWristAngle = Math.toRadians(40.0);
        public static final double highShoulderAngle = Math.toRadians(-4.0);

        public static final double shelfWristAngle = Math.toRadians(0.0);
        public static final double shelfShoulderAngle = Math.toRadians(-7.0);

    }

    public static final class BalanceConstants {
        public static final double KI = 0;
        public static final double KP = .016;
        public static final double KD = 0;
    }
}
