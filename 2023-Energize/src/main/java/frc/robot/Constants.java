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
        public static final double maxSpeed = 4.5; // meters per second (Max for robot is 4.5)
        public static final double maxTeleopSpeed = 7.5; // meters per second (Max for robot is 4.5)
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 7;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(274.69);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 0;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(73.56);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(314.56);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
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

        public static final double kPXController = 0;
        public static final double kPYController = 0;
        public static final double kPThetaController = 0;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static final class IntakeConstants {

        public static final double shoulderMaxGravFF = 0.0;
        public static final double wristMaxGravFF = 0.0;

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

        // public static final double highArmAngle = 3.0;
        // public static final double defaultArmAngle = 0.0; 
        // public static final double lowArmangle = 1.0; 
        // public static final double mediumArmAngle = 2.0;
        // public static final double shelfArmAngle = 4.0;
        // public static final double highWristAngle = 3.0;
        // public static final double defaultWristAngle = 0.0; 
        // public static final double lowWristAngle = 1.0; 
        // public static final double mediumWristAngle = 2.0;
        // public static final double shelfWristAngle = 4.0;

        public static final double intakeSpeedPercent = 0.1;
        public static final double ejectSpeedPercent = -1;

        public static final double wristMoveSpeedPercentage = 0.25;
        public static final double shoulderMoveSpeedPercentage = 0.15;

        public static final double shoulderCanCoderOffset = 13.79;
        public static final double wristCanCoderOffset = -5.97;

        public static final int shoulderUpperLimit = 10;
        public static final int shoulderLowerLimit = -86;

        public static final int wristUpperLimit = 136;
        public static final int wristLowerLimit = -62;

        // IshowSpeeds

        public static final double speed = 0.2;

        public static final int canPause = 100;
    }

    public static final class IntakeAngles {

        public static final double stowedWristAngle = 0.0;
        public static final double stowedShoulderAngle = 0.0;

        public static final double pickUpWristAngle = 0.0;
        public static final double pickUpShoulderAngle = 0.0;

        public static final double lowWristAngle = 0.0;
        public static final double lowShoulderAngle = 0.0;

        public static final double midWristAngle = 0.0;
        public static final double midShoulderAngle = 0.0;

        public static final double highWristAngle = 0.0;
        public static final double highShoulderAngle = 0.0;

        public static final double shelfWristAngle = 0.0;
        public static final double shelfShoulderAngle = 0.0;

    }

    public static final class BalanceConstants {
        public static final double KI = 0;
        public static final double KP = .016;
        public static final double KD = 0;
    }

    public static final class PathFactory {
        // NOTE: All measurements here are in inches. TODO: make it not inches.

        public static final double p1 = 20.19;
        public static final double p2 = 42.19;
        public static final double p3 = 64.19;
        public static final double p4 = 86.19;
        public static final double p5 = 108.19;
        public static final double p6 = 130.19;
        public static final double p7 = 157.19;
        public static final double p8 = 174.19;
        public static final double p9 = 196.19;

        public static final double a = 180.19;
        public static final double b = 36.19;

        public static final double c1 = 29.695;
        public static final double c2 = 186.335;

        public static final class redSide {

            public static final double x = 610.77;

            public static final double abx = 372.92;

            public static final double cx = 438.07;

        }
        public static final class blueSide {
            public static final double x = 40.45;
            
            public static final double abx = 278.25;

            public static final double cx = 132.25;
            
        }
    }
}
