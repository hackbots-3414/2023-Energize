package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.math.Conversions;

public final class Constants {
    public static final double stickDeadband = 0.06;

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
        public static final double maxSpeed = 4.5; // meters per second (Max for robot is 4.5)
        public static final double maxAngularVelocity = 11.5;
        public static final double reducedAngVel = 6.0; 
        public static final double reducedSpeed = 1.0; 

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        // Toothless drive constants
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 7;
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(260.51);
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(310.95); // LoveLace
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 0;
            public static final int canCoderID = 2;
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(72.15);
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(190.54); // LoveLace
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 12;
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(314.91);
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(330.92); // LoveLace
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 18;
            public static final int angleMotorID = 19;
            public static final int canCoderID = 17;
            // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(70.48);
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(249.96); //Lovelace
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

         /* Spare - Module 4 */
        // public static final class SpareMod { //TODO: This must be tuned to specific robot
        //     public static final int driveMotorID = 30;
        //     public static final int angleMotorID = 31;
        //     public static final int canCoderID = 32;
        //     public static final Rotation2d angleOffset = Rotation2d.fromDegrees(70.48);
        //     //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(249.96); //Lovelace
        //     public static final SwerveModuleConstants constants = 
        //         new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        // }
        
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 1;// 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;//1.05;
        public static final double kMaxBalanceSpeedMetersPerSecond = 1;
        public static final double kMaxBalanceAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 2.0;
        public static final double kPYController = 2.0;
        public static final double kPThetaController = 3.80;

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

        public static final double shoulderGearRatio = 308.34;
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
        public static final double ejectSpeedPercent = -0.4;
        public static final double ejectSpeedAutonPercent = -0.7;
        public static final double intakeSpeedAutonPercent = 0.2;
        public static final double objectHoldSpeedPercent = 0.1;

        public static final double wristMoveSpeedPercentage = 0.25;
        public static final double shoulderMoveSpeedPercentage = 0.15;

        //main bot
        public static final double shoulderCanCoderOffset = 4.21;//14.67; //9.31
        public static final double wristCanCoderOffset = 77.39;//-109.42;  //-53.78

        //woody
        // public static final double shoulderCanCoderOffset = 135.8;
        // public static final double wristCanCoderOffset = 151.87;

        //MAIN BOT
        public static final int handCurrentLimit = 50; // 50
        public static final int secondHandCurrentLimit = 10; // 15
        public static final double handCurrentThreshold = 40;

        public static final int shoulderUpperLimit = 17;
        public static final int shoulderLowerLimit = -86;

        public static final int wristUpperLimit = 6;
        public static final int wristLowerLimit = -145;

        public static final double shoulderkP = 0.03;
        public static final double shoulderkI = 0.0;
        public static final double shoulderkD = 0.0;

        public static final double wristkP = 1.5;
        public static final double wristkI = 0.0;
        public static final double wristkD = 0.0;

        //Volts
        public static final double shoulderkS = 0;//0.05;
        public static final double shoulderkG = 0.9;//0.7458;
        public static final double shoulderkV = 6.0;
        public static final double shoulderkA = 0.6;

        public static final double wristkS = 0.0;
        public static final double wristkG = 0.4;
        public static final double wristkV = 4.5;
        public static final double wristkA = 0.01;

        public static final double shouldermaxVelo = Math.toRadians(180); // degrees / seconds
        public static final double shouldermaxAccel = Math.toRadians(220); // degrees / seconds^2

        public static final double wristmaxVelo = Math.toRadians(190); // degrees / seconds
        public static final double wristmaxAccel = Math.toRadians(190); // degrees / seconds^2

        public static final double speed = 0.2;
        public static final int canPause = 100;
    }

    public static final class IntakeAngles {

        public static final double stowedWristAngle = Math.toRadians(0);
        public static final double stowedShoulderAngle = Math.toRadians(-86.57);

        public static final double pickUpWristAngle = Math.toRadians(-37);  
        public static final double pickUpShoulderAngle = Math.toRadians(-66.882); //-74.882 // -68.25

        // public static final double lowWristAngle = Math.toRadians(35.0);
        // public static final double lowShoulderAngle = Math.toRadians(-85.0);

        public static final double midWristAngle = Math.toRadians(-44);
        public static final double midShoulderAngle = Math.toRadians(-13);

        public static final double highWristAngle = Math.toRadians(-33);
        public static final double highShoulderAngle = Math.toRadians(8.5);

        public static final double shelfWristAngle = Math.toRadians(-71);
        public static final double shelfShoulderAngle = Math.toRadians(15.5);

        public static final double standingConeWristAngle = Math.toRadians(-71);
        public static final double standingConeShoulderAngle = Math.toRadians(-44);

        public static final double knockedConeShoulderAngle = Math.toRadians(-70.57);
        public static final double KnockedConeWristAngle = Math.toRadians(-35); //estimated Wrist Angle might not work

        public static final double uprightConeShoulderAngle = Math.toRadians(-51.70);
        public static final double uprightConeWristAngle = Math.toRadians(-59); //estimated Wrist angle might not work

        public static final double shelfShoulderDown = Math.toRadians(2.0);
    }

    public static final class BalanceConstants {
        public static final double KI = 0;
        public static final double KP = .01; // 0.016
        public static final double KD = 0;
    }

    public static final class RobotContstants {
        public static final double armLength = Conversions.inchesToMeters(45.5);
        public static final double armFromCenterOfRobot = Conversions.inchesToMeters(-1);
        public static final double armAdjustment = armLength + armFromCenterOfRobot;
    }

    public static final class PathFactory {
        public static final double p1 = Conversions.inchesToMeters(20.19);
        public static final double p2 = Conversions.inchesToMeters(42.19);
        public static final double p3 = Conversions.inchesToMeters(64.19);
        public static final double p4 = Conversions.inchesToMeters(86.19);
        public static final double p5 = Conversions.inchesToMeters(108.19);
        public static final double p6 = Conversions.inchesToMeters(130.19);
        public static final double p7 = Conversions.inchesToMeters(157.19);
        public static final double p8 = Conversions.inchesToMeters(174.19);
        public static final double p9 = Conversions.inchesToMeters(196.19);

        public static final double c1 = Conversions.inchesToMeters(29.695);
        public static final double c2 = Conversions.inchesToMeters(186.335);

        public static final class redSide {

            public static final double x = Conversions.inchesToMeters(610.77) - RobotContstants.armAdjustment;

            public static final double cx = Conversions.inchesToMeters(457.64);
            
            public static final double offset = -RobotContstants.armAdjustment; // Negative because the red side is going to need to move left, not right.
        }
        public static final class blueSide {
            // public static final double x = Conversions.inchesToMeters(40.45) + RobotContstants.armAdjustment;
            public static final double x = Conversions.inchesToMeters(50.45) + RobotContstants.armAdjustment;
            
            public static final double cx = Conversions.inchesToMeters(165.06);

            public static final double offset = RobotContstants.armAdjustment;
        }
    }

    public static final class ButtonBoard {
        public static final int buttonBoardPort = 2;
    }

    public static final class IntakeAutomatic {
        public static final double shelfApproachLimit = 80;
        public static final double redSideX = Units.inchesToMeters(596.51 - shelfApproachLimit);
        public static final double blueSideX = Units.inchesToMeters(26.19 + shelfApproachLimit);
        // public static final double redSlowDownX = 15.2707257538;   //601.21 / 39.3701;
        // public static final double blueSlowDownX = 1.63194911875;  //64.25 / 39.3701;
        public static final double slowLimit = 0.2; //TODO: fine-tune slowMultiplier variable.
    }
}
