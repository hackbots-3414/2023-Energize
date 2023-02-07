package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public final class Constants {
    final static Logger logger = LoggerFactory.getLogger(Constants.class);
    public static final double stickDeadband = 0.15;

    public static final class Swerve {
        public static final int pigeonID = 25;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
        public static final String canbusString = "CANivore";

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(25.86);
        public static final double wheelBase = Units.inchesToMeters(26.125);
        public static final double wheelDiameter = Units.inchesToMeters(4.42);
        public static final double wheelCircumference = wheelDiameter * Math.PI;
        public static final double ticksPerRevolution = 2048;
        public static final double driveGearRatio = (8.14 / 1.0); // https://www.swervedrivespecialties.com/products/mk4i-swerve-module
        public static final double inchesPerTick = 4.42 * Math.PI / 2048;

        public static final double distanceToTicks = wheelDiameter * Math.PI/ticksPerRevolution/driveGearRatio;
        public static final double tickstoDistance = 1.0 / (wheelDiameter * Math.PI/ticksPerRevolution/driveGearRatio);

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double angleGearRatio = (150/7/1);

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.55;
        public static final double angleKI = 0.0;
        public static final double angleKD = 12;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.11237;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.10933 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.4855 / 12);
        public static final double driveKA = (0.21837 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //meters per second (Max for robot is 4.5)
        public static final double maxTeleopSpeed = 7.5; //meters per second (Max for robot is 4.5)
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 0;
            public static final int canCoderID = 11;
            public static final double angleOffset = 309.81;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 40;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 10;
            public static final double angleOffset = 199.42;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 16;
            public static final int angleMotorID = 17;
            public static final int canCoderID = 8;
            public static final double angleOffset = 331.61;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 19;
            public static final int angleMotorID = 18;
            public static final int canCoderID = 9;
            public static final double angleOffset = 80.15;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static final class IntakeConstants {
        public static final int handMotorID = 10;
        public static final int wristMotorID = 7;
        public static final int shoulderMotorID = 6;


        public static final double highArmAngle = 3.0;
        public static final double defaultArmAngle = 0.0; 
        public static final double lowArmangle = 1.0; 
        public static final double mediumArmAngle = 2.0;
        // public static final double shelfArmAngle = 4.0; 
        public static final double highWristAngle = 3.0;
        public static final double defaultWristAngle = 0.0; 
        public static final double lowWristAngle = 1.0; 
        public static final double mediumWristAngle = 2.0;
        // public static final double shelfWristAngle = 4.0;


        

        public static final int wristCanCoderID = 0;
        public static final int ShoulderCanCoderID = 0;

        public static final double intakeSpeedPercent = 0.1;
        public static final double ejectSpeedPercent = -1;

        public static final int wristRotationTarget = 4000;
        public static final int shoulderRotationTarget = 4000;
        public static final int shoulderUpperLimit = 1000;
        public static final int shoulderLowerLimit = 1000;
        public static final int wristUpperLimit = 1000;
        public static final int wristLowerLimit = 1000;

        // IshowSpeeds 

        public static final double speed = 0.2;




        public static final int canPause = 100;
    }

}
