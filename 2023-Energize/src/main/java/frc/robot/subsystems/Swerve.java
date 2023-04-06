package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix.sensors.Pigeon2;

import org.photonvision.EstimatedRobotPose;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.VisionWrapper;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public Translation2d translation2d;

    public SwerveDrivePoseEstimator poseEstimator;
    public Field2d fieldSim;
    public VisionWrapper visionWrapper;

    private static Logger log = LoggerFactory.getLogger(Swerve.class);
    private int visionError = 0;
    
    private boolean isfieldRelative;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Swerve.canbusString);
        gyro.configFactoryDefault();
        gyro.setYaw(0, Constants.IntakeConstants.canPause);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

        Matrix<N3, N1> robotSD = new Matrix<>(Nat.N3(), Nat.N1());
        robotSD.set(0, 0, 0.1);
        robotSD.set(1, 0, 0.1);
        robotSD.set(2, 0, Math.toRadians(0.5));

        Matrix<N3, N1> visionSD = new Matrix<>(Nat.N3(), Nat.N1());
        visionSD.set(0, 0, 0.01);
        visionSD.set(1, 0, 0.9);
        visionSD.set(2, 0, 0.01);

        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, new Rotation2d(gyro.getYaw()),
                getModulePositions(), new Pose2d(), robotSD, visionSD);
        fieldSim = new Field2d();
        visionWrapper = new VisionWrapper();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        setModuleStates(swerveModuleStates);
        isfieldRelative = fieldRelative;
    }

    public void autonDrive(Translation2d translation, double rotation, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation
            )
        );
        setModuleStates(swerveModuleStates);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }
    

    public void resetOdometry(Pose2d pose) {
        gyro.setYaw(pose.getRotation().getDegrees(), Constants.IntakeConstants.canPause);
        swerveOdometry.resetPosition(pose.getRotation(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0, Constants.IntakeConstants.canPause);
        swerveOdometry.resetPosition(new Rotation2d(0), getModulePositions(), new Pose2d(new Translation2d(getPose().getX(), getPose().getY()), Rotation2d.fromDegrees(0)));
    }

    public void zeroHeading() {
        gyro.setYaw(0, Constants.IntakeConstants.canPause);
        swerveOdometry.resetPosition(new Rotation2d(0), getModulePositions(), new Pose2d(new Translation2d(getPose().getX(), getPose().getY()), Rotation2d.fromDegrees(0)));
    }

    public Rotation2d getYaw() {
        double yaw = gyro.getYaw();
        yaw %= 360;
        yaw = (yaw + 360) % 360;

        if (yaw > 180) {
            yaw -= 360;
        }

        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(yaw * -1) : Rotation2d.fromDegrees(yaw);
    }

    public Rotation2d getHeading() {
        return swerveOdometry.getPoseMeters().getRotation();
    }

    public Rotation2d getPitch() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[1]) : Rotation2d.fromDegrees(ypr[1]);
    }

    public Rotation2d getRoll() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[2]) : Rotation2d.fromDegrees(ypr[2]);
    }

    public double getAverageSensorPositions() {
        double[] positions = new double[4]; // FL FR BL BR
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getSensorPosition();
        }

        return ((positions[0] + positions[2]) / 2D) + ((positions[1] + positions[3]) / 2D) / 2D;
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
        log.warn("Mod positions set to absolute");
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(fieldSim);
        swerveOdometry.update(getYaw(), getModulePositions());
        // updateOdometry();
        translation2d = getPose().getTranslation();
        SmartDashboard.putBoolean("IsFieldRelative", isfieldRelative);
        SmartDashboard.putNumber("gyro", getYaw().getDegrees());
        SmartDashboard.putNumber("Odometry Heading", swerveOdometry.getPoseMeters().getRotation().getDegrees());

        for(SwerveModule mod : mSwerveMods){
        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder",
        mod.getCanCoder().getDegrees());
        // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
        // mod.getPosition().angle.getDegrees());
        // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
        // mod.getState().speedMetersPerSecond);
        }
    }

    public void setX() {
        SwerveModuleState[] states = { new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
                new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
                new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
                new SwerveModuleState(0, new Rotation2d(Math.PI / 4)) };
        setModuleStates(states);
    }

    public boolean isfieldRelative() {
        return isfieldRelative;
    }

    public void updateOdometry() {
        poseEstimator.update(getYaw(), getModulePositions());

        Optional<EstimatedRobotPose> result = visionWrapper.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());

        try {
            if (result.isPresent()) {
                poseEstimator.setVisionMeasurementStdDevs(visionWrapper.getStandardD());
                EstimatedRobotPose camPose = result.get();
                Pose2d robotLocation = camPose.estimatedPose.toPose2d();



                // if (Math.abs(getPose().getTranslation().getDistance(robotLocation.getTranslation())) < 1) {
                    poseEstimator.addVisionMeasurement(
                    robotLocation,
                    camPose.timestampSeconds);
                    fieldSim.getObject("Cam Est Pos").setPose(camPose.estimatedPose.toPose2d());
                // }
                
                
            } /* else {
                fieldSim.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
            }*/

            fieldSim.getObject("Actual Pos").setPose(getPose());
            fieldSim.setRobotPose(poseEstimator.getEstimatedPosition());

            resetOdometry(poseEstimator.getEstimatedPosition());
        } catch (Exception e) {
            visionError++;
            if (visionError % 1000 == 0) {
                log.error("Beelink exception caught: " + e.toString());
            }

        }

    }

    public void driveForward(double distancex, double distancey) {
        Translation2d targetTranslation = new Translation2d(
            distancex,
            distancey
        );
        drive(targetTranslation, 0, false, false);
    }

    public void stopDriving() {
        drive(new Translation2d(), 0, false, false);
    
    }

}