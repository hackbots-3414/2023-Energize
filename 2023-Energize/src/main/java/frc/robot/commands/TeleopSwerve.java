package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends CommandBase {
    final static Logger logger = LoggerFactory.getLogger(TeleopSwerve.class);
    final static SlewRateLimiter filter = new SlewRateLimiter(0.5);

    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private double multiplier = 1;
    private boolean isReduced;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, boolean isReduced) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.isReduced = isReduced;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        //SmartDashboard.putNumber("Speed Percentage", 1);
    }

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {

        this(s_Swerve, translationSup, strafeSup, rotationSup, robotCentricSup, false);

    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        
        /* Drive */
        if (isReduced) {

            s_Swerve.drive(
                    new Translation2d(translationVal, strafeVal)
                            .times(Constants.Swerve.reducedSpeed * SmartDashboard.getNumber("Speed Percentage", 1)),
                    rotationVal * Constants.Swerve.reducedAngVel,
                    !robotCentricSup.getAsBoolean(),
                    true);

        } else {
            s_Swerve.drive(
                    new Translation2d(translationVal, strafeVal)
                            .times(Constants.Swerve.maxSpeed * SmartDashboard.getNumber("Speed Percentage", 1)),
                    rotationVal * Constants.Swerve.maxAngularVelocity,
                    !robotCentricSup.getAsBoolean(),
                    true);
        }
    }
}
