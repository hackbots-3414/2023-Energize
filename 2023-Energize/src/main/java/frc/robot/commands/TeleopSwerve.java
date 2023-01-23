package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        double yAxis = -OI.getLeftVerticalRaw();
        double xAxis = -OI.getLeftLateralRaw();
        double rAxis = -OI.getRightLateralRaw();
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;

        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxTeleopSpeed);

        //System.out.println(translation);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);

        //Rotation in omega radians
        //
    }
}
