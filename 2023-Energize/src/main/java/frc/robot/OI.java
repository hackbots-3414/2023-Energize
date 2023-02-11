package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


public class OI {
    final static Logger logger = LoggerFactory.getLogger(OI.class);

    private static Joystick joystick = new Joystick(0);
    private static Joystick xbox = new Joystick(1);


    static double left_x_offset = -0.13379;
    static double left_x_max = 0.81982;
    static double left_y_offset = 0.01758;
    static double left_y_max = 0.62793;
    static double right_x_offset = 0.03418;
    static double right_x_max = 0.80713;
    static double right_y_offset = 0.03418;
    static double right_y_max = 0.85005;
    static double r_knob_offset = 0.03371;


    

    // ConfigureReverseControls normalDriveButton = new ConfigureReverseControls(drivetrainSubsystem);

    /*static {
        SmartDashboard.putBoolean("Controller (false = dev, true = comp)", true);
        JoystickButton reverseControlsButton = new JoystickButton(joystick, 12);
        reverseControlsButton.whileHeld(new ConfigureReverseControls(RobotContainer.getInstance().m_drivetrain, true));
        reverseControlsButton.whenReleased(new ConfigureReverseControls(RobotContainer.getInstance().m_drivetrain, false));
    }*/

    private static void updateController() {
        if (SmartDashboard.getBoolean("Controller (false = dev, true = comp)", true)) {
            left_x_offset = -0.05518;
            left_x_max = 0.83401;

            left_y_offset = -0.01953;
            left_y_max = 0.64453;

            right_x_offset = 0.03711;
            right_x_max = 0.73144;

            right_y_offset = 0.01367;
            right_y_max = 0.87256;

            r_knob_offset = 0.03371;
        }
        else {
            left_x_offset = -0.13379;
            left_x_max = 0.81982;

            left_y_offset = 0.01758;
            left_y_max = 0.62793;

            right_x_offset = 0.03418;
            right_x_max = 0.80713;

            right_y_offset = 0.03418;
            right_y_max = 0.85005;

            r_knob_offset = 0.03613;
        }
    }

    public static double getLeftLateral() {
        updateController();
        return (joystick.getRawAxis(0) - left_x_offset) / left_x_max;
    }

    public static double getLeftVertical() {
        updateController();
        return (joystick.getRawAxis(1) - left_y_offset) / left_y_max;
    }

    public static double getRightLateral() {
        updateController();
        return (joystick.getRawAxis(3) - right_x_offset) / right_x_max;
    }

    public static double getRightVertical() {
        updateController();
        return (joystick.getRawAxis(4) - right_y_offset) / right_y_max;
    }

    public static double getLeftLateralRaw() {
        updateController();
        return joystick.getRawAxis(0);
    }

    public static double getLeftVerticalRaw() {
        updateController();
        return joystick.getRawAxis(1);
    }

    public static double getRightLateralRaw() {
        updateController();
        return joystick.getRawAxis(3);
    }

    public static double getRightVerticalRaw() {
        updateController();
        return joystick.getRawAxis(4);
    }

    public static double getRKnob() {
        updateController();
        return (joystick.getRawAxis(6) + r_knob_offset + 1) / 2;
    }

    public static double getRKnobRaw() {
        updateController();
        return joystick.getRawAxis(6);
    }



    public static boolean getButtonH() {
        updateController();
        return joystick.getRawButton(12);
    }

    public static boolean getButtonB() {
        updateController();
        return xbox.getRawButton(2);
    }

    public static boolean getButtonA() {
        updateController();
        return xbox.getRawButton(1);
    }

    public static boolean getButtonY(){
        updateController();
        return xbox.getRawButton(4);
    }

    public static boolean getButtonX(){
        updateController();
        return xbox.getRawButton(3);
    }

    public static double getButtonLeftTrigger(){
        updateController();
        return xbox.getRawAxis(2);
    }

    public static double getButtonRightTrigger(){
        updateController();
        return xbox.getRawAxis(3);
    }

    public static boolean getButtonLeftBumper(){
        updateController();
        return xbox.getRawButton(5);
    }

    public static boolean getButtonRightBumper(){
        updateController();
        return xbox.getRawButton(6);
    }

    public static double getLeftAxisY(){
        updateController();
        return xbox.getRawAxis(1);
    }

    public static double getLeftAxisX(){
        updateController();
        return xbox.getRawAxis(0);
    }

    public static double getRightAxisX(){
        updateController();
        return xbox.getRawAxis(4);
    }

    public static double getRightAxisY(){
        updateController();
        return xbox.getRawAxis(5);
    }

    public static boolean getRightJoystick(){
        updateController();
        return xbox.getRawButton(10);
    }

    public static boolean getLeftJoystick(){
        updateController();
        return xbox.getRawButton(9);
    }

    public static int getPOVUp(){
        updateController();
        return xbox.getPOV(0);
    }

    public static int getPOVDown(){
        updateController();
        return xbox.getPOV(180);
    }

    public static int getPOVRight(){
        updateController();
        return xbox.getPOV(90);
    }

    public static int getPOVLeft(){
        updateController();
        return xbox.getPOV(270);
    }

    public static boolean getEndButton(){
        updateController();
        return xbox.getRawButton(7);
    }

    public static boolean getStartButton(){
        updateController();
        return xbox.getRawButton(8);
    }






}
