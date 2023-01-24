package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  DigitalInput irInput = new DigitalInput(9);

  public IntakeSubsystem() {}

  @Override
  public void periodic() {}

  public boolean getIRInput() {
    return !irInput.get();
  }
}
