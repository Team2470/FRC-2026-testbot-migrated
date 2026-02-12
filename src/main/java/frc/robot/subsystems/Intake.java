package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(0);

  public void runMotor() {
    m_motor.set(0.8);
  }

  public void stopMotor() {
    m_motor.set(0.0);
  }
}