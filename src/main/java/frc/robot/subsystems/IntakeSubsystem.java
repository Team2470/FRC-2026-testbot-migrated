package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.intakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  //change id
  private final TalonFX m_intake = new TalonFX(intakeConstants.INTAKE_ROLLERS_DEVICE_ID);;
  private VelocityTorqueCurrentFOC velocity = new VelocityTorqueCurrentFOC(0);

  public IntakeSubsystem() {
    TalonFXConfiguration config           = new TalonFXConfiguration();
    TorqueCurrentConfigs torqueConfig     = new TorqueCurrentConfigs();
    torqueConfig.PeakForwardTorqueCurrent = 100.00; // 100 Amps
    torqueConfig.PeakReverseTorqueCurrent = -100.00; // -100 Amps
    torqueConfig.TorqueNeutralDeadband    = 0.5; // 0.5 amp deadband
    config.withTorqueCurrent(torqueConfig);
  }


  public void intake() {
    m_intake.setControl(velocity.withVelocity(intakeConstants.INTAKE_RPM / Constants.SECONDS_PER_MINUTE));
  }

  public void reverse_intake () {
    m_intake.setControl(velocity.withVelocity(intakeConstants.OUTTAKE_RPM / Constants.SECONDS_PER_MINUTE));

  }
  public void intakePercet(double volt){
    m_intake.setVoltage(volt);
  }
  public void stop() {
    m_intake.stopMotor();
  }
  public Command intakeCommand() {
    return Commands.runEnd(
      ()-> this.intake(),
      this::stop,
      this);
  }
    public Command outtakeCommand() {
      return Commands.runEnd(
      ()-> this.reverse_intake(),
      this::stop,
      this);
  }
  public Command intakePercentCommand(double volt){
      return Commands.runEnd(
      ()-> this.intakePercet(volt),
      this::stop,
      this);
  }
}