package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX m_turretMotor = new TalonFX(11);
    private final MotionMagicVoltage m_mmRequest = new MotionMagicVoltage(0);
    
    // Adjust based on your physical gear ratio (e.g., 100:1)
    private final double GEAR_RATIO = 100.0; 

    public TurretSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 12.0; 
        config.MotionMagic.MotionMagicCruiseVelocity = 80; 
        config.MotionMagic.MotionMagicAcceleration = 160;
        m_turretMotor.getConfigurator().apply(config);
    }

    /**
     * Set turret position.
     * @param robotRelativeAngle Angle relative to the robot's front.
     */
    public void setTargetAngle(Rotation2d robotRelativeAngle) {
        double rotations = robotRelativeAngle.getRotations() * GEAR_RATIO;
        m_turretMotor.setControl(m_mmRequest.withPosition(rotations));
    }

    public boolean isOnTarget(Rotation2d target, double toleranceDegrees) {
        double currentRot = m_turretMotor.getPosition().getValueAsDouble() / GEAR_RATIO;
        double error = Math.abs(currentRot - target.getRotations()) * 360.0;
        return error < toleranceDegrees;
    }
}