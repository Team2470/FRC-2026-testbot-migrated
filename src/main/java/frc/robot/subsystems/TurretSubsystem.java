package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX m_turretMotor                 = new TalonFX(shooterConstants.TURRET_DEVICE_ID);
    private final CANcoder turretEncoder                = new CANcoder(shooterConstants.TURRET_CANCODER_ID);
    private final MotionMagicVoltage positionRequest    = new MotionMagicVoltage(0);

    private StatusSignal<AngularVelocity> turretYawVelocity = m_turretMotor.getVelocity();
    private StatusSignal<Angle> turretYawPosition           = m_turretMotor.getPosition();
    public final double toleranceDegrees = 5.0;

    public double targetYaw = Units.radiansToDegrees(0);

    // Adjust based on your physical gear ratio (e.g., 100:1)
    // private final double GEAR_RATIO = shooterConstants.TURRET_GEAR_RATIO;

    public TurretSubsystem() {
        TalonFXConfiguration config                             = new TalonFXConfiguration();
        config.Slot0.kP                                         = shooterConstants.TURRET_KP;
        config.Slot0.kI                                         = shooterConstants.TURRET_KI;
        config.Slot0.kD                                         = shooterConstants.TURRET_KD;
        config.Slot0.kV                                         = shooterConstants.TURRET_KV;
        config.MotionMagic.MotionMagicCruiseVelocity            = shooterConstants.TURRET_MOTION_MAGIC_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration              = shooterConstants.TURRET_MOTION_MAGIC_ACCELERACTIION;

        config.MotorOutput.NeutralMode                          = NeutralModeValue.Brake;

        config.Feedback.FeedbackRemoteSensorID                  = turretEncoder.getDeviceID();
        config.Feedback.FeedbackSensorSource                    = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.SensorToMechanismRatio                  = shooterConstants.TURRET_ENCODER_TO_TURRET_GEAR_RATIO;
        config.Feedback.RotorToSensorRatio                      = shooterConstants.TURRET_MOTOR_TO_ENCODER_RATIO;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable       = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable       = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold    = shooterConstants.MAX_TURRET_SOFT_LIMIT;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold    = shooterConstants.MIN_TURRET_SOFT_LIMIT;
        m_turretMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        updateMotorControl();
    }

    public void updateMotorControl() {
        double targetRotations = targetYaw;
        setAngle(targetRotations);
    }

    // public void resetToEncoder() {
        // double encoderPosition = turretEncoder.getAbsolutePosition();
        // double turretRotations = encoderPosition / shooterConstants.TURRET_ENCODER_TO_TURRET_GEAR_RATIO;
        // m_turretMotor.setPosition(turretRotations);
    // }

    public void setAngle(double degrees) {
        double clampedDegrees = MathUtil.clamp(degrees, -175.0, 175.0);
        // Because SensorToMechanismRatio is 45.0, 1.0 = 1 Turret Rotation
        double turretRotations = clampedDegrees / 360.0;

        m_turretMotor.setControl(positionRequest.withPosition(turretRotations));
    }

    public boolean isOnTarget(double toleranceDegrees) {
        BaseStatusSignal.refreshAll(turretYawPosition, turretYawVelocity);
        Angle currentYaw = BaseStatusSignal.getLatencyCompensatedValue(turretYawPosition, turretYawVelocity);
        return MathUtil.isNear(targetYaw, currentYaw.in(Degrees), toleranceDegrees);
    }

    public Angle getYaw(){
        BaseStatusSignal.refreshAll(turretYawPosition, turretYawVelocity);
        return BaseStatusSignal.getLatencyCompensatedValue(turretYawPosition, turretYawVelocity);
    }

    public AngularVelocity getYawVelocity() {
        return turretYawVelocity.refresh().getValue();
    }

    public void setYawAngle(Angle targetAngle) {
        m_turretMotor.setControl(positionRequest.withPosition((targetAngle)));
    }
}