package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.shooterConstants;
import frc.robot.Constants.shooterConstants.SHOOTER_PARAMETERS;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX flywheelMotorOne              = new TalonFX(shooterConstants.FLYWHEEL_1_DEVICE_ID);
    private final TalonFX flywheelMotorTwo              = new TalonFX(shooterConstants.FLYWHEEL_2_DEVICE_ID);
    private final TalonFX feederMotor                   = new TalonFX(shooterConstants.FEEDER_DEVICE_ID);
    private final Hood hood                             = new Hood();
    private final TurretSubsystem turret                = new TurretSubsystem();
    private final VelocityVoltage m_velocityRequest     = new VelocityVoltage(0);
    public double targetRPM;

    public ShooterSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // PID gains must be tuned for RPS (Phoenix 6 standard)
        config.Slot0.kP = shooterConstants.FLYWHEEL_KP;
        config.Slot0.kI = shooterConstants.FLYWHEEL_KI;
        config.Slot0.kD = shooterConstants.FLYWHEEL_KD;
        config.Slot0.kV = shooterConstants.FLYWHEEL_KV;
        flywheelMotorOne.getConfigurator().apply(config);
        flywheelMotorTwo.getConfigurator().apply(config);
        flywheelMotorTwo.optimizeBusUtilization();
        flywheelMotorTwo.setControl(new Follower(flywheelMotorOne.getDeviceID(), shooterConstants.FLYWHEEL_ALIGNMENT_VALUE));

    }

    public SHOOTER_PARAMETERS getHubParameters(double distance) {
        return shooterConstants.HUB_MAP.get(distance);
    }

    public SHOOTER_PARAMETERS getPassParameters(double distance) {
        return shooterConstants.PASS_MAP.get(distance);
    }

    public void runFeeder(double speed) {
       feederMotor.set(speed);
    }

    public void setHoodPosition(double position) {
        hood.setPosition(position);
    }

    public void setTurretAngle(Rotation2d degrees) {
        turret.targetAngle = degrees;
    }

    public void setRPM(double rpm) {
        // Phoenix sends values in Rotations Per Seconds (RPS)
        // Must handle value accordingly
        flywheelMotorOne.setControl(m_velocityRequest.withVelocity(rpm / Constants.SECONDS_PER_MINUTE));
    }

    public boolean isAtSpeed(double targetRPM, double tolerance) {
        // Get actual speed (Phoenix gives RPS), convert to RPM
        double currentRPM = flywheelMotorOne.getVelocity().getValueAsDouble() * Constants.SECONDS_PER_MINUTE;

        // Tolerance: Is Current RPM = Target RPM +/- Tolerance (measured in RPM)
        return Math.abs(currentRPM - targetRPM) < tolerance;
    }

    public boolean isHoodOnTarget() {
        return hood.isPositionWithinTolerance();
    }

    public boolean isTurretOnTarget(Rotation2d target, double toleranceDegrees) {
        return turret.isOnTarget(target, toleranceDegrees);
    }

    // Input: RPM of the main flywheel
    // Output: Ball's velocity coming out of shooter
    public double getExpectedExitVelocity(double mainRPM) {
        double mainRPS          = mainRPM / Constants.SECONDS_PER_MINUTE;
        double backspinRPS      = mainRPS * shooterConstants.BACKSPIN_GEAR_RATIO;
        double mainSurface      = mainRPS * Math.PI * shooterConstants.FLYWHEEL_DIAMETER_METERS;
        double backspinSurface  = backspinRPS * Math.PI * shooterConstants.BACKSPIN_DIAMETER_METERS;

        // The ball speed is roughly the average of the two contacting surfaces
        // Multiplied by efficiency (slip)
        return ((mainSurface + backspinSurface) / 2.0) * shooterConstants.SHOOTER_EFFICIENCY;
    }

    public Command runShooterCommand(){

        return Commands.runEnd(
            () -> {
                this.setRPM(targetRPM);
            },
            () -> { this.setRPM(0);}, this);
    }

}