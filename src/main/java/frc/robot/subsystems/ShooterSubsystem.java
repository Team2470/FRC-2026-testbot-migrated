package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.shooterConstants;
import frc.robot.Constants.shooterConstants.SHOOTER_PARAMETERS;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_topMotor_1                      = new TalonFX(shooterConstants.FLYWHEEL_1_DEVICE_ID);
    private final TalonFX m_topMotor_2                      = new TalonFX(shooterConstants.FLYWHEEL_2_DEVICE_ID);
    private final TalonFX m_feederMotor                     = new TalonFX(shooterConstants.FEEDER_DEVICE_ID);
    private final Hood m_hood                               = new Hood();
    private final VelocityVoltage m_velocityRequest         = new VelocityVoltage(0);

public ShooterSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // PID gains must be tuned for RPS (Phoenix 6 standard)
        config.Slot0.kP = shooterConstants.FLYWHEEL_KP;
        config.Slot0.kI = shooterConstants.FLYWHEEL_KI;
        config.Slot0.kD = shooterConstants.FLYWHEEL_KD;
        config.Slot0.kV = shooterConstants.FLYWHEEL_KV;
        m_topMotor_1.getConfigurator().apply(config);
        m_topMotor_2.getConfigurator().apply(config);
        m_topMotor_2.optimizeBusUtilization();
        m_topMotor_2.setControl(new Follower(m_topMotor_1.getDeviceID(), shooterConstants.FLYWHEEL_ALIGNMENT_VALUE));

    }

    public SHOOTER_PARAMETERS getHubParameters(double distance) {
        return shooterConstants.HUB_MAP.get(distance);
    }

    public SHOOTER_PARAMETERS getPassParameters(double distance) {
        return shooterConstants.PASS_MAP.get(distance);
    }

    public void runFeeder(double speed) {
       m_feederMotor.set(speed);
    }

    public void setHoodAngle(double degrees) {
        double clamped      = Math.max(shooterConstants.MIN_HOOD_ANGLE,
                                Math.min(shooterConstants.MAX_HOOD_ANGLE, degrees));
        double rotations    = (clamped / 360.0) * shooterConstants.HOOD_LENGTH_TO_ANGLE_RATIO;

        double rotations_to_servo_pose = rotations / (2 * Math.PI); // TODO: fix math for ratio of servo position to hood angle
        m_hood.setPosition(rotations_to_servo_pose);
    }

    public void setRPM(double rpm) {
        // Phoenix sends values in Rotations Per Seconds (RPS)
        // Must handle value accordingly
        m_topMotor_1.setControl(m_velocityRequest.withVelocity(rpm / Constants.SECONDS_PER_MINUTE));
    }

    public boolean isAtSpeed(double targetRPM, double tolerance) {
        // Get actual speed (Phoenix gives RPS), convert to RPM
        double currentRPM = m_topMotor_1.getVelocity().getValueAsDouble() * Constants.SECONDS_PER_MINUTE;

        // Tolerance: Is Current RPM = Target RPM +/- Tolerance (measured in RPM)
        return Math.abs(currentRPM - targetRPM) < tolerance;
    }

    public boolean isHoodOnTarget(double targetDegrees, double tolerance) {
        double currentHoodPose = m_hood.getPosition();
        double currentRot = currentHoodPose * 2 * Math.PI; // TODO: fix math for ratio of servo position to hood angle
        double currentDeg = (currentRot / shooterConstants.HOOD_LENGTH_TO_ANGLE_RATIO) * 360.0;

        // Tolerance: Is Current Angle = Target Angle +/- Tolerance (measured in Degrees)
        return Math.abs(currentDeg - targetDegrees) < tolerance;
    }

    // Input: RPM of the main flywheel
    // Output: Ball's velocity coming out of shooter
    public double getExpectedExitVelocity(double mainRPM) {
        double mainRPS      = mainRPM / Constants.SECONDS_PER_MINUTE;
        double topRPS       = mainRPS * shooterConstants.BACKSPIN_GEAR_RATIO;
        double mainSurface  = mainRPS * Math.PI * shooterConstants.FLYWHEEL_DIAMETER_METERS;
        double topSurface   = topRPS * Math.PI * shooterConstants.BACKSPIN_DIAMETER_METERS;

        // The ball speed is roughly the average of the two contacting surfaces
        // Multiplied by efficiency (slip)
        return ((mainSurface + topSurface) / 2.0) * shooterConstants.SHOOTER_EFFICIENCY;
    }

}