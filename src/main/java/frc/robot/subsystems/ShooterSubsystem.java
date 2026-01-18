package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_topMotor = new TalonFX(10); // Example ID
    private final TalonFX m_feederMotor = new TalonFX(11);
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    private final InterpolatingDoubleTreeMap m_hubRPMMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_passRPMMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_hoodHubMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_hoodPassMap = new InterpolatingDoubleTreeMap();
    private static final double kWheelDiameterMeters = 0.1016; // 4 inches
    private static final double kEfficiency = 0.75; // Tune this based on dyno/video testing
    private final TalonFX m_hoodMotor = new TalonFX(13); 
    private final double HOOD_GEAR_RATIO = 50.0; // Example
    private final double HOOD_OFFSET_DEGREES = 25.0; // Physical minimum angle

public ShooterSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // PID gains must be tuned for RPS (Phoenix 6 standard)
        config.Slot0.kP = 0.11; 
        config.Slot0.kV = 0.12; 
        m_topMotor.getConfigurator().apply(config);

        var hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0.kP = 20.0; // Tune this!
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 75.0 / 360.0 * HOOD_GEAR_RATIO;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 25.0 / 360.0 * HOOD_GEAR_RATIO;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        m_hoodMotor.getConfigurator().apply(hoodConfig);

        // Populate Hood map with Hood Angle Values
        // Distance (meters), Hood Angle (degrees)
        m_hoodHubMap.put(2.0, 65.0); 
        m_hoodHubMap.put(4.0, 45.0);
        m_hoodHubMap.put(6.0, 30.0);
        
        // Populate Hood map with Hood Angle Values
        // Distance (meters), Hood Angle (degrees)
        m_hoodPassMap.put(2.0, 50.0); 
        m_hoodPassMap.put(4.0, 30.0);
        m_hoodPassMap.put(6.0, 25.0);
        
        // Populate Map with RPM values
        // Distance (meters), Flywheel Speed (RPM)
        m_hubRPMMap.put(2.0, 2500.0);
        m_hubRPMMap.put(4.0, 3200.0);
        m_hubRPMMap.put(6.0, 4500.0);
        
        // Passing Map (Flatter, faster shots)
        // Distance (meters), Flywheel Speed (RPM)
        m_passRPMMap.put(5.0, 3000.0);
        m_passRPMMap.put(10.0, 4000.0);
    }

    public double getTargetRPM(double distance) {
        return m_hubRPMMap.get(distance);
    }

    public double getPassRPM(double distance) {
        // You might want a flatter trajectory for passing
        return m_passRPMMap.get(distance); // Fixed speed or use a second LUT
    }
    
    public double getHubHoodAngle(double distance) {
        return m_hoodHubMap.get(distance);
    }
    
    public double getPassHoodAngle(double distance) {
        return m_hoodPassMap.get(distance);
    }

    public void runFeeder(double speed) {
       m_feederMotor.set(speed);
    }

    public void setHoodAngle(double degrees) {
        double clamped = Math.max(25.0, Math.min(75.0, degrees));
        double rotations = (clamped / 360.0) * HOOD_GEAR_RATIO;

        m_hoodMotor.setControl(new com.ctre.phoenix6.controls.PositionVoltage(rotations));
    }

    public void setRPM(double rpm) {
        // Phoenix sends values in Rotations Per Seconds (RPS)
        // Must handle value accordingly
        m_topMotor.setControl(m_velocityRequest.withVelocity(rpm / 60.0));
    }

    public boolean isAtSpeed(double targetRPM, double tolerance) {
        // Get actual speed (Phoenix gives RPS), convert to RPM
        double currentRPM = m_topMotor.getVelocity().getValueAsDouble() * 60.0;
        
        // Tolerance: +/- Tolerance (measured in RPM)
        return Math.abs(currentRPM - targetRPM) < tolerance;
    }

    public boolean isHoodOnTarget(double targetDegrees, double tolerance) {
        double currentRot = m_hoodMotor.getPosition().getValueAsDouble();
        double currentDeg = (currentRot / HOOD_GEAR_RATIO) * 360.0;
        return Math.abs(currentDeg - targetDegrees) < tolerance;
    }


    public double getExpectedExitVelocity(double rpm) {
        double rps = rpm / 60.0;
        // v = RPS * PI * D * Efficiency
        // We divide by 2 because it's a hooded shooter (one side is 0 m/s)
        return (rps * Math.PI * kWheelDiameterMeters * kEfficiency) / 2.0;
    }
}