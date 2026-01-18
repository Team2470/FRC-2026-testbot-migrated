package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_topMotor                        = new TalonFX(10); // Example ID
    private final TalonFX m_feederMotor                     = new TalonFX(11);
    private static final double kMainWheelDiameterMeters    = 0.1016;   // 4 inches
    private static final double kTopWheelDiameterMeters     = 0.0508;   // 2 inches (Meters)
    private static final double kTopWheelGearRatio          = 0.5;      // Figure out specific gear ratio
    private static final double kEfficiency                 = 0.85;     // Tune this based on dyno/video testing
    private final VelocityVoltage m_velocityRequest         = new VelocityVoltage(0);
    private final InterpolatingDoubleTreeMap m_hubRPMMap    = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_passRPMMap   = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_hoodHubMap   = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_hoodPassMap  = new InterpolatingDoubleTreeMap();
    private final TalonFX m_hoodMotor                       = new TalonFX(13); 
    private final double HOOD_GEAR_RATIO                    = 50.0; // Example

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

        // TODO: ALL MAP VALUES SHOULD COME FROM TESTING,
        // CURRENT VALUES ARE FILLERS AND DO NOT REPRESENT
        // REAL WORLD DATA

        // This map is for the Hood angle 
        // when we are shooting into hub
        // Distance (meters), Hood Angle (degrees)
        m_hoodHubMap.put(2.0, 65.0); 
        m_hoodHubMap.put(4.0, 45.0);
        m_hoodHubMap.put(6.0, 30.0);
        
        // This map is for the Hood angle 
        // when we are passing into alliance zone
        // Distance (meters), Hood Angle (degrees)
        m_hoodPassMap.put(2.0, 50.0); 
        m_hoodPassMap.put(4.0, 30.0);
        m_hoodPassMap.put(6.0, 25.0);
        
        // This map is for the shooter flywheel
        // when we are shooting into hub
        // Distance (meters), Flywheel Speed (RPM)
        m_hubRPMMap.put(2.0, 2500.0);
        m_hubRPMMap.put(4.0, 3200.0);
        m_hubRPMMap.put(6.0, 4500.0);
        
        // This map is for the shooter flywheel
        // when we are passing into alliance zone
        // Distance (meters), Flywheel Speed (RPM)
        m_passRPMMap.put(5.0, 3000.0);
        m_passRPMMap.put(10.0, 4000.0);
        m_passRPMMap.put(20.0, 7500.0);
    }

    public double getTargetRPM(double distance) {
        return m_hubRPMMap.get(distance);
    }

    public double getPassRPM(double distance) {
        return m_passRPMMap.get(distance);
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
        
        // Tolerance: Is Current RPM = Target RPM +/- Tolerance (measured in RPM)
        return Math.abs(currentRPM - targetRPM) < tolerance;
    }

    public boolean isHoodOnTarget(double targetDegrees, double tolerance) {
        double currentRot = m_hoodMotor.getPosition().getValueAsDouble();
        double currentDeg = (currentRot / HOOD_GEAR_RATIO) * 360.0;
        
        // Tolerance: Is Current Angle = Target Angle +/- Tolerance (measured in Degrees) 
        return Math.abs(currentDeg - targetDegrees) < tolerance;
    }

    // Input: RPM of the main flywheel
    // Output: Ball's velocity coming out of shooter
    public double getExpectedExitVelocity(double mainRPM) {
        double mainRPS = mainRPM / 60.0;
        double topRPS = mainRPS * kTopWheelGearRatio;

        double mainSurface = mainRPS * Math.PI * kMainWheelDiameterMeters;
        double topSurface = topRPS * Math.PI * kTopWheelDiameterMeters;
        
        // The ball speed is roughly the average of the two contacting surfaces
        // Multiplied by efficiency (slip)
        return ((mainSurface + topSurface) / 2.0) * kEfficiency;
    }
}