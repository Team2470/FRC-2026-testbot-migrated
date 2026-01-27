package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.jni.PlatformJNI;
import com.ctre.phoenix6.sim.DeviceType;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.spns.*;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.Follower;
import frc.robot.Constants;
import frc.robot.Constants.shooterConstants;


public class Shooter extends SubsystemBase {

     private final TalonFX m_topMotor_1                      = new TalonFX(shooterConstants.FLYWHEEL_1_DEVICE_ID);
    private final TalonFX m_topMotor_2                      = new TalonFX(shooterConstants.FLYWHEEL_2_DEVICE_ID);
    // private final TalonFX m_feederMotor                     = new TalonFX(shooterConstants.FEEDER_DEVICE_ID);
    // private final TalonFX m_hoodMotor                       = new TalonFX(shooterConstants.HOOD_DEVICE_ID); 
    private final VelocityVoltage m_velocityRequest         = new VelocityVoltage(0);

    // private final TalonFX m_motor;
    // private final CANcoder m_encoder;
    // //placeholder constants for PID
    // private double m_demand;
    // private final PIDController m_pidController = new PIDController(shooterConstants.FLYWHEEL_KP, shooterConstants.FLYWHEEL_KI, shooterConstants.FLYWHEEL_KD);


    private enum ControlMode {
        kOpenLoop, kPID
    }

    private ControlMode m_controlMode = ControlMode.kOpenLoop;
    
    
    // formula for ballistic trajectory WITHOUT DRAG, change in the future to account for this if needed 
    private Double angleCalculator(Double v,  Double x, Double y) {
        return(Math.atan((Math.pow(v, 2) + Math.sqrt(Math.pow(v, 4) - 9.8 * (9.8 * Math.pow(x,2) + 2 * y * Math.pow(v,2))))/ (9.8 * x)));
    }
    
    public Shooter() {
        
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

       
    }public double getHubRPM(double distance) {
        return shooterConstants.HUB_RPM_MAP.get(distance);
    }

    public double getPassRPM(double distance) {
        return shooterConstants.PASS_RPM_MAP.get(distance);
    }

    public double getHubTOF(double distance) {
        return shooterConstants.HUB_TOF_MAP.get(distance);
    }

    public double getPassTOF(double distance) {
        return shooterConstants.PASS_TOF_MAP.get(distance);
    }


    /*  public double getHubHoodAngle(double distance) {
         return shooterConstants.HOOD_HUB_MAP.get(distance);
     }
    
     public double getPassHoodAngle(double distance) {
         return shooterConstants.HOOD_PASS_MAP.get(distance);
    }

    public void runFeeder(double speed) {
       m_feederMotor.set(speed);
    } */ 

/*  public void setHoodAngle(double degrees) {
        double clamped      = Math.max(shooterConstants.MIN_HOOD_ANGLE, 
                                Math.min(shooterConstants.MAX_HOOD_ANGLE, degrees));
        double rotations    = (clamped / 360.0) * shooterConstants.HOOD_GEAR_RATIO;

        m_hoodMotor.setControl(new com.ctre.phoenix6.controls.PositionVoltage(rotations));
    } */

    public void setRPM(double rpm) {
        // Phoenix sends values in Rotations Per Seconds (RPS)
        // Must handle value accordingly
        m_topMotor_1.setControl(m_velocityRequest.withVelocity(rpm / Constants.MINUTE_TO_SECONDS));
        SmartDashboard.putNumber("Flywheel Velocity ", rpm);
    }

    public boolean isAtSpeed(double targetRPM, double tolerance) {
        // Get actual speed (Phoenix gives RPS), convert to RPM
        double currentRPM = m_topMotor_1.getVelocity().getValueAsDouble() * Constants.MINUTE_TO_SECONDS;
        
        // Tolerance: Is Current RPM = Target RPM +/- Tolerance (measured in RPM)
        return Math.abs(currentRPM - targetRPM) < tolerance;
    }

    // public boolean isHoodOnTarget(double targetDegrees, double tolerance) {
    //     double currentRot = m_hoodMotor.getPosition().getValueAsDouble();
    //     double currentDeg = (currentRot / shooterConstants.HOOD_GEAR_RATIO) * 360.0;
        
    //     // Tolerance: Is Current Angle = Target Angle +/- Tolerance (measured in Degrees) 
    //     return Math.abs(currentDeg - targetDegrees) < tolerance;
    // }

    // Input: RPM of the main flywheel
    // Output: Ball's velocity coming out of shooter
    // public double getExpectedExitVelocity(double mainRPM) {
    //     double mainRPS      = mainRPM / Constants.MINUTE_TO_SECONDS;
    //     double topRPS       = mainRPS * shooterConstants.BACKSPIN_GEAR_RATIO;
    //     double mainSurface  = mainRPS * Math.PI * shooterConstants.FLYWHEEL_DIAMETER_METERS;
    //     double topSurface   = topRPS * Math.PI * shooterConstants.BACKSPIN_DIAMETER_METERS;
        
    //     // The ball speed is roughly the average of the two contacting surfaces
    //     // Multiplied by efficiency (slip)
    //     return ((mainSurface + topSurface) / 2.0) * shooterConstants.SHOOTER_EFFICIENCY;
    // }

//     @Override
//     public void periodic () {
//         // Determine output voltage
//     double outputVoltage = 0;
// 	switch (m_controlMode) {
//         case kOpenLoop:
//             // Do openloop stuff here
//             outputVoltage = m_demand;
//             break;

//         case kPID:
//             m_pidController.setP(SmartDashboard.getNumber("kP", shooterConstants.FLYWHEEL_KP));
//             m_pidController.setI(SmartDashboard.getNumber("kI", shooterConstants.FLYWHEEL_KI));
//             m_pidController.setD(SmartDashboard.getNumber("kD", shooterConstants.FLYWHEEL_KD));
//             double kF = SmartDashboard.getNumber("kF", shooterConstants.FLYWHEEL_KV);

//             // Do PID stuff
//             outputVoltage = kF * m_demand + m_pidController.calculate(getVelocity(), m_demand);

//             break;
//         default:
//             // What happened!?
//             break;
//         }


// 	// Do something with the motor
// 	m_motor.setVoltage(outputVoltage);


// 	// Publish to smart dashboard

//     }
    

// public double getErrorRPM(){
// 	if (m_controlMode == ControlMode.kPID){
// 	return m_pidController.getPositionError();
// 	}
// 	return 0;
// }

// public double getErrorPercent(){
// 	if (m_controlMode == ControlMode.kPID) {
// 	return (m_demand - m_encoder.getVelocity().getValueAsDouble()) / m_demand * 10;
// 	}

// 	return 0;
// }
// public boolean isErrorInRange() {
// 	return (-4 < this.getErrorPercent() && this.getErrorPercent() < 4);
// }

// 	public boolean isErrorBelow() {
// 	return (-5 > this.getErrorPercent());
// }

// 	public boolean isErrorAbove() {
// 	return (this.getErrorPercent() > 5);
// }

// public Command waitUntilErrorInrange(){
// return Commands.waitUntil(()-> this.isErrorInRange());
// }

// public boolean isErrorOutOfRange() {
// 	return (this.getErrorPercent() > 15);
// }

// public Command waitUntilErrorOutOfRange(){
// return Commands.waitUntil(() -> this.isErrorOutOfRange());

// }

// public void setOutputVoltage(double OutputVoltage) {
// 	m_controlMode = ControlMode.kOpenLoop;
// 	m_demand = OutputVoltage;
// }

// public void setPIDSetpoint(double rpm) {
// 	m_controlMode = ControlMode.kPID;
// 	m_demand = rpm;
// }

// public void stop() {
// 	setOutputVoltage(0);
// }
// /**
// * Example command factory method.
// *
// * @return a command
// */
// public Command openLoopCommand(DoubleSupplier OutputVoltageSupplier) {


// 	// Inline construction of command goes here.
// 	// Subsystem::RunOnce implicitly requires `this` subsystem.
// 	return Commands.runEnd(
// 		() -> this.setOutputVoltage(OutputVoltageSupplier.getAsDouble()), this::stop, this);

// }

// public Command openLoopCommand(double OutputVoltage) {
// 	return openLoopCommand(()-> OutputVoltage);
// }


// public Command pidCommand(DoubleSupplier rpmSupplier){
// 	return Commands.runEnd(
// 	() -> this.setPIDSetpoint(rpmSupplier.getAsDouble()), this::stop, this);
// }

// public Command pidCommand(double rpm){
// 	return pidCommand(() -> rpm);
// }
public Command runShooterCommand(){
    return Commands.runEnd(
        () -> {
            this.setRPM(50);
        },
        () -> { this.setRPM(0);}, this);
    }
}

