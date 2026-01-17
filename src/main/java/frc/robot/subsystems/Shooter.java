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
import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix6.hardware.TalonFX;


public class Shooter extends SubsystemBase {
    private final TalonFX m_motor;
    private final CANcoder m_encoder;
    //placeholder constants for PID
    private double m_demand;
    private final PIDController m_pidController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);

    private enum ControlMode {
        kOpenLoop, kPID
    }

    private ControlMode m_controlMode = ControlMode.kOpenLoop;
    
    
    // formula for ballistic trajectory WITHOUT DRAG, change in the future to account for this if needed 
    private Double angleCalculator(Double v,  Double x, Double y) {
        return(Math.atan((Math.pow(v, 2) + Math.sqrt(Math.pow(v, 4) - 9.8 * (9.8 * Math.pow(x,2) + 2 * y * Math.pow(v,2))))/ (9.8 * x)));
    }
    
    public Shooter() {
        
        m_encoder = new CANcoder(30, "rio");
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Feedback.FeedbackRemoteSensorID = m_encoder.getDeviceID();
        
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        m_motor = new TalonFX(30, "rio");
        m_motor.getConfigurator().apply(motorConfig);

        m_motor.getPosition().setUpdateFrequency(50);
        m_motor.getVelocity().setUpdateFrequency(50);
        m_motor.optimizeBusUtilization();
       
    }

    public double getPosition() {
        return Units.rotationsToDegrees(m_motor.getPosition().getValueAsDouble());
    }

    public double getVelocity() {
        return Units.rotationsToDegrees(m_motor.getVelocity().getValueAsDouble());
    }

    @Override
    public void periodic () {
        // Determine output voltage
    double outputVoltage = 0;
	switch (m_controlMode) {
        case kOpenLoop:
            // Do openloop stuff here
            outputVoltage = m_demand;
            break;

        case kPID:
            m_pidController.setP(SmartDashboard.getNumber("kP", ShooterConstants.kP));
            m_pidController.setI(SmartDashboard.getNumber("kI", ShooterConstants.kI));
            m_pidController.setD(SmartDashboard.getNumber("kD", ShooterConstants.kD));
            double kF = SmartDashboard.getNumber("kF", ShooterConstants.kF);

            // Do PID stuff
            outputVoltage = kF * m_demand + m_pidController.calculate(getVelocity(), m_demand);

            break;
        default:
            // What happened!?
            break;
        }


	// Do something with the motor
	m_motor.setVoltage(outputVoltage);


	// Publish to smart dashboard

    }
    

public double getErrorRPM(){
	if (m_controlMode == ControlMode.kPID){
	return m_pidController.getPositionError();
	}
	return 0;
}

public double getErrorPercent(){
	if (m_controlMode == ControlMode.kPID) {
	return (m_demand - m_encoder.getVelocity().getValueAsDouble()) / m_demand * 10;
	}

	return 0;
}
public boolean isErrorInRange() {
	return (-4 < this.getErrorPercent() && this.getErrorPercent() < 4);
}

	public boolean isErrorBelow() {
	return (-5 > this.getErrorPercent());
}

	public boolean isErrorAbove() {
	return (this.getErrorPercent() > 5);
}

public Command waitUntilErrorInrange(){
return Commands.waitUntil(()-> this.isErrorInRange());
}

public boolean isErrorOutOfRange() {
	return (this.getErrorPercent() > 15);
}

public Command waitUntilErrorOutOfRange(){
return Commands.waitUntil(() -> this.isErrorOutOfRange());

}

public void setOutputVoltage(double OutputVoltage) {
	m_controlMode = ControlMode.kOpenLoop;
	m_demand = OutputVoltage;
}

public void setPIDSetpoint(double rpm) {
	m_controlMode = ControlMode.kPID;
	m_demand = rpm;
}

public void stop() {
	setOutputVoltage(0);
}
/**
* Example command factory method.
*
* @return a command
*/
public Command openLoopCommand(DoubleSupplier OutputVoltageSupplier) {


	// Inline construction of command goes here.
	// Subsystem::RunOnce implicitly requires `this` subsystem.
	return Commands.runEnd(
		() -> this.setOutputVoltage(OutputVoltageSupplier.getAsDouble()), this::stop, this);

}

public Command openLoopCommand(double OutputVoltage) {
	return openLoopCommand(()-> OutputVoltage);
}


public Command pidCommand(DoubleSupplier rpmSupplier){
	return Commands.runEnd(
	() -> this.setPIDSetpoint(rpmSupplier.getAsDouble()), this::stop, this);
}

public Command pidCommand(double rpm){
	return pidCommand(() -> rpm);
}
}

