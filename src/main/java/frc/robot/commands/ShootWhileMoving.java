package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.fieldConstants;
import frc.robot.Constants.shooterConstants;
import frc.robot.Constants.shooterConstants.SHOOTER_PARAMETERS;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootWhileMoving extends Command {
    private final CommandSwerveDrivetrain   m_drive;
    private final ShooterSubsystem          m_shooter;
    private final boolean                   isPassing;

    public double           turretToleranceDegrees;
    public double           hoodToleranceDegrees;
    public double           flywheelToleranceRPM;
    public Pose2d           robotPose;
    public double           robotToPassLeft;
    public double           robotToPassRight;
    public Translation2d    target_location = new Translation2d(0.0, 0.0);
    public Translation2d    robotToGoal     = new Translation2d(0.0, 0.0);


    public ShootWhileMoving(CommandSwerveDrivetrain drive,
                            ShooterSubsystem shooter,
                            boolean isPassing) {
        this.m_drive    = drive;
        this.m_shooter  = shooter;
        this.isPassing  = isPassing;
        addRequirements(m_shooter);
    }

    // Initialized is used to grab constants depending on if we are passing or not
    public void initialize() {
        robotPose                   = m_drive.getPose();
        if(isPassing) {
            turretToleranceDegrees  = shooterConstants.TURRET_PASS_TOLERANCE;
            flywheelToleranceRPM    = shooterConstants.RPM_PASS_TOLERANCE;
            // Find how far from each of 2 passing spots the robot is
            robotToPassLeft         = fieldConstants.PASS_LEFT_LOCATION.minus(robotPose.getTranslation()).getNorm();
            robotToPassRight        = fieldConstants.PASS_RIGHT_LOCATION.minus(robotPose.getTranslation()).getNorm();
            // Aim for the closest location
            target_location         = (robotToPassLeft < robotToPassRight) ? fieldConstants.PASS_LEFT_LOCATION :
                                                                                fieldConstants.PASS_RIGHT_LOCATION;
        } else {
            turretToleranceDegrees  = shooterConstants.TURRET_HUB_TOLERANCE;
            flywheelToleranceRPM    = shooterConstants.RPM_HUB_TOLERANCE;
            // Vector math to get basic distance (without including moving)
            target_location         = fieldConstants.HUB_LOCATION;
        }
    }

    @Override
    public void execute() {
        robotToGoal             = target_location.minus(robotPose.getTranslation());
        double physicalDistance = robotToGoal.getNorm();
        double timeOfFlight     = isPassing ? m_shooter.getPassParameters(physicalDistance).timeOfFlight() :
                                                m_shooter.getHubParameters(physicalDistance).timeOfFlight();

        // Get robot speed for virtual goal math
        ChassisSpeeds fieldSpeeds       = m_drive.getFieldRelativeSpeeds();

        // The "Virtual Goal" accounts for robot velocity during ball flight
        Translation2d virtualGoal       = target_location.minus(
            new Translation2d(fieldSpeeds.vxMetersPerSecond * timeOfFlight,
                              fieldSpeeds.vyMetersPerSecond * timeOfFlight)
        );

        // Vector match to include robot velocity
        Translation2d robotToVirtual    = virtualGoal.minus(robotPose.getTranslation());
        double virtualDistance          = robotToVirtual.getNorm();
        SHOOTER_PARAMETERS shotParams   = isPassing ? m_shooter.getPassParameters(virtualDistance) :
                                                        m_shooter.getHubParameters(virtualDistance);
        Rotation2d fieldRelativeTarget  = robotToVirtual.getAngle();
        Rotation2d robotRotation        = robotPose.getRotation();
        Rotation2d turretTarget         = fieldRelativeTarget.minus(robotRotation);
        double targetRPM                = shotParams.rpm();
        double targetHood               = shotParams.hoodPosition();

        // Set Subsystem Targets
        m_shooter.setTurretAngle(turretTarget);
        m_shooter.setRPM(targetRPM);
        m_shooter.setHoodPosition(targetHood);

        // Once Turret and shooter are at the correct set points
        // Unleash fuel into turret
        if (m_shooter.isTurretOnTarget(turretTarget, turretToleranceDegrees)
            && m_shooter.isAtSpeed(targetRPM, flywheelToleranceRPM)
            && m_shooter.isHoodOnTarget()) {
            m_shooter.runFeeder(shooterConstants.FEEDER_RUN);
        }
        else {
            m_shooter.runFeeder(shooterConstants.FEEDER_OFF);
        }
    }
}