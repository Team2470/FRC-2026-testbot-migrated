package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.fieldConstants;
import frc.robot.Constants.shooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootWhileMoving extends Command {
    private final CommandSwerveDrivetrain   m_drive;
    private final TurretSubsystem           m_turret;
    private final ShooterSubsystem          m_shooter;
    private final boolean                   isPassing;

    public ShootWhileMoving(CommandSwerveDrivetrain drive,
                            TurretSubsystem turret,
                            ShooterSubsystem shooter,
                            boolean isPassing) {
        this.m_drive    = drive;
        this.m_turret   = turret;
        this.m_shooter  = shooter;
        this.isPassing  = isPassing;
        addRequirements(m_turret, m_shooter);
    }

    @Override
    public void execute() {
        // Tolerances are different for if we are passing or shooting
        // Thought process being that we don't need to be as accurate when passing
        double turretToleranceDegrees   = isPassing ? shooterConstants.TURRET_PASS_TOLERANCE :
                                                        shooterConstants.TURRET_HUB_TOLERANCE;
        double hoodToleranceDegrees     = isPassing ? shooterConstants.HOOD_PASS_TOLERANCE :
                                                        shooterConstants.HOOD_HUB_TOLERANCE;
        double flywheelToleranceRPM     = isPassing ? shooterConstants.RPM_PASS_TOLERANCE :
                                                        shooterConstants.RPM_HUB_TOLERANCE;

        // Get Current Pose for vector math
        Pose2d robotPose                = m_drive.getPose();

        // Find how far from each of 2 passing spots the robot is
        double robotToPassLeft          = fieldConstants.PASS_LEFT_LOCATION.minus(robotPose.getTranslation()).getNorm();
        double robotToPassRight         = fieldConstants.PASS_RIGHT_LOCATION.minus(robotPose.getTranslation()).getNorm();

        // Aim for the closest location
        Translation2d passLocation      = (robotToPassLeft < robotToPassRight) ? fieldConstants.PASS_LEFT_LOCATION :
                                                                                    fieldConstants.PASS_RIGHT_LOCATION;

        // Vector math to get basic distance (without including moving)
        Translation2d target_location   = isPassing ? passLocation :
                                                        fieldConstants.HUB_LOCATION;
        Translation2d robotToGoal       = target_location.minus(robotPose.getTranslation());
        double physicalDistance         = robotToGoal.getNorm();
        double timeOfFlight             = isPassing ? m_shooter.getPassTOF(physicalDistance) :
                                                        m_shooter.getHubTOF(physicalDistance);

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
        Rotation2d fieldRelativeTarget  = robotToVirtual.getAngle();
        Rotation2d robotRotation        = robotPose.getRotation();
        Rotation2d turretTarget         = fieldRelativeTarget.minus(robotRotation);
        double targetRPM                = isPassing ? m_shooter.getPassRPM(virtualDistance) :
                                                        m_shooter.getHubRPM(virtualDistance);
        double targetHood               = isPassing ? m_shooter.getPassHoodAngle(virtualDistance) :
                                                        m_shooter.getHubHoodAngle(virtualDistance);

        // Set Subsystem Targets
        m_turret.setTargetAngle(turretTarget);
        m_shooter.setRPM(targetRPM);
        m_shooter.setHoodAngle(targetHood);

        // Once Turret and shooter are at the correct set points
        // Unleash fuel into turret
        if (m_turret.isOnTarget(turretTarget, turretToleranceDegrees)
            && m_shooter.isAtSpeed(targetRPM, flywheelToleranceRPM)
            && m_shooter.isHoodOnTarget(targetHood, hoodToleranceDegrees)) {
            m_shooter.runFeeder(shooterConstants.FEEDER_RUN);
        }
        else {
            m_shooter.runFeeder(shooterConstants.FEEDER_OFF);
        }
    }
}