package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootWhileMoving extends Command {
    private final CommandSwerveDrivetrain m_drive;
    private final TurretSubsystem m_turret;
    private final ShooterSubsystem m_shooter;
    private final boolean isPassing;

    public ShootWhileMoving(CommandSwerveDrivetrain drive, TurretSubsystem turret, ShooterSubsystem shooter, boolean isPassing) {
        this.m_drive = drive;
        this.m_turret = turret;
        this.m_shooter = shooter;
        this.isPassing = isPassing;
        addRequirements(m_turret, m_shooter);
    }

    @Override
    public void execute() {
        // Tolerances are different for if we are passing or shooting
        // Thought process being that we don't need to be as accurate when passing
        double turretToleranceDegrees   = isPassing ? 0.05 : 0.01;
        double hoodToleranceDegrees     = isPassing ? 0.05 : 0.01;
        double toleranceRPM             = isPassing ? 600 : 50;
        Pose2d robotPose                = m_drive.getPose();
        ChassisSpeeds fieldSpeeds       = m_drive.getFieldRelativeSpeeds();

        // Vector math to get basic distance (without including moving)
        Translation2d target_location   = isPassing ? new Translation2d(14.5, 1.0) : 
                                                        new Translation2d(8.25, 4.1);
        Translation2d robotToGoal       = target_location.minus(robotPose.getTranslation());
        double physicalDistance         = robotToGoal.getNorm();
        double guessedRPM               = isPassing ? m_shooter.getPassRPM(physicalDistance) :
                                                        m_shooter.getTargetRPM(physicalDistance);
        double ballExitVelocity         = m_shooter.getExpectedExitVelocity(guessedRPM);
        double guessedHoodAngle         = isPassing ? m_shooter.getPassHoodAngle(physicalDistance) : 
                                                        m_shooter.getHubHoodAngle(physicalDistance);
        double thetaRad                 = Math.toRadians(guessedHoodAngle);
        double horizontal               = ballExitVelocity * Math.cos(thetaRad);
        double flightTime               = physicalDistance / 
                                            (horizontal > 0 ? horizontal : 10.0);

        // The "Virtual Goal" accounts for robot velocity during ball flight
        Translation2d virtualGoal = target_location.minus(
            new Translation2d(fieldSpeeds.vxMetersPerSecond * flightTime, 
                              fieldSpeeds.vyMetersPerSecond * flightTime)
        );

        // Vector match to include robot velocity
        Translation2d robotToVirtual    = virtualGoal.minus(robotPose.getTranslation());
        double virtualDistance          = robotToVirtual.getNorm();
        Rotation2d fieldRelativeTarget  = robotToVirtual.getAngle();
        Rotation2d robotRotation        = robotPose.getRotation();
        Rotation2d turretTarget         = fieldRelativeTarget.minus(robotRotation);
        double targetRPM                = isPassing ? m_shooter.getPassRPM(virtualDistance) :
                                                        m_shooter.getTargetRPM(virtualDistance);
        double targetHood               = isPassing ? m_shooter.getPassHoodAngle(virtualDistance) :
                                                        m_shooter.getHubHoodAngle(virtualDistance);

        // Set Subsystem Targets
        m_turret.setTargetAngle(turretTarget);
        m_shooter.setRPM(targetRPM);
        m_shooter.setHoodAngle(targetHood);

        // Once Turret and shooter are at the correct set points
        // Unleash fuel into turret
        if (m_turret.isOnTarget(turretTarget, turretToleranceDegrees) 
            && m_shooter.isAtSpeed(targetRPM, toleranceRPM)
            && m_shooter.isHoodOnTarget(targetHood, hoodToleranceDegrees)) {
            m_shooter.runFeeder(1.0); // TODO: set feeder speed
        }
        else {
            m_shooter.runFeeder(0.0); // Turn shooter off if we are not at target values
        }
    }
}