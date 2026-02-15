package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutAngle;
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
    // Reusable object to prevent reallocation (to reduce memory pressure)
    private final MutAngle turretYawTarget = Rotations.mutable(0);


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

        ChassisSpeeds fieldSpeeds       = m_drive.getFieldRelativeSpeeds();
        var curretTurretYaw = m_shooter.turret.getYaw();
        var turretTranslation   = m_shooter.getTurretTranslation(robotPose, curretTurretYaw);
        var vRobot = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
        var omegaRobot = fieldSpeeds.omegaRadiansPerSecond;
        var turretVelocity = m_shooter.turret.getYawVelocity().in(RadiansPerSecond);
        var totalOmega = omegaRobot + turretVelocity;

        var robotToShot = turretTranslation.minus(robotPose.getTranslation());
        var vTanTotal   = new Translation2d(-totalOmega * robotToShot.getY(), totalOmega * robotToShot.getX());

        var effectiveShooterVelocity = vRobot.plus(vTanTotal);

        var predictedTargetTranslation = target_location;

        SHOOTER_PARAMETERS shotParameters;

        // Iterate 4 times to converge on the intersection of trajectory and target
        for (int i = 0; i < 4; i++) {
            var dist = predictedTargetTranslation.getDistance(turretTranslation);
            shotParameters = isPassing ? m_shooter.getPassParameters(dist) :
                                            m_shooter.getHubParameters(dist);
            var timeUntilScored = shotParameters.timeOfFlight();
            var targetPredictedOffset = effectiveShooterVelocity.times(timeUntilScored);
            predictedTargetTranslation = target_location.minus(targetPredictedOffset);
        }

        shotParameters = isPassing ? m_shooter.getPassParameters(predictedTargetTranslation.getDistance(turretTranslation)) :
                                        m_shooter.getHubParameters(predictedTargetTranslation.getDistance(turretTranslation));

        var angleToTarget = predictedTargetTranslation.minus(turretTranslation).getAngle();
        turretYawTarget.mut_replace(angleToTarget.minus(robotPose.getRotation()).getRotations(), Rotations);
        m_shooter.turret.setYawAngle(turretYawTarget);
        m_shooter.setRPM(shotParameters.rpm());
        m_shooter.setHoodPosition(shotParameters.hoodPosition());

        // Once Turret and shooter are at the correct set points
        // Unleash fuel into turret
        if (m_shooter.isTurretOnTarget(turretToleranceDegrees)
            && m_shooter.isAtSpeed(shotParameters.rpm(), flywheelToleranceRPM)
            && m_shooter.isHoodOnTarget()) {
            m_shooter.runFeeder(shooterConstants.FEEDER_RUN);
        }
        else {
            m_shooter.runFeeder(shooterConstants.FEEDER_OFF);
        }
    }
}