package frc.robot.subsystems.limelightVision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain m_drive;

    // Names of your Limelights as set in the Web UI
    private final String[] kLimelightNames = {"limelight-left", "limelight-right"};

    public VisionSubsystem(CommandSwerveDrivetrain drive) {
        this.m_drive = drive;
    }

    @Override
    public void periodic() {
        Rotation2d gyroYaw = m_drive.getPose().getRotation(); // Or directly from Pigeon

        for (String llName : kLimelightNames) {
            LimelightHelpers.SetRobotOrientation(llName, gyroYaw.getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);

            // Only update if we actually saw tags
            if (mt2.tagCount > 0) {
                double stdDev = calculateStdDev(mt2);
                m_drive.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds,
                    VecBuilder.fill(stdDev, stdDev, 9999999)
                );
            }
        }
    }

    /**
     * Logic to determine how much we trust the Limelight frame
     */
    private double calculateStdDev(LimelightHelpers.PoseEstimate estimate) {
        double stdDev = 0.1; // Set base trust

        if (estimate.tagCount == 1){
            stdDev *= 2.0; // High uncertainty for single tag
        } else if (estimate.tagCount == 2) {
            stdDev *= 0.7; // Moderate uncertainty with 2 tags
        } else if (estimate.tagCount == 3) {
            stdDev *= 0.3; // Low uncertainty with 3 tags
        } else if (estimate.tagCount >= 4) {
            stdDev *= 0.1; // Super low uncertainty with 4+ tags
        } else {
            stdDev *= 10.0; // Super high uncertainty with 0 tags
        }

        // Distance factor increases exponetially with distance
        // f: distance factor
        // d: distance
        // k: coefficient (0.01 for now)
        // f = d^2 * k
        double distanceMultiplier = Math.pow(estimate.avgTagDist,2) * 0.01;
        return stdDev + distanceMultiplier;
    }
}