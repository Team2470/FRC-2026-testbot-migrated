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
        double tagStdDev;

        switch(estimate.tagCount){
            // case 0 shouldn't happen
            // Only call calculateStdDev when estimate.tagCount > 0
            case 0:
                tagStdDev = 100;    // Super High uncertainty with no tags
                return tagStdDev;   // No tags, so no estimate.avgTagDist
            case 1:
                tagStdDev = 2.0;    // High uncertainty for single tag
                break;
            case 2:
                tagStdDev = 0.5;    // Moderate uncertainty with 2 tags
                break;
            case 3:
                tagStdDev = 0.1;    // Low uncertainty with 3 tags
                break;
            case 4:
                tagStdDev = 0.001;  // Super Low uncertainty with 4 tags
                break;
            default:
                tagStdDev = 0.0001; // Super-Duper-Low uncertainty with 5+ tags
                break;
        }

        // Distance factor increases exponentially with distance
        // f: distance factor
        // d: distance
        // k: coefficient (0.1 for now)
        // f = d^2 * k
        // 1 meter average tag distance:    f =  1^2 * .1 =  0.1
        // 5 meter average tag distance:    f =  5^2 * .1 =  2.5
        // 10 meter average tag distance:   f = 10^2 * .1 = 10.0
        double distanceMultiplier = Math.pow(estimate.avgTagDist,2) * 0.1;

        // stdDev to use will be the product of the tag dev * distance dev
        // Examples:
        // 1t at  1m: 2.0 * 0.1 = 0.2
        // 3t at  3m: 0.1 * 0.9 = 0.9
        // 5t at 10m: 0.0001 * 10.0 = 0.001
        return tagStdDev * distanceMultiplier;
    }
}