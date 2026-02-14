package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CalibrateCameraOffset extends Command {
    private final CommandSwerveDrivetrain m_drive;
    private final String m_limelightName;
    private final Pose2d m_groundTruth;

    /**
     * @param drive The drivetrain
     * @param limelightName The camera to calibrate
     * @param groundTruth The EXACT Pose2d where the robot is physically sitting
     */
    public CalibrateCameraOffset(CommandSwerveDrivetrain drive, String limelightName, Pose2d groundTruth) {
        this.m_drive = drive;
        this.m_limelightName = limelightName;
        this.m_groundTruth = groundTruth;
    }

    @Override
    public void initialize() {
        // Get what the camera currently THINKS the robot pose is
        // We use the raw BotPose (not MegaTag2) for initial mounting calibration
        Pose2d cameraReportedPose = LimelightHelpers.getBotPose2d_wpiBlue(m_limelightName);

        // Calculate the error (The "Delta")
        // This represents how much your mounting config in the Limelight UI is off
        Transform2d error = new Transform2d(cameraReportedPose, m_groundTruth);

        // Output the suggested adjustment to the Dashboard
        updateSmartDashboard(m_limelightName, error);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    private void updateSmartDashboard(String name, Transform2d error) {
        String prefix = "LimelightCal/" + name + "/";

        // Put raw numbers for precise reading/logging
        SmartDashboard.putNumber(prefix + "X Adjustment (m)", error.getX());
        SmartDashboard.putNumber(prefix + "Y Adjustment (m)", error.getY());
        SmartDashboard.putNumber(prefix + "Yaw Adjustment (deg)", error.getRotation().getDegrees());

        // Create a "Human Readable" summary string
        String summary = String.format(
            "X: %+.5f m | Y: %+.5f m | Yaw: %+.5f°",
            error.getX(),   // Positive X Error: Tell Limelight it is mounted further Forward
            error.getY(),   // Positive Y Error: Tell Limelight it is mounted further to the Left (looking from back of bot towards front)
            error.getRotation().getDegrees()
        );
        SmartDashboard.putString(prefix + "Summary", summary);
        // Optional: Add a timestamp so you know it actually updated
        SmartDashboard.putNumber(prefix + "Last Update", Timer.getFPGATimestamp());
    }
}