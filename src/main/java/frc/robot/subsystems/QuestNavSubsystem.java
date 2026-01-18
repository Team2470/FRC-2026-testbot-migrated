package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain; // Adjust path based on your repo
import frc.robot.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
    private final QuestNav m_questNav;
    private final CommandSwerveDrivetrain m_drive;
    private final DoubleArraySubscriber m_poseSub;

    /** * The physical location of the Quest 3S on your robot. 
     * X is forward, Y is left, Z is up. 
     */
    private final Transform3d ROBOT_TO_QUEST = new Transform3d(
        new Translation3d(0.15, 0.0, 0.5), // Example: 15cm forward, 50cm up
        new Rotation3d(0.0, 0.0, 0.0)      // Level and facing forward
    );

    public QuestNavSubsystem(CommandSwerveDrivetrain drive) {
        this.m_drive = drive;
        this.m_questNav = new QuestNav();
        var table = NetworkTableInstance.getDefault().getTable("QuestNav");
        this.m_poseSub = table.getDoubleArrayTopic("pose").subscribe(new double[] {0, 0, 0});
    }

    @Override
    public void periodic() {
        double[] data = m_poseSub.get(new double[] {0, 0, 0});
        // QuestNav updates are usually high frequency; we get the latest
        Pose2d visionPose = new Pose2d(data[0], data[1], Rotation2d.fromDegrees(data[2]));
        
        // Add to CTRE Pose Estimator
        m_drive.addVision(visionPose, Timer.getFPGATimestamp());
    }

    /**
     * Call this at the start of Auto or via a button to align 
     * the Quest's (0,0,0) with the Field's actual starting position.
     */
    public void resetToPose(Pose2d fieldPose) {
        m_questNav.resetPose(fieldPose);
    }
}