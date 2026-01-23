package frc.robot.autos.modes;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.autos.common.AutoBase;
import frc.robot.commands.ShootWhileMoving;

public class LeftNZRightClimb extends AutoBase{
    private static final Path startPath = PathsBase.LEFT_START_TO_DEPOT;

    public LeftNZRightClimb() {
        // Set Starting Pose
        super(startPath.getPathPlannerPath().getStartingHolonomicPose());
    }

    public void init() {
        addCommands(new ParallelCommandGroup(
            followPathCommand(startPath.getPathPlannerPath()))
            // ShootWhileMoving Auto Command
            // Intake Auto Command
            );
    }
}
