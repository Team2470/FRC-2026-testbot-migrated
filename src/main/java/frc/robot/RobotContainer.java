// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ShootWhileMoving;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.limelightVision.LimelightHelpers;
import frc.robot.subsystems.limelightVision.VisionApriltagSubsystem;
import frc.robot.util.FieldObject;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private Rotation2d turretAngle = Rotation2d.fromRadians(0);
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    // private QuestNav questNav = new QuestNav();
    // public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain((pose) -> questNav.resetPose(pose));
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();

    StructPublisher<Pose2d> posePublisher =
        NetworkTableInstance.getDefault().getStructTopic("robotPose", Pose2d.struct).publish();
    StructPublisher<Pose2d> questPosePublisher =
        NetworkTableInstance.getDefault().getStructTopic("questPose", Pose2d.struct).publish();
    private VisionApriltagSubsystem visionApriltagSubsystem;

    private final SendableChooser<Pose2d> calibrationChooser = new SendableChooser<>();

    public RobotContainer() {
        // Define a few "Gold Standard" spots on your carpet
        calibrationChooser.setDefaultOption("Blue Hub 1m out", new Pose2d(3.02, 4.03, Rotation2d.fromDegrees(0)));
        calibrationChooser.addOption("Red Hub 1m out", new Pose2d(13.51, 4.03, Rotation2d.fromDegrees(180)));
        SmartDashboard.putData("Calibration Points", calibrationChooser);


        configureBindings();
    }

    /*  public void periodic() {
        Hood.periodic();
    } Is this nessesary if there's a periodic in the Hood class? */

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * 0.25*MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * 0.25*MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * 0.25*MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));


        //joystick.x().whileTrue(Commands.runOnce(() -> linearServo.extendActuator()));
        //joystick.y().whileTrue(Commands.runOnce(() -> linearServo.retractActuator()));
        // uncomment these for motor test and comment the ones above, vice versa to test linear actuator
        joystick.x().whileTrue(intake.intakeCommand());
        joystick.y().whileTrue(intake.outtakeCommand());

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // joystick.leftBumper().whileTrue(turret.runTurretCommand(1));
        // joystick.rightBumper().whileTrue(turret.runTurretCommand(-1));
        joystick.rightTrigger().whileTrue(shooter.runShooterCommand());
        drivetrain.registerTelemetry(logger::telemeterize);

        // Shoot at Hub while moving
        joystick.rightTrigger()
            .whileTrue(new ShootWhileMoving(drivetrain, shooter, false));

        // Pass to Alliance zone while moving
        joystick.leftTrigger()
            .whileTrue(new ShootWhileMoving(drivetrain, shooter, true));

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void autonomousInit() {
        // Reset questNav pose right at being of auto
        // Grab reset pose from Limelights
        // at least one AprilTag would need to be
        // in view in starting position
        updateVisionPose();
        Pose2d startPose = extractLimelightPose();
        // questNav.resetPose(startPose);
    }

    public void periodic () {
        // questNav.cleanUpQuestNavMessages();
        posePublisher.set(drivetrain.getPose());
        updateVisionPose();
        // questPosePublisher.set(questNav.getRobotPose());
    }

    public void updateVisionPose() {
        // if (questNav.isConnected()) {
            // questNav.updateAverageRobotPose();
            //   drivetrain.addVisionMeasurement(
            //       questNav.getRobotPose(), VecBuilder.fill(0.0, 0.0, 9999999.0));
            // drivetrain.addVisionMeasurement(
                // questNav.getAverageRobotPose(), VecBuilder.fill(0.0, 0.0, 0.0));
            // return;
        // }

        // LimelightHelpers.PoseEstimate limelightMeasurement =
        // visionApriltagSubsystem.getPoseEstimate();
        // if (limelightMeasurement.tagCount >= 2
        //     || (limelightMeasurement.tagCount == 1 && limelightMeasurement.avgTagDist < 1.25)) {
        //   drivetrain.addVisionMeasurement(
        //       limelightMeasurement.pose,
        //       limelightMeasurement.timestampSeconds,
        //       VecBuilder.fill(.6, .6, 9999999));
        }

    private Pose2d extractLimelightPose() {
        LimelightHelpers.PoseEstimate limelightMeasurement = visionApriltagSubsystem.getPoseEstimate();
        if (limelightMeasurement.tagCount >= 2
                || (limelightMeasurement.tagCount == 1 && limelightMeasurement.avgTagDist < 1.25)) {
            return limelightMeasurement.pose;
        }
        return null;
    }
}
