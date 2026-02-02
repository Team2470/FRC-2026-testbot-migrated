package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;

public class Hood extends SubsystemBase {
    private final Servo leftServo;
    private final Servo rightServo;

    private double currentPosition  = 0.5;
    private double targetPosition   = 0.5;
    private Time lastUpdateTime = Seconds.of(0);

    public Hood() {
        leftServo   = new Servo(shooterConstants.LEFT_HOOD_DEVICE_ID);
        rightServo  = new Servo(shooterConstants.RIGHT_HOOD_DEVICE_ID);
        leftServo.setBoundsMicroseconds(2000, 1800,
                                    1500, 1200, 1000);
        rightServo.setBoundsMicroseconds(2000, 1800,
                                    1500, 1200, 1000);
        setPosition(currentPosition);
        SmartDashboard.putData(this);
    }

    /** Expects a position between 0.0 and 1.0 */
    public void setPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position,
                                                        shooterConstants.MIN_HOOD_POSITION,
                                                        shooterConstants.MAX_HOOD_POSITION);
        leftServo.set(clampedPosition);
        rightServo.set(clampedPosition);
        targetPosition = clampedPosition;
    }

    public void setAngle(double degrees) {
        double clampedDegrees = MathUtil.clamp(degrees,
                                                shooterConstants.MIN_HOOD_ANGLE,
                                                shooterConstants.MAX_HOOD_ANGLE);

        // This math assumes that extending the hood servos and resulting launch angle is a linear relationship
        // ratio is percentage the requested angle is in the range the hood can accomplish
        // Example:
        // Min Hood Angle:  25 degrees
        // Max Hood Angle:  75 degrees
        // Requested Angle: 45 degrees
        //     20          50
        // (45 - 25) / (75 - 25) = .40
        // 1 - 0.40 = 0.60 <- This step is required since Max Hood Angle = Min Hood Position
        double ratio = 1 - ((clampedDegrees - shooterConstants.MIN_HOOD_ANGLE) /
                        (shooterConstants.MAX_HOOD_ANGLE - shooterConstants.MIN_HOOD_ANGLE));
        double position = 1 - ratio;
        setPosition(position);
    }

    /** Expects a position between 0.0 and 1.0 */
    public Command positionCommand(double position) {
        return runOnce(() -> setPosition(position))
            .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(targetPosition, currentPosition, shooterConstants.HOOD_TOLERANCE);
    }

    private void updateCurrentPosition() {
        final Time currentTime = Seconds.of(Timer.getFPGATimestamp());
        final Time elapsedTime = currentTime.minus(lastUpdateTime);
        lastUpdateTime = currentTime;

        if (isPositionWithinTolerance()) {
            currentPosition = targetPosition;
            return;
        }

        final Distance maxDistanceTraveled = shooterConstants.HOOD_SPEED.times(elapsedTime);
        final double maxPercentageTraveled = maxDistanceTraveled.div(shooterConstants.HOOD_LENGTH).in(Value);
        currentPosition = targetPosition > currentPosition
            ? Math.min(targetPosition, currentPosition + maxPercentageTraveled)
            : Math.max(targetPosition, currentPosition - maxPercentageTraveled);
    }

    @Override
    public void periodic() {
        updateCurrentPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Current Position", () -> currentPosition, null);
        builder.addDoubleProperty("Target Position", () -> targetPosition, value -> setPosition(value));
    }

    public double getPosition(){
        return currentPosition;
    }
}