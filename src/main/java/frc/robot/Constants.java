package frc.robot;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;

public class Constants {
    // TODO:
    // Things to make constants
    // Tolerance for pass vs scoring shots in RPM and degrees
    // Translation2d of where to pass vs where to shoot
    public static final double SECONDS_PER_MINUTE = 60.0;

    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().isEmpty()
                || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }

    public static final class fieldConstants {
        // Target Locations
        public static final Translation2d BLUE_HUB_LOCATION             = new Translation2d(4.600, 4.025);
        public static final Translation2d RED_HUB_LOCATION              = new Translation2d(12.000, 4.025);
        public static final Translation2d BLUE_RIGHT_PASS_LOCATION      = new Translation2d(3.750, 2.000);
        public static final Translation2d BLUE_LEFT_PASS_LOCATION       = new Translation2d(3.750, 6.000);
        public static final Translation2d RED_LEFT_PASS_LOCATION        = new Translation2d(12.750, 2.000);
        public static final Translation2d RED_RIGHT_PASS_LOCATION       = new Translation2d(12.750, 6.000);
        public static final Translation2d HUB_LOCATION                  = isBlueAlliance() ? BLUE_HUB_LOCATION :
                                                                                                RED_HUB_LOCATION;
        public static final Translation2d PASS_LEFT_LOCATION            = isBlueAlliance() ? BLUE_LEFT_PASS_LOCATION :
                                                                                                RED_LEFT_PASS_LOCATION;
        public static final Translation2d PASS_RIGHT_LOCATION           = isBlueAlliance() ? BLUE_RIGHT_PASS_LOCATION :
                                                                                                RED_RIGHT_PASS_LOCATION;
    }

    public static final class shooterConstants {

        public static final MotorAlignmentValue FLYWHEEL_ALIGNMENT_VALUE = MotorAlignmentValue.Opposed;

        // Flywheel Constants
        public static final int FLYWHEEL_1_DEVICE_ID                    = 1;
        public static final int FLYWHEEL_2_DEVICE_ID                    = 2;
        public static final double FLYWHEEL_KP                          = .2;
        public static final double FLYWHEEL_KI                          = 0.0;
        public static final double FLYWHEEL_KD                          = 0.0;
        public static final double FLYWHEEL_KV                          = .125;
        public static final double FLYWHEEL_DIAMETER_METERS             = 0.1016;
        public static final double BACKSPIN_DIAMETER_METERS             = 0.0508;
        public static final double BACKSPIN_GEAR_RATIO                  = 0.5;
        public static final double SHOOTER_EFFICIENCY                   = 0.85;

        // Hood Constants
        public static final int LEFT_HOOD_DEVICE_ID                     = 3;
        public static final int RIGHT_HOOD_DEVICE_ID                    = 4;
        public static final double HOOD_LENGTH_TO_ANGLE_RATIO           = 50.0; // Servo has position from 0 to 1, we need to convert that to the full range of motion in degrees
        public static final double MIN_HOOD_ANGLE                       = 15.0;
        public static final double MAX_HOOD_ANGLE                       = 85.0;
        public static final double MIN_HOOD_SOFT_LIMIT                  = MIN_HOOD_ANGLE / 360.0 * HOOD_LENGTH_TO_ANGLE_RATIO;
        public static final double MAX_HOOD_SOFT_LIMIT                  = MAX_HOOD_ANGLE / 360.0 * HOOD_LENGTH_TO_ANGLE_RATIO;
        public static final double HOOD_KP                              = 20.0;
        public static final double HOOD_KI                              = 0.0;
        public static final double HOOD_KD                              = 0.0;

        // Turret Constants
        public static final int TURRET_DEVICE_ID                        = 0;
        public static final double TURRET_GEAR_RATIO                    = 50.0;
        // Even thought the Turret is designed to have a 360 degree range of motion
        // set min/max points short to avoid roll-over issues
        public static final double MIN_TURRET_ANGLE                     =   0.500;
        public static final double MAX_TURRET_ANGLE                     = 359.500;
        public static final double MIN_TURRET_SOFT_LIMIT                = MIN_TURRET_ANGLE /
                                                                            360.0 * TURRET_GEAR_RATIO;
        public static final double MAX_TURRET_SOFT_LIMIT                = MAX_TURRET_ANGLE /
                                                                            360.0 * TURRET_GEAR_RATIO;
        public static final double TURRET_KP                            = 0.15;
        public static final double TURRET_KI                            = 0.0;
        public static final double TURRET_KD                            = 0.0;
        public static final double TURRET_KV                            = 0.105;
        public static final double TURRET_MOTION_MAGIC_CRUISE_VELOCITY  = 80.0;
        public static final double TURRET_MOTION_MAGIC_ACCELERACTIION   = 160.0;

        // Feeder Constants
        public static final int FEEDER_DEVICE_ID                        = 11;
        public static final double FEEDER_RUN                           = 1.0;
        public static final double FEEDER_OFF                           = 0.0;

        // Tolerances
        // TODO: Run tests to determine what level of tolerance gives
        // "the best" balance of speed and accuracy
        public static final double TURRET_PASS_TOLERANCE                = 3.0;
        public static final double TURRET_HUB_TOLERANCE                 = 1.0;
        public static final double HOOD_PASS_TOLERANCE                  = 3.0;
        public static final double HOOD_HUB_TOLERANCE                   = 1.0;
        public static final double RPM_PASS_TOLERANCE                   = 300;
        public static final double RPM_HUB_TOLERANCE                    = 100;

        // TODO: grab coordinates of Center of Turret compared to our robot's origin point (typically in the center of our bellypan)
        public static final Transform3d ROBOT_TO_TURRET                 = new Transform3d(1.0, 0.0, 0.44, Rotation3d.kZero);

        // Record to easily store parameters for shoot on the move
        public record SHOOTER_PARAMETERS(double rpm, double hoodAngle, double timeOfFlight){}

        public static InterpolatingTreeMap<Double, SHOOTER_PARAMETERS> HUB_MAP  = new InterpolatingTreeMap<Double, SHOOTER_PARAMETERS>(null, null);
        public static InterpolatingTreeMap<Double, SHOOTER_PARAMETERS> PASS_MAP = new InterpolatingTreeMap<Double, SHOOTER_PARAMETERS>(null, null);

        public void ShooterInterpolation() {
            // TODO: Get good values for passing
            PASS_MAP.put( 1.000, new SHOOTER_PARAMETERS(1500.000, 30.000, 0.800));
            PASS_MAP.put( 2.000, new SHOOTER_PARAMETERS(2000.000, 30.000, 0.900));
            // PASS_MAP.put( 1.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put( 2.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put( 3.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put( 4.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put( 5.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put( 6.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put( 7.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put( 8.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put( 9.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put(10.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put(11.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put(12.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // PASS_MAP.put(13.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));

            // TODO: Get good values for the HUB
            // Only values tested for so far: rpm at 7.5 and 13 feet (2.286 and 3.962 meters respectively)
            //                                HOOD_ANGLE AND TIME_OF_FLIGHT NOT TESTED FOR
            // Distance from front of shooter to front of HUB
            HUB_MAP.put(2.286, new SHOOTER_PARAMETERS(2940.000, 30.000, 1.100));
            HUB_MAP.put(3.962, new SHOOTER_PARAMETERS(3420.000, 30.000, 1.900));
            // 0.500 Meters ( 1.640 feet)
            // HUB_MAP.put(0.500, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 1.000 Meters ( 3.281 feet)
            // HUB_MAP.put(1.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 1.500 Meters ( 4.921 feet)
            // HUB_MAP.put(1.500, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 2.000 Meters ( 6.562 feet)
            // HUB_MAP.put(2.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 2.500 Meters ( 8.202 feet)
            // HUB_MAP.put(2.500, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 3.000 Meters ( 9.843 feet)
            // HUB_MAP.put(3.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 3.500 Meters (11.483 feet)
            // HUB_MAP.put(3.500, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 4.000 Meters (13.123 feet)
            // HUB_MAP.put(4.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 4.500 Meters (14.764 feet)
            // HUB_MAP.put(4.500, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 5.000 Meters (16.404 feet)
            // HUB_MAP.put(5.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 5.500 Meters (18.045 feet)
            // HUB_MAP.put(5.500, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
            // 6.000 Meters (19.685 feet)
            // Just past real max value of shooting inside alliance zone
            // HUB_MAP.put(6.000, new SHOOTER_PARAMETERS(RPM, HOOD_ANGLE, TIME_OF_FLIGHT));
        }
    }

    public static final class DashboardConstants {
        public static final String DRIVE_MODE_KEY           = "Drive Mode";
        public static final String AUTO_COMPILED_KEY        = "Auto Compiled";
        public static final String AUTO_DESCRIPTION_KEY     = "Auto Description";
        public static final String WAIT_SECONDS_SAVED_KEY   = "Wait Seconds Saved";
        public static final String WAIT_SECONDS_DISPLAY_KEY = "Wait Seconds Display";
  }
}
