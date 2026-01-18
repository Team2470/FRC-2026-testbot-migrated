package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;

public class Constants {
    // TODO:
    // Things to make constants
    // Tolerance for pass vs scoring shots in RPM and degrees
    // Translation2d of where to pass vs where to shoot
    public static final double MINUTE_TO_SECONDS = 60.0;

    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().isEmpty()
                || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }

    public static final class fieldConstants {
        // Target Locations
        public static final Translation2d BLUE_HUB_LOCATION     = new Translation2d(4.600, 4.025);
        public static final Translation2d RED_HUB_LOCATION      = new Translation2d(12.000, 4.025);
        public static final Translation2d BLUE_PASS_LOCATION    = new Translation2d(2.000, 2.500);
        public static final Translation2d RED_PASS_LOCATION     = new Translation2d(15.000, 2.500);
        public static final Translation2d HUB_LOCATION          = isBlueAlliance() ? BLUE_HUB_LOCATION : RED_HUB_LOCATION;
        public static final Translation2d PASS_LOCATION         = isBlueAlliance() ? BLUE_PASS_LOCATION : RED_PASS_LOCATION;
    }

    public static final class shooterConstants {
        // Flywheel Constants
        public static final int FLYWHEEL_1_DEVICE_ID        = 10;
        // My (Josh) guess is that we will use 2 motors on the flywheel
        // public static final int FLYWHEEL_2_DEVICE_ID        = 10;
        public static final double FLYWHEEL_KP              = 10.0;
        public static final double FLYWHEEL_KI              = 0.0;
        public static final double FLYWHEEL_KD              = 0.0;
        public static final double FLYWHEEL_KV              = 10.0;
        public static final double FLYWHEEL_DIAMETER_METERS = 0.1016;
        public static final double BACKSPIN_DIAMETER_METERS = 0.0508;
        public static final double BACKSPIN_GEAR_RATIO      = 0.5;
        public static final double SHOOTER_EFFICIENCY       = 0.85;

        // Hood Constants
        public static final int HOOD_DEVICE_ID          = 13;
        public static final double HOOD_GEAR_RATIO      = 50.0;
        public static final double MIN_HOOD_ANGLE       = 15.0;
        public static final double MAX_HOOD_ANGLE       = 85.0;
        public static final double MIN_HOOD_SOFT_LIMIT  = MIN_HOOD_ANGLE / 360.0 * HOOD_GEAR_RATIO; 
        public static final double MAX_HOOD_SOFT_LIMIT  = MAX_HOOD_ANGLE / 360.0 * HOOD_GEAR_RATIO;
        public static final double HOOD_KP              = 20.0; 
        public static final double HOOD_KI              = 0.0; 
        public static final double HOOD_KD              = 0.0;

        // Turret Constants
        public static final int TURRET_DEVICE_ID                        = 14;
        public static final double TURRET_GEAR_RATIO                    = 100.0;
        public static final double MIN_TURRET_ANGLE                     = 0.0;
        public static final double MAX_TURRET_ANGLE                     = 180.0;
        public static final double MIN_TURRET_SOFT_LIMIT                = MIN_TURRET_ANGLE /
                                                                            360.0 * TURRET_GEAR_RATIO; 
        public static final double MAX_TURRET_SOFT_LIMIT                = MAX_TURRET_ANGLE /
                                                                            360.0 * TURRET_GEAR_RATIO;
        public static final double TURRET_KP                            = 20.0; 
        public static final double TURRET_KI                            = 0.0; 
        public static final double TURRET_KD                            = 0.0;
        public static final double TURRET_MOTION_MAGIC_CRUISE_VELOCITY  = 80.0;
        public static final double TURRET_MOTION_MAGIC_ACCELERACTIION   = 160.0;

        // Feeder Constants
        public static final int FEEDER_DEVICE_ID    = 11;
        public static final double FEEDER_RUN       = 1.0;
        public static final double FEEDER_OFF       = 0.0;

        // Tolerances
        // TODO: Run tests to determine what level of tolerance gives
        // "the best" balance of speed and accuracy
        public static final double TURRET_PASS_TOLERANCE    = 3.0;
        public static final double TURRET_HUB_TOLERANCE     = 1.0;
        public static final double HOOD_PASS_TOLERANCE      = 3.0;
        public static final double HOOD_HUB_TOLERANCE       = 1.0;
        public static final double RPM_PASS_TOLERANCE       = 300;
        public static final double RPM_HUB_TOLERANCE        = 100;

        public static InterpolatingDoubleTreeMap HUB_RPM_MAP    = new InterpolatingDoubleTreeMap();
        public static InterpolatingDoubleTreeMap PASS_RPM_MAP   = new InterpolatingDoubleTreeMap();
        public static InterpolatingDoubleTreeMap HOOD_HUB_MAP   = new InterpolatingDoubleTreeMap();
        public static InterpolatingDoubleTreeMap HOOD_PASS_MAP  = new InterpolatingDoubleTreeMap();

        public void ShooterInterpolation() {
            // TODO: Actually test for these values.
            // Initial values are based on using desmos Trajectory Calculator
            // And do not reflect real-world-values

            // This map is for the Hood angle 
            // when we are shooting into hub
            // Distance (meters), Hood Angle (degrees)
            HOOD_HUB_MAP.put(3.993, 52.000); 
            HOOD_HUB_MAP.put(3.048, 67.000);
            HOOD_HUB_MAP.put(2.4384, 65.000);
            HOOD_HUB_MAP.put(1.829, 72.000);
            HOOD_HUB_MAP.put(1.219, 80.000);
        
            // This map is for the Hood angle 
            // when we are passing into alliance zone
            // Distance (meters), Hood Angle (degrees)
            HOOD_PASS_MAP.put(1.524, 40.0); 
            HOOD_PASS_MAP.put(3.048, 30.0);
            HOOD_PASS_MAP.put(6.096, 18.0);
            HOOD_PASS_MAP.put(7.620, 23.0);
            HOOD_PASS_MAP.put(9.144, 30.0);
            HOOD_PASS_MAP.put(11.280, 35.0);
        
            // This map is for the shooter flywheel
            // when we are shooting into hub
            // Distance (meters), Flywheel Speed (RPM)
            HUB_RPM_MAP.put(3.993, 4247.640);
            HUB_RPM_MAP.put(3.048, 4070.655);
            HUB_RPM_MAP.put(2.4384, 3716.685);
            HUB_RPM_MAP.put(1.829, 3539.700);
            HUB_RPM_MAP.put(1.219, 3716.685);
        
            // This map is for the shooter flywheel
            // when we are passing into alliance zone
            // Distance (meters), Flywheel Speed (RPM)
            PASS_RPM_MAP.put(1.524, 3539.700);
            PASS_RPM_MAP.put(3.048, 4601.610);
            PASS_RPM_MAP.put(6.096, 6902.415);
            PASS_RPM_MAP.put(7.620, 7079.400);
            PASS_RPM_MAP.put(9.144, 6725.430);
            PASS_RPM_MAP.put(11.280,7256.385);
        }
    }
}
