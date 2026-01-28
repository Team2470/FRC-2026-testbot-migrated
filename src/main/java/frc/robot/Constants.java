package frc.robot;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
        public static final Translation2d BLUE_HUB_LOCATION         = new Translation2d(4.600, 4.025);
        public static final Translation2d RED_HUB_LOCATION          = new Translation2d(12.000, 4.025);
        public static final Translation2d BLUE_RIGHT_PASS_LOCATION  = new Translation2d(3.750, 2.000);
        public static final Translation2d BLUE_LEFT_PASS_LOCATION   = new Translation2d(3.750, 6.000);
        public static final Translation2d RED_LEFT_PASS_LOCATION    = new Translation2d(12.750, 2.000);
        public static final Translation2d RED_RIGHT_PASS_LOCATION   = new Translation2d(12.750, 6.000);
        public static final Translation2d HUB_LOCATION              = isBlueAlliance() ? BLUE_HUB_LOCATION : RED_HUB_LOCATION;
        public static final Translation2d PASS_LEFT_LOCATION        = isBlueAlliance() ? BLUE_LEFT_PASS_LOCATION : RED_LEFT_PASS_LOCATION;
        public static final Translation2d PASS_RIGHT_LOCATION       = isBlueAlliance() ? BLUE_RIGHT_PASS_LOCATION : RED_RIGHT_PASS_LOCATION;
    }

    public static final class shooterConstants {

        public static final MotorAlignmentValue FLYWHEEL_ALIGNMENT_VALUE = MotorAlignmentValue.Opposed;

        // Flywheel Constants
        public static final int FLYWHEEL_1_DEVICE_ID        = 1;
        public static final int FLYWHEEL_2_DEVICE_ID        = 2;
        public static final double FLYWHEEL_KP              = .2;
        public static final double FLYWHEEL_KI              = 0.0;
        public static final double FLYWHEEL_KD              = 0.0;
        public static final double FLYWHEEL_KV              = .125;
        public static final double FLYWHEEL_DIAMETER_METERS = 0.1016;
        public static final double BACKSPIN_DIAMETER_METERS = 0.0508;
        public static final double BACKSPIN_GEAR_RATIO      = 0.5;
        public static final double SHOOTER_EFFICIENCY       = 0.85;


        // Hood Constants
        public static final int LEFT_HOOD_DEVICE_ID     = 3;
        public static final int RIGHT_HOOD_DEVICE_ID    = 4;
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
        public static final double TURRET_GEAR_RATIO                    = 50.0;
        public static final double MIN_TURRET_ANGLE                     = 0.0;
        public static final double MAX_TURRET_ANGLE                     = 180.0;
        public static final double MIN_TURRET_SOFT_LIMIT                = MIN_TURRET_ANGLE /
                                                                            360.0 * TURRET_GEAR_RATIO;
        public static final double MAX_TURRET_SOFT_LIMIT                = MAX_TURRET_ANGLE /
                                                                            360.0 * TURRET_GEAR_RATIO;
        public static final double TURRET_KP                            = 0.01;
        public static final double TURRET_KI                            = 0.0;
        public static final double TURRET_KD                            = 0.0;
        public static final double TURRET_KV                            = 0.01;
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

        // TODO: grab coordinates of Center of Turret compared to our robot's origin point (typically in the center of our bellypan)
        public static final Transform3d ROBOT_TO_TURRET = new Transform3d(-1.0, 0.0, 0.44, Rotation3d.kZero);

        public static InterpolatingDoubleTreeMap HUB_RPM_MAP    = new InterpolatingDoubleTreeMap();
        public static InterpolatingDoubleTreeMap HUB_HOOD_MAP   = new InterpolatingDoubleTreeMap();
        public static InterpolatingDoubleTreeMap HUB_TOF_MAP    = new InterpolatingDoubleTreeMap();
        public static InterpolatingDoubleTreeMap PASS_RPM_MAP   = new InterpolatingDoubleTreeMap();
        public static InterpolatingDoubleTreeMap PASS_HOOD_MAP  = new InterpolatingDoubleTreeMap();
        public static InterpolatingDoubleTreeMap PASS_TOF_MAP   = new InterpolatingDoubleTreeMap();

        public void ShooterInterpolation() {
            // TODO: Actually test for these values.
            // Initial values are based on using desmos Trajectory Calculator
            // And do not reflect real-world-values

            // This map is for the shooter flywheel
            // when we are shooting into hub
            // Distance (meters), Flywheel Speed (RPM)
            // Grabbed from basic test from 1/26 Practice on proto-shooter
            HUB_RPM_MAP.put(2.286, 2940.000);
            HUB_RPM_MAP.put(3.962, 3420.000);
            // Min HUB shot distance (Mid robot - Mid HUB):
            // 0.945 meters -> TODO: TEST THIS VALUE
            // Max HUB shot distance (Mid robot - Mid HUB):
            // 5.600 meters -> TODO: TEST THIS VALUE

            // This map is for the Hood angle
            // when we are shooting into hub
            // Distance (meters), Hood Angle (degrees)
            HUB_HOOD_MAP.put(3.993, 52.000);
            HUB_HOOD_MAP.put(3.048, 67.000);
            HUB_HOOD_MAP.put(2.438, 65.000);
            HUB_HOOD_MAP.put(1.829, 72.000);
            HUB_HOOD_MAP.put(1.219, 80.000);

            // This map is time of flight in seconds
            // when we are shooting into hub
            // Distance (meters), Time of Flight (Seconds)
            HUB_TOF_MAP.put(3.993, 1.900);
            HUB_TOF_MAP.put(3.048, 1.600);
            HUB_TOF_MAP.put(2.438, 1.200);
            HUB_TOF_MAP.put(1.829, 1.000);
            HUB_TOF_MAP.put(1.219, 0.900);

            // This map is for the shooter flywheel
            // when we are passing into alliance zone
            // Distance (meters), Flywheel Speed (RPM)
            PASS_RPM_MAP.put(1.524, 3539.700);
            PASS_RPM_MAP.put(3.048, 4601.610);
            PASS_RPM_MAP.put(6.096, 6902.415);
            PASS_RPM_MAP.put(7.620, 7079.400);
            PASS_RPM_MAP.put(9.144, 6725.430);
            PASS_RPM_MAP.put(11.280,7256.385);
            // Max Pass distance (Mid robot - Pass Location):
            // 12.650 meters -> TODO: TEST THIS VALUE

            // This map is for the Hood angle
            // when we are passing into alliance zone
            // Distance (meters), Hood Angle (degrees)
            PASS_HOOD_MAP.put(1.524, 40.0);
            PASS_HOOD_MAP.put(3.048, 30.0);
            PASS_HOOD_MAP.put(6.096, 18.0);
            PASS_HOOD_MAP.put(7.620, 23.0);
            PASS_HOOD_MAP.put(9.144, 30.0);
            PASS_HOOD_MAP.put(11.280, 35.0);

            // This map is time of flight in seconds
            // when we are passing into alliance zone
            // Distance (meters), Time of Flight (Seconds)
            PASS_TOF_MAP.put(1.524, 1.000);
            PASS_TOF_MAP.put(3.048, 1.200);
            PASS_TOF_MAP.put(6.096, 1.400);
            PASS_TOF_MAP.put(7.620, 1.600);
            PASS_TOF_MAP.put(9.144, 1.900);
            PASS_TOF_MAP.put(11.280, 2.400);
        }
    }

    public static final class DashboardConstants {
        public static final String DRIVE_MODE_KEY = "Drive Mode";
        public static final String AUTO_COMPILED_KEY = "Auto Compiled";
        public static final String AUTO_DESCRIPTION_KEY = "Auto Description";
        public static final String WAIT_SECONDS_SAVED_KEY = "Wait Seconds Saved";
        public static final String WAIT_SECONDS_DISPLAY_KEY = "Wait Seconds Display";
  }
}
