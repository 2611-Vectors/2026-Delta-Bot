// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int OPERATOR_CONTROLLER_ID = 1;
    public static final LinearVelocity MAX_DRIVE_SPEED = MetersPerSecond.of(3.25);
  }

  public static class ShooterConstants {
    public static final int LEFT_MOTOR_ID = 43; // 61;
    public static final int LEFT_MOTOR2_ID = 41;
    public static final int RIGHT_MOTOR_ID = 42; // 62;
    public static final int RIGHT_MOTOR2_ID = 44;

    public static final int LEFT_LINEAR_ACTUATOR_ID = 0;
    public static final int RIGHT_LINEAR_ACTUATOR_ID = 1;

    public static final double LINEAR_ACTUATOR_MINIMUM = 0.65;
    public static final double LINEAR_ACTUATOR_MAXIMUM = 1.0;

    public static final double SHOOTER_WHEEL_DIAMETER = Units.inchesToMeters(3.0);
    public static final double TIP_TO_RPM = 425.0;
    public static final double GRAVITATIONAL_CONSTANT = 9.8; // m/s
    public static final double INITIAL_HEIGHT = Units.inchesToMeters(18.33);
    // public static final double LAUNCH_ANGLE = 0.93;
    public static final double LAUNCH_ANGLE = Units.degreesToRadians(90 - 35);
    public static final double LAUNCH_ANGLE_COS = 2 * Math.pow(Math.cos(LAUNCH_ANGLE), 2);

    public static final double MAXIMUM_RPM = 4500.0;
  }

  public static class FullSendConstants {
    public static final int MOTOR_ID = 60;
    public static final double GEAR_RATIO = 1.0 / 4.0;
  }

  public static class IntakeConstants {
    public static final int PIVOT_MOTOR_ID = 32;
    public static final int WHEEL_MOTOR_ID = 31;
    public static final int WHEEL_MOTOR2_ID = 33; // inverted follower of 31

    public static final int PIVOT_ENCODER_ID = 1;
    public static final double PIVOT_ENCODER_OFFSET = -100;

    public static final double PIVOT_GEAR_RATIO = 1.0 / 25.0;
    public static final double INTAKE_GEAR_RATIO = 1.0 / 2.0;

    public static final double PIVOT_ANGLE_TOLERANCE = 1;
    public static final double PIVOT_IN_ANGLE = 3;
    public static final double PIVOT_OUT_ANGLE = 145;
    public static final double PIVOT_RUN_ANGLE = 130;
  }

  public static class TransitionConstants {
    // public static final int UPPER_LEFT_MOTOR_ID = 61;//provides absolutely nothing
    // public static final int UPPER_RIGHT_MOTOR_ID = 62; //provides absolutely nothing
    public static final int LOWER_MOTOR_ID = 52;

    public static final double UPPER_GEAR_RATIO = 1.0 / 3.0;
    public static final double LOWER_GEAR_RATIO = 1.0 / 2.0;
  }

  public static class VisionConstants {
    // Apriltag Field Layout

    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double FIELD_WIDTH = 16.541;
    public static final double FIELD_HEIGHT = 8.069;

    public static final Transform3d groundToRobotTopPlate =
        new Transform3d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            Units.inchesToMeters(1.938) + TunerConstants.FrontLeft.WheelRadius,
            new Rotation3d());

    // Name of PhotonVision Camera
    public static final String LeftRearCam = "Camera6";

    // Position of PhotonVision Camera
    public static final Transform3d robotToLeftRearCam =
        new Transform3d(
            Units.inchesToMeters(-10.5),
            Units.inchesToMeters(12),
            Units.inchesToMeters(21),
            new Rotation3d(Math.toRadians(-2), Math.toRadians(-10), Math.toRadians(180.985)));

    // Name of PhotonVision Camera
    public static final String LeftFrontCam = "Camera5";

    // Position of PhotonVision Camera
    public static final Transform3d robotToLeftFrontCam =
        new Transform3d(
            Units.inchesToMeters(-7.75),
            Units.inchesToMeters(12.25),
            Units.inchesToMeters(21),
            new Rotation3d(0.0, Math.toRadians(-10), Math.toRadians(23.162)));

    // Name of PhotonVision Camera
    public static final String RightRearCam = "Camera4";

    // Position of PhotonVision Camera
    public static final Transform3d robotToRightRearCam =
        new Transform3d(
                Units.inchesToMeters(-11.6875),
                Units.inchesToMeters(-12),
                Units.inchesToMeters(16.837),
                new Rotation3d(0.0, Math.toRadians(-12.5), Math.toRadians(180.985)))
            .plus(groundToRobotTopPlate);

    // Name of PhotonVision Camera
    public static final String RightFrontCam = "Camera3";

    // Position of PhotonVision Camera
    public static final Transform3d robotToRightFrontCam =
        new Transform3d(
            Units.inchesToMeters(-8.625),
            Units.inchesToMeters(-12.25),
            Units.inchesToMeters(20.875),
            new Rotation3d(0.0, Math.toRadians(-8.2), Math.toRadians(-23.162)));

    // Basic filtering thresholds
    public static final double MAX_AMBIGUITY = 0.2;
    public static final double MAX_Z_ERROR = 0.2;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.03; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {1.0};

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static class FieldConstants {
    public static final Pose3d HUB_POSITION =
        new Pose3d(
            Units.inchesToMeters(182.11),
            Units.inchesToMeters(158.84),
            Units.inchesToMeters(70),
            new Rotation3d()); // Change 5.9 to 70" (2" below top of target)
  }

  public static class RobotConstants {
    public static final int PDH_ID = 1;
    public static final double ROBOT_MASS_KG = Units.lbsToKilograms(137.95);
    public static final double ROBOT_MOI = 5.5;
    public static final double WHEEL_COF_FRICTION = 2.255;
    public static final double ROTATION_ERROR = 2.0;
  }
}
