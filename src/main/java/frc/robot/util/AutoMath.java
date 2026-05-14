// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static frc.robot.Constants.ShooterConstants.GRAVITATIONAL_CONSTANT;
import static frc.robot.Constants.ShooterConstants.INITIAL_HEIGHT;
import static frc.robot.Constants.ShooterConstants.LAUNCH_ANGLE;
import static frc.robot.Constants.ShooterConstants.LAUNCH_ANGLE_COS;
import static frc.robot.Constants.VisionConstants.FIELD_HEIGHT;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commands.PathfindToStart;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class AutoMath {
    public AutoMath() {}

    public static Rotation2d getRobotAngleToTarget(Pose2d robotPose, Pose2d target) {
        robotPose = flipRed(robotPose);
        double a = robotPose.getX() - target.getX();
        double b = robotPose.getY() - target.getY();

        return new Rotation2d(
                Math.atan(b / a) + ((DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) ? 0 : Math.PI));
    }

    public static double getDistanceToTarget(Pose2d robotPose, Pose2d target) {
        robotPose = PathfindToStart.flipRed(robotPose);
        double a = robotPose.getX() - target.getX();
        double b = robotPose.getY() - target.getY();
        double c = Math.sqrt((a * a) + (b * b));

        Logger.recordOutput("Targeting/distToHub", c);
        return c;
    }

    public static Pose3d translateTargetByChassisSpeeds(Pose2d robotPose, Pose3d target, ChassisSpeeds speeds) {
        double dist = getDistanceToTarget(robotPose, target.toPose2d());
        double speed = getFuelSpeedToTarget(robotPose, target);
        double time = dist / (speed * Math.cos(LAUNCH_ANGLE));

        Pose2d rotVector = new Pose2d(dist, 0, new Rotation2d());
        rotVector.rotateAround(new Translation2d(0, 0), new Rotation2d(speeds.omegaRadiansPerSecond * time));

        Pose3d out = new Pose3d(
                target.getX() + Math.abs(speeds.vxMetersPerSecond * time) + rotVector.getX(),
                target.getY() + Math.abs(speeds.vyMetersPerSecond * time) + rotVector.getY(),
                target.getZ(),
                target.getRotation());

        Logger.recordOutput("Targeting/Target", target);
        Logger.recordOutput("Targeting/Target New", out);
        return out;
    }

    public static double getFuelSpeedToTarget(Pose2d robotPose, Pose3d target) {
        double dist = getDistanceToTarget(robotPose, target.toPose2d());
        double heightDiff = target.getZ() - INITIAL_HEIGHT;
        return Math.sqrt((GRAVITATIONAL_CONSTANT * (dist * dist))
                / (LAUNCH_ANGLE_COS * (Math.tan(LAUNCH_ANGLE) * dist - heightDiff)));
    }

    public static double getFuelSpeedToRobot(double dist) {
        return Math.sqrt(
                (GRAVITATIONAL_CONSTANT * (dist * dist)) / (LAUNCH_ANGLE_COS * (Math.tan(LAUNCH_ANGLE) * dist)));
    }

    public static Pose2d flipRed(Pose2d point) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            point = new Pose2d(
                    FIELD_WIDTH - point.getX(),
                    FIELD_HEIGHT - point.getY(),
                    Rotation2d.fromDegrees(flipAngle(point.getRotation().getDegrees())));
        }

        return point;
    }

    public static Rotation2d flipRed(Rotation2d angle) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
            angle = Rotation2d.fromDegrees(flipAngle(angle.getDegrees()));

        return angle;
    }

    public static double flipAngle(double angle) {
        double reflectedAngle = -180 - angle;
        if (reflectedAngle < -180) {
            return reflectedAngle + 360;
        }
        return reflectedAngle;
    }
}
