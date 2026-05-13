// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.VisionConstants.FIELD_HEIGHT;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathfindToStart extends SequentialCommandGroup {
  public PathfindToStart(PathPlannerAuto selectedAuto) {
    Pose2d startPose = flipRed(selectedAuto.getStartingPose());
    PathConstraints constraints = new PathConstraints(3.0, 2.5, 90.0, 720.0);

    addCommands(AutoBuilder.pathfindToPose(startPose, constraints), selectedAuto);
  }

  public static Pose2d flipRed(Pose2d point) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      point =
          (new Pose2d(
              FIELD_WIDTH - point.getX(),
              FIELD_HEIGHT - point.getY(),
              Rotation2d.fromDegrees(flipAngle(point.getRotation().getDegrees()))));
    }

    return point;
  }

  public static double flipAngle(double angle) {
    double reflectedAngle = -180 - angle;
    if (reflectedAngle < -180) {
      return reflectedAngle + 360;
    }
    return reflectedAngle;
  }
}
