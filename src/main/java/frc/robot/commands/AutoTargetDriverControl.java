// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.FieldConstants.HUB_POSITION;
import static frc.robot.Constants.ShooterConstants.TIP_TO_RPM;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.FullSend;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoMath;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTargetDriverControl extends SequentialCommandGroup {
  /** Creates a new AutoShooterDistance. */
  public AutoTargetDriverControl(
      Drive m_Drive,
      Shooter m_Shooter,
      FullSend m_FullSend,
      Transition m_Transition,
      CommandXboxController m_DriverController) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    LoggedNetworkNumber tip_to_rpm = new LoggedNetworkNumber("/Shooter/Tip To RPM", TIP_TO_RPM);

    Supplier<Double> shooterSpeed =
        () -> tip_to_rpm.get() * AutoMath.getFuelSpeedToTarget(m_Drive.getPose(), HUB_POSITION);
    Supplier<Rotation2d> targetAngle =
        () -> AutoMath.getRobotAngleToTarget(m_Drive.getPose(), HUB_POSITION.toPose2d());
    Supplier<Double> correctedRobotAngle =
        () ->
            (Math.abs(
                m_Drive.getRotation().getDegrees() > 0
                    ? Math.abs(m_Drive.getRotation().getDegrees() - 180.0)
                    : Math.abs(m_Drive.getRotation().getDegrees() + 180.0)));
    Supplier<Double> correctedTargetAngle =
        () ->
            Math.abs(
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                    ? flipAngle(targetAngle.get().getDegrees())
                    : targetAngle.get().getDegrees());
    Supplier<Double> angleError = () -> correctedTargetAngle.get() - correctedRobotAngle.get();

    addCommands(
        new ParallelCommandGroup(
                m_Shooter.setShooterRPM(() -> shooterSpeed.get()),
                DriveCommands.joystickDriveAtAngle(
                    m_Drive,
                    () -> -m_DriverController.getLeftY(),
                    () -> -m_DriverController.getLeftX(),
                    () -> targetAngle.get()),
                Commands.run(
                    () -> {
                      Logger.recordOutput("Targeting/Robot Angle", correctedRobotAngle.get());
                      Logger.recordOutput("Targeting/Target Angle", correctedTargetAngle.get());
                      Logger.recordOutput("Targeting/Angle Error", angleError.get());
                    }))
            .until(
                () -> (m_Shooter.isAtSpeed() && angleError.get() <= RobotConstants.ROTATION_ERROR)),
        new ParallelCommandGroup(
            m_Shooter.setShooterRPM(() -> shooterSpeed.get()),
            DriveCommands.joystickDriveAtAngle(
                m_Drive,
                () -> -m_DriverController.getLeftY(),
                () -> -m_DriverController.getLeftX(),
                () -> targetAngle.get())));
    // new ParallelCommandGroup(
    //                 m_FullSend.manualFullSendRPM(() -> false),
    //                 m_Transition.manualTransitionRPM(() -> false))
    //         .onlyIf(() -> m_DriverController.rightTrigger().getAsBoolean())));
  }

  public static double flipAngle(double angle) {
    double reflectedAngle = -180 - angle;
    if (reflectedAngle < -180) {
      return reflectedAngle + 360;
    }
    return reflectedAngle;
  }
}
