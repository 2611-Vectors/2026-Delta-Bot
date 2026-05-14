// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.TIP_TO_RPM;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FullSend;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transition;
import frc.robot.util.AutoMath;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RobotTarget extends SequentialCommandGroup {

    /** Creates a new AutoShooterDistance. */
    public RobotTarget(Shooter m_Shooter, Intake m_Intake, FullSend m_FullSend, Transition m_Transition) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        LoggedNetworkNumber dist = new LoggedNetworkNumber("/Shooter/Robot Distance", 0.0);
        double RobotDist = Units.inchesToMeters(dist.get());

        Supplier<Double> shooterSpeed = () -> TIP_TO_RPM * AutoMath.getFuelSpeedToRobot(RobotDist);

        addCommands(new ParallelCommandGroup(
                m_Shooter.setShooterRPM(() -> shooterSpeed.get()),
                m_FullSend.setFullSendRPM(() -> 5000.0),
                m_Transition.setLowerTransitionRPM(() -> 1000.0),
                m_Intake.setIntakeRPM(() -> 500.0)));
    }
}
