package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.VectorKit.hardware.KrakenX60;
import frc.robot.VectorKit.tuners.PidTuner;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shooter extends SubsystemBase {
  private final KrakenX60 leftMotor = new KrakenX60(ShooterConstants.LEFT_MOTOR_ID);
  private final KrakenX60 leftMotor2 = new KrakenX60(ShooterConstants.LEFT_MOTOR2_ID);
  private final KrakenX60 rightMotor = new KrakenX60(ShooterConstants.RIGHT_MOTOR_ID);
  private final KrakenX60 rightMotor2 = new KrakenX60(ShooterConstants.RIGHT_MOTOR2_ID);

  PidTuner shooterPidTuner = new PidTuner("/Shooter/", 0.6, 0.0, 0.0, 0.0, 0.12);

  LoggedNetworkNumber manualRPM = new LoggedNetworkNumber("/Shooter/Target RPM", 2900.0);

  public Shooter() {
    leftMotor.setFollower(rightMotor2, MotorAlignmentValue.Aligned);
    leftMotor.setFollower(leftMotor2, MotorAlignmentValue.Opposed);
    leftMotor.setFollower(rightMotor, MotorAlignmentValue.Opposed);

    leftMotor.setInverted(InvertedValue.Clockwise_Positive);
    leftMotor.setStatorCurrentLimit(40);
    leftMotor2.setStatorCurrentLimit(40);
    leftMotor2.setStatorCurrentLimit(-40);
    rightMotor.setStatorCurrentLimit(40);
    rightMotor2.setStatorCurrentLimit(40);
  }

  public Command setShooterRPM(Supplier<Double> rpm) {
    return run(() -> {
          double rpmActual = rpm.get();
          if (rpm.get() > ShooterConstants.MAXIMUM_RPM) rpmActual = ShooterConstants.MAXIMUM_RPM;

          manualRPM.set(rpmActual);
          leftMotor.setVelocity(rpmActual, RPM);
          Logger.recordOutput("Shooter/On", true);
        })
        .handleInterrupt(
            () -> {
              leftMotor.set(0.0);
              Logger.recordOutput("Shooter/On", false);
            });
  }

  public Command manualShooterRPM() {
    return run(() -> {
          leftMotor.setVelocity(manualRPM.get(), RPM);
        })
        .handleInterrupt(
            () -> {
              leftMotor.set(0.0);
            });
  }

  public Boolean isAtSpeed() {
    boolean atSpeed =
        Math.abs(
                RPM.convertFrom(leftMotor.getVelocity().getValueAsDouble(), RotationsPerSecond)
                    - manualRPM.get())
            < 100.0;
    return atSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (shooterPidTuner.updated()) leftMotor.updateFromTuner(shooterPidTuner);
    Logger.recordOutput(
        "Shooter/Current RPM",
        RPM.convertFrom(leftMotor.getVelocity().getValueAsDouble(), RotationsPerSecond));

    Logger.recordOutput("Shooter/At Speed", isAtSpeed());
    Logger.recordOutput("Shooter/Manual Mode", Math.abs(manualRPM.get() - 3050.0) <= 2);

    leftMotor.logCurrents("Shooter/Left");
    rightMotor.logCurrents("Shooter/Right");
    leftMotor2.logCurrents("Shooter/Left2");
    rightMotor2.logCurrents("Shooter/Right2");
  }
}
