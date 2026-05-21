package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.VectorKit.hardware.KrakenX60;
import frc.robot.VectorKit.tuners.PidTuner;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake extends SubsystemBase {
  private final KrakenX60 intakeMotor = new KrakenX60(IntakeConstants.WHEEL_MOTOR_ID);
  private final KrakenX60 intakeMotor2 = new KrakenX60(IntakeConstants.WHEEL_MOTOR2_ID);

  private final PidTuner intakePidTuner = new PidTuner("/Intake/", 0.1, 0.02, 0.0, 0.0, 0.13);

  public Intake() {
    intakeMotor.setFollower(intakeMotor2, MotorAlignmentValue.Opposed);

    intakeMotor.setInverted(InvertedValue.CounterClockwise_Positive);
    intakeMotor.setStatorCurrentLimit(80);
  }

  public Command setIntakeVoltage(Supplier<Double> voltage) {
    return runOnce(
            () -> {
              intakeMotor.setVoltage(voltage.get());
            })
        .handleInterrupt(
            () -> {
              intakeMotor.setVoltage(0.0);
            });
  }

  public Command setIntakeRPM(Supplier<Double> rpm) {
    return run(() -> {
          intakeMotor.setVelocity(rpm.get() / IntakeConstants.INTAKE_GEAR_RATIO, RPM);
        })
        .handleInterrupt(
            () -> {
              intakeMotor.setVoltage(0.0);
            });
  }

  public Command manualIntakeRPM(Supplier<Boolean> reverse) {
    LoggedNetworkNumber rpm = new LoggedNetworkNumber("/Intake/Target RPM", 3000.0);
    LoggedNetworkNumber revrpm = new LoggedNetworkNumber("/Intake/Target Reverse RPM", 500.0);
    return setIntakeRPM(() -> (reverse.get() ? -revrpm.get() : rpm.get()));
  }

  @Override
  public void periodic() {
    if (intakePidTuner.updated()) intakeMotor.updateFromTuner(intakePidTuner);

    Logger.recordOutput("Intake/Current RPM (Motor)", intakeMotor.getRPM());
    Logger.recordOutput(
        "Intake/Current RPM (Output)", intakeMotor.getRPM() * IntakeConstants.INTAKE_GEAR_RATIO);

    intakeMotor.logCurrents("Intake");
  }
}
