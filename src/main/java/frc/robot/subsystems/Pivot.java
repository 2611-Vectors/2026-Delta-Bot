package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.VectorKit.hardware.AbsoluteEncoder;
import frc.robot.VectorKit.hardware.KrakenX60;
import frc.robot.VectorKit.tuners.TunablePidController;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Pivot extends SubsystemBase {
  private final KrakenX60 pivotMotor = new KrakenX60(IntakeConstants.PIVOT_MOTOR_ID);

  private final AbsoluteEncoder pivotEncoder =
      new AbsoluteEncoder(IntakeConstants.PIVOT_ENCODER_ID, IntakeConstants.PIVOT_ENCODER_OFFSET);

  private final TunablePidController pivotController =
      new TunablePidController("/Intake/Pivot/", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  public Pivot() {
    pivotEncoder.setInverted(false);
    pivotMotor.setBrakeMode(NeutralModeValue.Brake);
    pivotMotor.setInverted(InvertedValue.Clockwise_Positive);
  }

  public Command manualPivotPosition() {
    LoggedNetworkNumber pos = new LoggedNetworkNumber("/Intake/Pivot/Target Position", 0.0);
    return setPivotPosition(() -> pos.get());
  }

  public Command setPivotVoltage(Supplier<Double> voltage) {
    return runOnce(
        () -> {
          pivotMotor.setVoltage(voltage.get());
        });
  }

  public Command setPivotPosition(Supplier<Double> position) {
    return setPivotVoltage(() -> pivotController.calculate(pivotEncoder.get(), position.get()));
  }

  public Command manualPivotVoltage() {
    LoggedNetworkNumber voltage = new LoggedNetworkNumber("/Intake/Pivot/Voltage", 0.0);
    return setPivotVoltage(() -> voltage.get());
  }

  public Command dumbIntakeOut() {
    return run(() -> {
          pivotMotor.setVoltage(5.0);
        })
        .until(() -> intakeIsOut())
        .andThen(
            () -> {
              pivotMotor.setVoltage(0.0);
            })
        .handleInterrupt(
            () -> {
              pivotMotor.setVoltage(0.0);
            });
  }

  public Command dumbIntakeIn() {
    return run(() -> {
          pivotMotor.setVoltage(-3.0);
        })
        .until(() -> pivotEncoder.get() <= IntakeConstants.PIVOT_IN_ANGLE)
        .andThen(
            () -> {
              pivotMotor.setVoltage(0.0);
            })
        .handleInterrupt(
            () -> {
              pivotMotor.setVoltage(0.0);
            });
  }

  public boolean intakeIsOut() {
    return pivotEncoder.get() >= IntakeConstants.PIVOT_OUT_ANGLE;
  }

  public boolean intakeCanRun() {
    return pivotEncoder.get() >= IntakeConstants.PIVOT_RUN_ANGLE;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/Pivot/Current Angle", pivotEncoder.get());
    Logger.recordOutput("Intake/Pivot/New Offset", (pivotEncoder.getRaw() * 360.0) - 0.5);
    pivotController.update();
    Logger.recordOutput(
        "Intake/Pivot/Current RPM (Output)",
        pivotMotor.getRPM() / IntakeConstants.PIVOT_GEAR_RATIO);
    Logger.recordOutput("Intake/Pivot/Current RPM (Motor)", pivotMotor.getRPM());
    pivotMotor.logCurrents("Intake/Pivot");
  }
}
