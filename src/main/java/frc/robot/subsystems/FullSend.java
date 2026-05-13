package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FullSendConstants;
import frc.robot.VectorKit.hardware.KrakenX60;
import frc.robot.VectorKit.tuners.PidTuner;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class FullSend extends SubsystemBase {
  private final KrakenX60 fullSendMotor = new KrakenX60(FullSendConstants.MOTOR_ID);

  private final PidTuner fullSendPidTuner = new PidTuner("/FullSend/", 0.6, 0.0, 0.0, 0.0, 0.13);

  public FullSend() {
    fullSendMotor.setInverted(InvertedValue.Clockwise_Positive);
    fullSendMotor.setStatorCurrentLimit(50);
  }

  public Command setFullSendVoltage(Supplier<Double> voltage) {
    return run(() -> {
          fullSendMotor.setVoltage(voltage.get());
        })
        .handleInterrupt(
            () -> {
              fullSendMotor.setVoltage(0.0);
            });
  }

  public Command setFullSendRPM(Supplier<Double> rpm) {
    return run(() -> {
          fullSendMotor.setVelocity(rpm.get() / FullSendConstants.GEAR_RATIO, RPM);
        })
        .handleInterrupt(
            () -> {
              fullSendMotor.setVoltage(0.0);
            });
  }

  public Command manualFullSendRPM(Supplier<Boolean> reverse) {
    LoggedNetworkNumber rpm = new LoggedNetworkNumber("/FullSend/Target RPM", 1000.0);
    return setFullSendRPM(() -> (reverse.get() ? -rpm.get() : rpm.get()));
  }

  @Override
  public void periodic() {
    if (fullSendPidTuner.updated()) fullSendMotor.updateFromTuner(fullSendPidTuner);

    Logger.recordOutput(
        "/FullSend/Currend RPM", fullSendMotor.getVelocity().getValueAsDouble() * 60);
    fullSendMotor.logCurrents("/FullSend");
  }
}
