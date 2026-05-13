package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransitionConstants;
import frc.robot.VectorKit.hardware.KrakenX60;
import frc.robot.VectorKit.tuners.PidTuner;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Transition extends SubsystemBase {
  /** Creates a new Transition. */
  private final KrakenX60 lowerMotor = new KrakenX60(TransitionConstants.LOWER_MOTOR_ID);

  // private final KrakenX60 upperLeftMotor = new
  // KrakenX60(TransitionConstants.UPPER_LEFT_MOTOR_ID);
  // private final KrakenX60 upperRightMotor = new
  // KrakenX60(TransitionConstants.UPPER_RIGHT_MOTOR_ID);

  private final PidTuner lowerTransitionPidTuner =
      new PidTuner("/Transition/Lower/", 0.1, 0.02, 0.0, 0.0, 0.14);
  // private final PidTuner upperTransitionPidTuner = new PidTuner("/Transition/Upper/", 0.1, 0.02,
  // 0.0, 0.0, 0.11);

  public Transition() {
    // upperLeftMotor.setFollower(upperRightMotor, MotorAlignmentValue.Opposed);
    // upperLeftMotor.setInverted(InvertedValue.CounterClockwise_Positive);
    // upperLeftMotor.setStatorCurrentLimit(10);

    lowerMotor.setInverted(InvertedValue.Clockwise_Positive);
    lowerMotor.setStatorCurrentLimit(60);
  }

  // public Command setUpperTransitioVoltage(Supplier<Double> voltage) {
  //     return run(() -> {
  //                 upperLeftMotor.setVoltage(voltage.get());
  //             })
  //             .handleInterrupt(() -> {
  //                 upperLeftMotor.setVoltage(0.0);
  //             });
  // }

  // public Command setUpperTransitionRPM(Supplier<Double> rpm) {
  //     return run(() -> {
  //                 upperLeftMotor.setVelocity(rpm.get() / TransitionConstants.UPPER_GEAR_RATIO,
  // RPM);
  //             })
  //             .handleInterrupt(() -> {
  //                 upperLeftMotor.setVoltage(0.0);
  //             });
  // }

  // public Command manualUpperTransitionRPM(Supplier<Boolean> reverse) {
  //     LoggedNetworkNumber rpm = new LoggedNetworkNumber("/Transition/Upper/Target RPM", 2000.0);
  //     return setUpperTransitionRPM(() -> (reverse.get() ? -rpm.get() : rpm.get()));
  // }

  public Command setLowerTransitionVoltage(Supplier<Double> voltage) {
    return run(() -> {
          lowerMotor.setVoltage(voltage.get());
        })
        .handleInterrupt(
            () -> {
              lowerMotor.setVoltage(0.0);
            });
  }

  public Command setLowerTransitionRPM(Supplier<Double> rpm) {
    return run(() -> {
          lowerMotor.setVelocity(rpm.get() / TransitionConstants.LOWER_GEAR_RATIO, RPM);
        })
        .handleInterrupt(
            () -> {
              lowerMotor.setVoltage(0.0);
            });
  }

  public Command manualLowerTransitionRPM(Supplier<Boolean> reverse) {
    LoggedNetworkNumber rpm = new LoggedNetworkNumber("/Transition/Lower/Target RPM", 2000.0);
    return setLowerTransitionRPM(() -> (reverse.get() ? -rpm.get() : rpm.get()));
  }

  // public Command setTransitionVoltage(Supplier<Double> upperVoltage, Supplier<Double>
  // lowerVoltage) {
  //     return run(() -> {
  //                 lowerMotor.setVoltage(lowerVoltage.get());
  //                 upperLeftMotor.setVoltage(upperVoltage.get());
  //             })
  //             .handleInterrupt(() -> {
  //                 lowerMotor.setVoltage(0.0);
  //                 upperLeftMotor.setVoltage(0.0);
  //             });
  // }

  // public Command setTransitionRPM(Supplier<Double> upperRPM, Supplier<Double> lowerRPM) {
  //     return run(() -> {
  //                 lowerMotor.setVelocity(lowerRPM.get() / TransitionConstants.LOWER_GEAR_RATIO,
  // RPM);
  //                 upperLeftMotor.setVelocity(upperRPM.get() /
  // TransitionConstants.UPPER_GEAR_RATIO, RPM);
  //             })
  //             .handleInterrupt(() -> {
  //                 lowerMotor.setVoltage(0.0);
  //                 upperLeftMotor.setVoltage(0.0);
  //             });
  // }

  // public Command manualTransitionRPM(Supplier<Boolean> reverse) {
  //     LoggedNetworkNumber lowerRPM = new LoggedNetworkNumber("/Transition/Lower/Target RPM",
  // 2000.0);
  //     LoggedNetworkNumber upperRPM = new LoggedNetworkNumber("/Transition/Upper/Target RPM",
  // 2000.0);
  //     return setTransitionRPM(
  //             () -> (reverse.get() ? -upperRPM.get() : upperRPM.get()),
  //             () -> (reverse.get() ? -lowerRPM.get() : lowerRPM.get()));
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (lowerTransitionPidTuner.updated()) lowerMotor.updateFromTuner(lowerTransitionPidTuner);
    // if (upperTransitionPidTuner.updated())
    // upperLeftMotor.updateFromTuner(upperTransitionPidTuner);

    Logger.recordOutput("Transition/Lower/Current RPM (Motor)", lowerMotor.getRPM());
    // Logger.recordOutput("Transition/Upper/Current Left RPM (Motor)", upperLeftMotor.getRPM());
    // Logger.recordOutput("Transition/Upper/Current Right RPM (Motor)", upperRightMotor.getRPM());

    Logger.recordOutput(
        "Transition/Lower/Current RPM (Output)",
        lowerMotor.getRPM() * TransitionConstants.LOWER_GEAR_RATIO);
    // Logger.recordOutput(
    //         "Transition/Upper/Current Left RPM (Output)",
    //         upperLeftMotor.getRPM() * TransitionConstants.UPPER_GEAR_RATIO);
    // Logger.recordOutput(
    //         "Transition/Upper/Current Right RPM (Output)",
    //         upperRightMotor.getRPM() * TransitionConstants.UPPER_GEAR_RATIO);

    // upperLeftMotor.logCurrents("Transition/Upper/Left");
    // upperRightMotor.logCurrents("Transition/Upper/Right");
    lowerMotor.logCurrents("Transition/Lower");
  }
}
