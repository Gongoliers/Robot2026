package frc.robot.azimuth;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.motors.LossyMotorOutputSim;
import frc.lib.motors.MotorOutput;
import frc.lib.motors.MotorOutputSim;
import frc.lib.motors.MotorOutputTalonFXCANcoder;
import frc.lib.sensors.Gyroscope;
import frc.lib.sensors.GyroscopePigeon2;
import frc.lib.sensors.GyroscopeSim;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.drive.Drive;

/** Creates azimuth hardware abstractions */
public class AzimuthFactory {
  
  public static MotorOutput createAzimuthMotor(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(RobotConstants.Subsystem.AZIMUTH)) {
      return new MotorOutputTalonFXCANcoder(config.motorConfig(), config.absoluteEncoderConfig(), new CAN(0), new CAN(37));
    }

    MotorOutputSim losslessSim = new MotorOutputSim(
      Volts.per(RotationsPerSecond).ofNative(config.feedforwardControllerConfig().kV()),
      Volts.per(RotationsPerSecondPerSecond).ofNative(config.feedforwardControllerConfig().kA()),
      DCMotor.getKrakenX60(1));

    return new LossyMotorOutputSim(
      losslessSim, 
      Volts.of(config.feedforwardControllerConfig().kS()));
  }
}
