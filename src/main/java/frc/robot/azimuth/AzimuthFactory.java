package frc.robot.azimuth;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.motors.LossyMotorOutputSim;
import frc.lib.motors.MotorOutput;
import frc.lib.motors.MotorOutputSim;
import frc.lib.motors.MotorOutputTalonFX;
import frc.robot.Robot;
import frc.robot.RobotConstants;

/** Creates azimuth hardware abstractions */
public class AzimuthFactory {
  
  public static MotorOutput createAzimuthMotor(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(RobotConstants.Subsystem.AZIMUTH)) {
      return new MotorOutputTalonFX(config.motorConfig(), new CAN(0));
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
