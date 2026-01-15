package frc.robot.shooter;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import frc.lib.MultithreadedSubsystem;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;
import frc.lib.motors.MotorOutput;
import frc.lib.motors.MotorValues;
import frc.robot.RobotConstants;

/** Shooter subsystem (handles velocity control of shooter rollers */
public class Shooter extends MultithreadedSubsystem {
  
  /** Shooter subsystem singleton */
  private static Shooter instance = null;

  /** Shooter motor output (specific type of motor output is defined in ShooterFactory) */
  private final MotorOutput motorOutput;

  /** Shooter motor output values (used to keep track of things like voltage, velocity, acceleration, stator current, temperature, etc) */
  private MotorValues motorValues = new MotorValues();

  /** Target angular velocity */
  private final MutAngularVelocity setpointVelocity;

  /** True if a manual voltage is set by runAtVoltage command */
  private boolean voltageSet;

  /** Output voltage */
  private final MutVoltage voltageOut;

  /** Feedback controller */
  private PIDController feedback;

  /** Feedforward controller */
  private SimpleMotorFeedforward feedforward;

  /** Shooter mechanism configuration that provides default values for motor control configuration and motor configuration */
  private MechanismConfig config =
    MechanismBuilder.defaults()
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.defaults()
          .kV(0.1164)
          .kA(0.0045783)
          .kS(0.006235)
          .build())
      .feedbackControllerConfig(
        FeedbackControllerBuilder.defaults()
          .kP(0.0)
          .kI(0.0)
          .kD(0.0)
          .build())
      .motorConfig(
        MotorBuilder.defaults()
          .ccwPositive(false)
          .rotorToSensorRatio(1)
          .sensorToMechRatio(1)
          .neutralBrake(true)
          .statorCurrentLimit(80)
          .supplyCurrentLimit(40)
          .build())
      .build();

  /**
   * Gets shooter subsystem instance
   * 
   * @return shooter subsystem instance
   */
  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }

    return instance;
  }

  /** Elevator subsystem constructor */
  private Shooter() {
    motorOutput = ShooterFactory.createShooterMotor(config);

    setpointVelocity = RotationsPerSecond.mutable(0);
    voltageSet = false;
    voltageOut = Volts.mutable(0);

    feedback = config.feedbackControllerConfig().createPIDController();
    feedforward = config.feedforwardControllerConfig().createSimpleMotorFeedforward();
  }

  @Override
  public void initializeTab() {

  }

  @Override
  public void periodic() {}

  @Override
  public void fastPeriodic() {
    motorOutput.updateValues(motorValues, RobotConstants.FAST_PERIODIC_DURATION);

    if (!voltageSet) {
      double feedbackVolts = feedback.calculate(motorValues.velocity.in(RotationsPerSecond), setpointVelocity.in(RotationsPerSecond));
      double feedforwardVolts = feedforward.calculate(setpointVelocity.in(RotationsPerSecond));

      voltageOut.mut_replace(feedbackVolts + feedforwardVolts, Volts);
    }

    motorOutput.setVoltage(voltageOut);
  }

  /**
   * Returns a command that allows for temporary manual voltage control of the shooter
   * 
   * @param voltageSupplier supplies a manual motor voltage to run at while the command is running
   * @return a command that allows for temporary manual voltage control of the shooter
   */
  public Command runAtVoltage(Supplier<Voltage> voltageSupplier) {
    return Commands.run(() -> {
      voltageSet = true;
      voltageOut.mut_replace(voltageSupplier.get());
    }).finallyDo(() -> voltageSet = false);
  }

  public void setSetpoint(AngularVelocity velocitySetpoint) {
    setpointVelocity.mut_replace(velocitySetpoint);
  }

  public void setKP(double kP) {
    config = MechanismBuilder.edit(config)
      .feedbackControllerConfig(
        FeedbackControllerBuilder.edit(config.feedbackControllerConfig())
          .kP(kP)
          .build())
      .build();

    feedback.setP(kP);
  }

  public void setKI(double kI) {
    config = MechanismBuilder.edit(config)
      .feedbackControllerConfig(
        FeedbackControllerBuilder.edit(config.feedbackControllerConfig())
          .kI(kI)
          .build())
      .build();

    feedback.setI(kI);
  }

  public void setKD(double kD) {
    config = MechanismBuilder.edit(config)
      .feedbackControllerConfig(
        FeedbackControllerBuilder.edit(config.feedbackControllerConfig())
          .kD(kD)
          .build())
      .build();
    
    feedback.setD(kD);
  }

  public void setKV(double kV) {
    config = MechanismBuilder.edit(config)
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.edit(config.feedforwardControllerConfig())
          .kV(kV)
          .build())
      .build();
  }

  public void setKA(double kA) {
    config = MechanismBuilder.edit(config)
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.edit(config.feedforwardControllerConfig())
          .kA(kA)
          .build())
      .build();
  }

  public void setKS(double kS) {
    config = MechanismBuilder.edit(config)
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.edit(config.feedforwardControllerConfig())
          .kS(kS)
          .build())
      .build();
  }

  public AngularVelocity getSetpoint() {
    return setpointVelocity;
  }

  public MechanismConfig getConfig() {
    return config;
  }

  public MotorValues getValues() {
    return motorValues;
  }
}
