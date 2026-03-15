package frc.robot.azimuth;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.MultithreadedSubsystem;
import frc.lib.SafeAngleOptimizer;
import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.AbsoluteEncoderConfig.AbsoluteEncoderBuilder;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;
import frc.lib.motors.DiscreteMotorOutputSim;
import frc.lib.motors.MotorOutput;
import frc.lib.motors.MotorValues;
import frc.robot.RobotConstants;

/** Azimuth (turret yaw control) subsystem */
public class Azimuth extends MultithreadedSubsystem {
  
  /** Azimuth subsystem singleton */
  private static Azimuth instance = null;

  /** Azimuth motor output */
  private final MotorOutput motorOutput;

  /** Azimuth motor output values */
  private MotorValues motorValues = new MotorValues();

  /** Target position */
  private final MutAngle setpoint;

  /** Setpoint optimizer that handles max and min azimuth angle and safe angle wrapping */
  private final SafeAngleOptimizer setpointOptimizer;

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
          .kV(0.0)
          .kA(0.0)
          .kS(0.0)
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
          .sensorToMechRatio(5*10)
          .neutralBrake(true)
          .statorCurrentLimit(240)
          .supplyCurrentLimit(120)
          .build())
      .absoluteEncoderConfig(
        AbsoluteEncoderBuilder.defaults()
          .ccwPositive(true)
          .sensorToMechRatio(10)
          .build())
      .build();

  /**
   * Gets azimuth subsystem instance
   *
   * @return azimuth subsystem instance
   */
  public static Azimuth getInstance() {
    if (instance == null) {
      instance = new Azimuth();
    }

    return instance;
  }

  /** Azimuth subsystem constructor */
  private Azimuth() {
    motorOutput = AzimuthFactory.createAzimuthMotor(config);
    motorOutput.configure();
    motorOutput.setPosition(Rotations.of(0.25));

    setpoint = Rotations.mutable(0);
    setpointOptimizer = new SafeAngleOptimizer(Rotations.of(-1), Rotations.of(1));
    voltageSet = false;
    voltageOut = Volts.mutable(0);

    feedback = config.feedbackControllerConfig().createPIDController();
    feedforward = config.feedforwardControllerConfig().createSimpleMotorFeedforward();

    if (motorOutput instanceof DiscreteMotorOutputSim discreteSim) {
      discreteSim.usePosition(() -> setpoint);
    }
  }

  @Override
  public void initializeTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Azimuth");

    ShuffleboardLayout stateTab = tab.getLayout("Current state", BuiltInLayouts.kList);

    stateTab.addDouble("Motor voltage (V)", () -> motorValues.motorVoltage.in(Volts));
    stateTab.addDouble("Supply voltage (V)", () -> motorValues.supplyVoltage.in(Volts));
    stateTab.addDouble("Stator current (A)", () -> motorValues.statorCurrent.in(Amps));
    stateTab.addDouble("Supply current (A)", () -> motorValues.supplyCurrent.in(Amps));
    stateTab.addDouble("Position (rot)", () -> motorValues.position.in(Rotations));
    stateTab.addDouble("Velocity (rotps)", () -> motorValues.velocity.in(RotationsPerSecond));
    stateTab.addDouble("Acceleration (rotpsps)", () -> motorValues.acceleration.in(RotationsPerSecondPerSecond));

    tab.addDouble("Setpoint (rot)", () -> setpoint.in(Rotations));
  }

  @Override
  public void periodic() {}

  @Override
  public void fastPeriodic() {
    motorOutput.updateValues(motorValues, RobotConstants.FAST_PERIODIC_DURATION);

    double setpointRotations = setpoint.in(Rotations);
    double positionRotations = motorValues.position.in(Rotations);

    if (!voltageSet) {
      double feedbackVolts = feedback.calculate(positionRotations, setpointRotations);
      double feedforwardVolts = Math.copySign(feedforward.getKs(), setpointRotations - positionRotations);

      voltageOut.mut_replace(feedbackVolts + feedforwardVolts, Volts);
    }

    motorOutput.setVoltage(voltageOut);
  }

  /**
   * Returns a command that allows for temporary manual voltage control of the azimuth motor
   * 
   * @param voltageSupplier supplies a manual motor voltage to run at while the command is running
   * @return a command that allows for temporary manual voltage control of the azimuth motor
   */
  public Command runAtVoltage(Supplier<Voltage> voltageSupplier) {
    return Commands.run(() -> {
      voltageSet = true;
      Voltage voltageRequest = voltageSupplier.get();
      if (motorValues.position.gte(setpointOptimizer.getMaxAngle()) && voltageRequest.lte(Volts.zero())) {
        voltageOut.mut_replace(voltageRequest);
      } else if (motorValues.position.lte(setpointOptimizer.getMaxAngle()) && voltageRequest.gte(Volts.zero())) {
        voltageOut.mut_replace(voltageRequest);
      } else {
        voltageOut.mut_replace(Volts.zero());
      }
    }).finallyDo(() -> voltageSet = false);
  }

  /**
   * Set the local setpoint of the azimuth
   * Automatically handles angle wrapping and constraining
   * 
   * @param newSetpoint New local azimuth setpoint
   */
  public void setSetpoint(Angle newSetpoint) {
    setpoint.mut_replace(setpointOptimizer.optimizeSetpoint(setpoint, newSetpoint));
  }

  /**
   * Set the local setpoint of the azimuth without an automatic wrapping and constraining
   * Will do nothing and report a warning if setpoitnPosition is not within safe bounds
   * 
   * @param newSetpoint New local azimuth setpoint
   */
  public void setAbsoluteSetpoint(Angle newSetpoint) {
    if (newSetpoint.gte(setpointOptimizer.getMinAngle()) && newSetpoint.lte(setpointOptimizer.getMaxAngle())) {
      setpoint.mut_replace(newSetpoint);
    } else {
      DriverStation.reportWarning("Tried setting azimuth setpoint to position out of safe range ("+newSetpoint.in(Rotations)+" rotations)", null);
    }
  }

  /**
   * Gets the local setpoint of the azimuth
   * 
   * @return the local setpoint of the azimuth
   */
  public Angle getSetpoint() {
    return setpoint;
  }

  /**
   * Gets minimum safe azimuth angle
   * 
   * @return minimum safe azimuth angle
   */
  public Angle getMinPosition() {
    return setpointOptimizer.getMinAngle();
  }

  /**
   * Gets maximum safe azimuth angle
   * 
   * @return maximum safe azimuth angle
   */
  public Angle getMaxPosition() {
    return setpointOptimizer.getMaxAngle();
  }

  /**
   * Gets center azimuth angle (value halfway between min position and max position)
   * 
   * @return center azimuth angle
   */
  public Angle getCenter() {
    return setpointOptimizer.getCenter();
  }

  /**
   * Gets the current azimuth setpoint's distance from the edges of the range of safe angles
   * 
   * @return The current azimuth setpoint's distance from the edges of the range of safe angles
   * Negative values returned measure how far past the edges of the safe range the angle is
   */
  public Angle getCushion() {
    return setpointOptimizer.getCushion(setpoint);
  }

  public void resetPosition(Angle newPosition) {
    motorOutput.setPosition(newPosition);
  }

  public MechanismConfig getConfig() {
    return config;
  }

  public MotorValues getValues() {
    return motorValues;
  }

  /**
   * Gets the azimuth subsystem's position in the local frame.
   *
   * @return The position in the local frame.
   */
  public Angle localPosition() {
    return motorValues.position;
  }
}
