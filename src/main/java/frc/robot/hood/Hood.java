package frc.robot.hood;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.MultithreadedSubsystem;
import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;
import frc.lib.motors.MotorOutput;
import frc.lib.motors.MotorValues;
import frc.robot.RobotConstants;

/** Hood subsystem */
public class Hood extends MultithreadedSubsystem {
  
  /** Hood subsystem singleton */
  private static Hood instance = null;

  /** Hood motor output */
  private final MotorOutput motorOutput;

  /** Hood motor output values */
  private MotorValues motorValues = new MotorValues();

  /** Target position */
  private final MutAngle setpoint;

  /** True if a manual voltage is set by runAtVoltage command */
  private boolean voltageSet;

  /** Output voltage */
  private final MutVoltage voltageOut;

  /** Feedback controller */
  private final PIDController feedback;

  /** Feedforward controller */
  private final ArmFeedforward feedforward;

  /** Hood mechanism configuration that provides default values for motor control configuration and motor configuration */
  private MechanismConfig config =
    MechanismBuilder.defaults()
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.defaults()
          .kV(7.216)
          .kA(0.20182)
          .kS(0.20099)
          .kG(0.0)
          .build())
      .feedbackControllerConfig(
        FeedbackControllerBuilder.defaults()
          .kP(45.336)
          .kI(0.0)
          .kD(5.2604)
          .build())
      .motorConfig(
        MotorBuilder.defaults()
          .ccwPositive(false)
          .rotorToSensorRatio(1)
          .sensorToMechRatio(4*2*18)
          .neutralBrake(true)
          .statorCurrentLimit(240)
          .supplyCurrentLimit(120)
          .build())
      .build();

  /**
   * Gets hood subsystem instance
   * 
   * @return hood subsystem instance
   */
  public static Hood getInstance() {
    if (instance == null) {
      instance = new Hood();
    }

    return instance;
  }

  /** Hood subsystem constructor */
  private Hood() {
    motorOutput = HoodFactory.createHoodMotor(config);
    motorOutput.configure();
    motorOutput.setPosition(Rotations.of(0.0)); // TODO: figure out the actaual position of the COM in starting configuration
    
    setpoint = Rotations.mutable(0.0);
    voltageSet = false;
    voltageOut = Volts.mutable(0.0);

    feedback = config.feedbackControllerConfig().createPIDController();
    feedforward = config.feedforwardControllerConfig().createArmFeedforward();
  }

  @Override
  public void initializeTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Hood");

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
      double feedforwardVolts = Math.cos(positionRotations)*feedforward.getKg() + feedforward.getKs();

      voltageOut.mut_replace(feedbackVolts + feedforwardVolts, Volts);
    }

    motorOutput.setVoltage(voltageOut);
  }
  
  /**
   * Returns a command that allows for temporary manual voltage control of the hood motor
   * 
   * @param voltageSupplier supplies a manual motor voltage to run at while the command is running
   * @return a command that allows for temporary manual voltage control of the hood motor
   */
  public Command runAtVoltage(Supplier<Voltage> voltageSupplier) {
    return Commands.run(() -> {
      voltageSet = true;
      voltageOut.mut_replace(voltageSupplier.get()); // TODO: build in some safety so something like sysid can't send the thing flying away to space
    }).finallyDo(() -> voltageSet = false);
  }

  public void setSetpoint(Angle setpointPosition) {
    setpoint.mut_replace(setpointPosition);
  }

  public Angle getSetpoint() {
    return setpoint;
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
}
