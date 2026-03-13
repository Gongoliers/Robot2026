package frc.robot.spindexer;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;
import frc.lib.motors.MotorOutput;
import frc.lib.motors.MotorValues;
import frc.robot.RobotConstants;

public class Spindexer extends Subsystem {
  
  private static Spindexer instance = null;

  private final MotorOutput motorOutput;

  private MotorValues motorValues = new MotorValues();

  private SpindexerState targetState;

  private SpindexerState currentState;

  private boolean voltageSet;

  private final MutVoltage voltageOut;

  private final PIDController feedback;

  private final SimpleMotorFeedforward feedforward;

  private final MechanismConfig config =
    MechanismBuilder.defaults()
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.defaults()
          .kV(0)
          .kA(0)
          .kS(0)
          .kG(0)
          .build())
      .feedbackControllerConfig(
        FeedbackControllerBuilder.defaults()
          .kP(0)
          .kI(0)
          .kD(0)
          .build())
      .motorConfig(
        MotorBuilder.defaults()
          .ccwPositive(false)
          .rotorToSensorRatio(1)
          .sensorToMechRatio(3*(36/15))
          .neutralBrake(true)
          .statorCurrentLimit(240)
          .supplyCurrentLimit(120)
          .build())
      .build();

  public static Spindexer getInstance() {
    if (instance == null) {
      instance = new Spindexer();
    }

    return instance;
  }

  private Spindexer() {
    motorOutput = SpindexerFactory.createSpindexerMotor(config);
    motorOutput.configure();
    
    targetState = SpindexerState.STOP;
    currentState = SpindexerState.STOP;

    voltageOut = Volts.mutable(0);

    voltageSet = false;

    feedback = config.feedbackControllerConfig().createPIDController();
    feedforward = config.feedforwardControllerConfig().createSimpleMotorFeedforward();
  }

  @Override
  public void initializeTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Spindexer");

    ShuffleboardLayout stateTab = tab.getLayout("Current state", BuiltInLayouts.kList);

    stateTab.addDouble("Motor voltage (V)", () -> motorValues.motorVoltage.in(Volts));
    stateTab.addDouble("Supply voltage (V)", () -> motorValues.supplyVoltage.in(Volts));
    stateTab.addDouble("Stator current (A)", () -> motorValues.statorCurrent.in(Amps));
    stateTab.addDouble("Supply current (A)", () -> motorValues.supplyCurrent.in(Amps));
    stateTab.addDouble("Position (rot)", () -> motorValues.position.in(Rotations));
    stateTab.addDouble("Velocity (rotps)", () -> motorValues.velocity.in(RotationsPerSecond));
    stateTab.addDouble("Acceleration (rotpsps)", () -> motorValues.acceleration.in(RotationsPerSecondPerSecond));
    stateTab.addString("Current state", () -> currentState.name());
    stateTab.addBoolean("At target state", this::atTargetState);

    tab.addString("Target state", () -> targetState.name());
    tab.addDouble("Target velocity (rotps)", () -> targetState.getVelocity().in(RotationsPerSecond));
  }

  @Override
  public void periodic() {
    motorOutput.updateValues(motorValues, RobotConstants.PERIODIC_DURATION);

    double setpointRotationsPerSecond = targetState.getVelocity().in(RotationsPerSecond);
    double velocityRotationsPerSecond = motorValues.velocity.in(RotationsPerSecond);

    if (!voltageSet) {
      double feedbackVolts = feedback.calculate(velocityRotationsPerSecond, setpointRotationsPerSecond);
      double feedforwardVolts = feedforward.calculate(setpointRotationsPerSecond);

      voltageOut.mut_replace(feedbackVolts + feedforwardVolts, Volts);
    }

    motorOutput.setVoltage(voltageOut);

    if (MathUtil.isNear(targetState.getVelocity().in(RotationsPerSecond), motorValues.velocity.in(RotationsPerSecond), 1)) {
      currentState = targetState;
    } else {
      currentState = SpindexerState.MOVING;
    }
  }

  public Command runAtVoltage(Supplier<Voltage> voltageSupplier) {
    return Commands.run(() -> {
      voltageSet = true;
      Voltage voltageRequest = voltageSupplier.get();
      voltageOut.mut_replace(voltageRequest);
    }).finallyDo(() -> voltageSet = false);
  }

  public Command goToState(SpindexerState spindexerState) {
    return Commands.race(
      Commands.run(() -> {
        targetState = spindexerState;
      }),
      Commands.waitUntil(this::atTargetState)
    );
  }

  public boolean atTargetState() {
    return currentState == targetState;
  }

  public SpindexerState getState() {
    return currentState;
  }

  public SpindexerState getTargetState() {
    return targetState;
  }

  public MechanismConfig getConfig() {
    return config;
  }

  public MotorValues getValues() {
    return motorValues;
  }
}
