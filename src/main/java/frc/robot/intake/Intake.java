package frc.robot.intake;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.*;
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

/** Intake subsystem */
public class Intake extends Subsystem {
  
  /** Intake subsystem instance */
  private static Intake instance = null;

  private final MotorOutput pivotOutput;

  private final MotorOutput rollerOutput;

  private MotorValues pivotValues = new MotorValues();

  private MotorValues rollerValues = new MotorValues();

  private IntakePivotState targetPivotState;

  private IntakeRollerState targetRollerState;

  private IntakePivotState currentPivotState;

  private IntakeRollerState currentRollerState;

  private boolean pivotVoltageSet;

  private boolean rollerVoltageSet;

  private final MutVoltage pivotVoltageOut;

  private final MutVoltage rollerVoltageOut;

  private final PIDController pivotFeedback;

  private final PIDController rollerFeedback;

  private final SimpleMotorFeedforward rollerFeedforward;

  private final MechanismConfig pivotConfig =
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
          .ccwPositive(true)
          .rotorToSensorRatio(1)
          .sensorToMechRatio(23)
          .neutralBrake(true)
          .statorCurrentLimit(240)
          .supplyCurrentLimit(120)
          .build())
      .build();

  private final MechanismConfig rollerConfig =
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
          .ccwPositive(true)
          .rotorToSensorRatio(1)
          .sensorToMechRatio(1)
          .neutralBrake(true)
          .statorCurrentLimit(240)
          .supplyCurrentLimit(120)
          .build())
      .build();

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }

    return instance;
  }
  
  private Intake() {
    pivotOutput = IntakeFactory.createPivotMotor(pivotConfig);
    pivotOutput.configure();
    pivotOutput.setPosition(Rotations.of(0.35));

    rollerOutput = IntakeFactory.createRollerMotor(rollerConfig);
    rollerOutput.configure();

    targetPivotState = IntakePivotState.STOW;
    currentPivotState = IntakePivotState.STOW;
    targetRollerState = IntakeRollerState.STOP;
    currentRollerState = IntakeRollerState.STOP;

    pivotVoltageOut = Volts.mutable(0);
    rollerVoltageOut = Volts.mutable(0);

    pivotVoltageSet = false;
    rollerVoltageSet = false;

    pivotFeedback = pivotConfig.feedbackControllerConfig().createPIDController();
    rollerFeedback = rollerConfig.feedbackControllerConfig().createPIDController();
    rollerFeedforward = rollerConfig.feedforwardControllerConfig().createSimpleMotorFeedforward();
  }

  @Override
  public void initializeTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Intake");

    ShuffleboardLayout pivotStateTab = tab.getLayout("Current pivot state", BuiltInLayouts.kList);

    pivotStateTab.addDouble("Motor voltage (V)", () -> pivotValues.motorVoltage.in(Volts));
    pivotStateTab.addDouble("Supply voltage (V)", () -> pivotValues.supplyVoltage.in(Volts));
    pivotStateTab.addDouble("Stator current (A)", () -> pivotValues.statorCurrent.in(Amps));
    pivotStateTab.addDouble("Supply current (A)", () -> pivotValues.supplyCurrent.in(Amps));
    pivotStateTab.addDouble("Position (rot)", () -> pivotValues.position.in(Rotations));
    pivotStateTab.addDouble("Velocity (rotps)", () -> pivotValues.velocity.in(RotationsPerSecond));
    pivotStateTab.addDouble("Acceleration (rotpsps)", () -> pivotValues.acceleration.in(RotationsPerSecondPerSecond));
    pivotStateTab.addString("Current state", () -> currentPivotState.name());
    pivotStateTab.addBoolean("At target state", this::pivotAtTargetState);

    ShuffleboardLayout rollerStateTab = tab.getLayout("Current roller state", BuiltInLayouts.kList);

    rollerStateTab.addDouble("Motor voltage (V)", () -> rollerValues.motorVoltage.in(Volts));
    rollerStateTab.addDouble("Supply voltage (V)", () -> rollerValues.supplyVoltage.in(Volts));
    rollerStateTab.addDouble("Stator current (A)", () -> rollerValues.statorCurrent.in(Amps));
    rollerStateTab.addDouble("Supply current (A)", () -> rollerValues.supplyCurrent.in(Amps));
    rollerStateTab.addDouble("Position (rot)", () -> rollerValues.position.in(Rotations));
    rollerStateTab.addDouble("Velocity (rotps)", () -> rollerValues.velocity.in(RotationsPerSecond));
    rollerStateTab.addDouble("Acceleration (rotpsps)", () -> rollerValues.acceleration.in(RotationsPerSecondPerSecond));
    rollerStateTab.addString("Current state", () -> currentRollerState.name());
    rollerStateTab.addBoolean("At target state", this::rollerAtTargetState);

    tab.addString("Target pivot state", () -> targetPivotState.name());
    tab.addDouble("Target pivot positon (rot)", () -> targetPivotState.getPosition().in(Rotations));
    tab.addString("Target roller state", () -> targetRollerState.name());
    tab.addDouble("Target roller speed (rotps)", () -> targetRollerState.getVelocity().in(RotationsPerSecond));
  }

  @Override
  public void periodic() {
    pivotOutput.updateValues(pivotValues, RobotConstants.PERIODIC_DURATION);
    rollerOutput.updateValues(rollerValues, RobotConstants.PERIODIC_DURATION);

    // Pivot control
    double pivotSetpointRotations = targetPivotState.getPosition().in(Rotations);
    double pivotPositionRotations = pivotValues.position.in(Rotations);

    if (!pivotVoltageSet) {
      double feedbackVolts = pivotFeedback.calculate(pivotPositionRotations, pivotSetpointRotations);

      pivotVoltageOut.mut_replace(feedbackVolts, Volts);
    }

    pivotOutput.setVoltage(pivotVoltageOut);

    if (MathUtil.isNear(targetPivotState.getPosition().in(Rotations), pivotValues.position.in(Rotations), 0.02)) {
      currentPivotState = targetPivotState;
    } else {
      currentPivotState = IntakePivotState.MOVING;
    }

    // Roller control
    double rollerSetpointRotationsPerSecond = targetRollerState.getVelocity().in(RotationsPerSecond);
    double rollerVelocityRotationsPerSecond = rollerValues.velocity.in(RotationsPerSecond);

    if (!rollerVoltageSet) {
      double feedbackVolts = rollerFeedback.calculate(rollerSetpointRotationsPerSecond, rollerVelocityRotationsPerSecond);
      double feedforwardVolts = rollerFeedforward.calculate(rollerSetpointRotationsPerSecond);

      rollerVoltageOut.mut_replace(feedbackVolts + feedforwardVolts, Volts);
    }

    if (MathUtil.isNear(targetRollerState.getVelocity().in(RotationsPerSecond), rollerValues.velocity.in(RotationsPerSecond), 1)) {
      currentRollerState = targetRollerState;
    } else {
      currentRollerState = IntakeRollerState.MOVING;
    }

    rollerOutput.setVoltage(rollerVoltageOut);
  }

  public Command runPivotAtVoltage(Supplier<Voltage> voltageSupplier) {
    return Commands.run(() -> {
      pivotVoltageSet = true;
      Voltage voltageRequest = voltageSupplier.get();
      pivotVoltageOut.mut_replace(voltageRequest);
    }).finallyDo(() -> pivotVoltageSet = false);
  }

  public Command runRollerAtVoltage(Supplier<Voltage> voltageSupplier) {
    return Commands.run(() -> {
      rollerVoltageSet = true;
      Voltage voltageRequest = voltageSupplier.get();
      rollerVoltageOut.mut_replace(voltageRequest);
    }).finallyDo(() -> rollerVoltageSet = false);
  }

  public Command goToState(IntakePivotState pivotState) {
    return Commands.race(
      Commands.run(() -> {
        targetPivotState = pivotState;}
      ),
      Commands.waitUntil(this::pivotAtTargetState)
    );
  }

  public Command goToState(IntakeRollerState rollerState) {
    return Commands.race(
      Commands.run(() -> {
        targetRollerState = rollerState;
      }),
      Commands.waitUntil(this::rollerAtTargetState)
    );
  }

  public boolean pivotAtTargetState() {
    return currentPivotState == targetPivotState;
  }

  public boolean rollerAtTargetState() {
    return currentRollerState == targetRollerState;
  }

  public IntakePivotState getPivotState() {
    return currentPivotState;
  }

  public IntakePivotState getTargetPivotState() {
    return targetPivotState;
  }

  public IntakeRollerState getRollerState() {
    return currentRollerState;
  }

  public IntakeRollerState getTargetRollerState() {
    return targetRollerState;
  }

  public void resetPivotPosition(Angle newPosition) {
    pivotOutput.setPosition(newPosition);
  }

  public MechanismConfig getPivotConfig() {
    return pivotConfig;
  }

  public MechanismConfig getRollerConfig() {
    return rollerConfig;
  }

  public MotorValues getPivotValues() {
    return pivotValues;
  }

  public MotorValues getRollerValues() {
    return rollerValues;
  }
}
