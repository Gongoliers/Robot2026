package frc.robot.intake;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
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
import frc.lib.motors.DiscreteMotorOutputSim;
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

  private IntakeState state;

  private boolean pivotVoltageSet;

  private boolean rollerVoltageSet;

  private final MutAngle pivotSetpoint;

  private final MutAngularVelocity rollerSetpoint;

  private final MutVoltage pivotVoltageOut;

  private final MutVoltage rollerVoltageOut;

  private final PIDController pivotFeedback;

  private final PIDController rollerFeedback;

  private final ArmFeedforward pivotFeedforward;

  private final SimpleMotorFeedforward rollerFeedforward;

  private final MechanismConfig pivotConfig =
    MechanismBuilder.defaults()
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.defaults()
          .kV(0)
          .kA(0)
          .kS(0)
          .kG(0.35)
          .build())
      .feedbackControllerConfig(
        FeedbackControllerBuilder.defaults()
          .kP(20)
          .kI(0)
          .kD(1)
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
          .kV(0.11367)
          .kA(0.0021923)
          .kS(0.16469)
          .kG(0)
          .build())
      .feedbackControllerConfig(
        FeedbackControllerBuilder.defaults()
          .kP(0.013229)
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
    pivotOutput.setPosition(IntakeState.START.pivotSetpoint);

    rollerOutput = IntakeFactory.createRollerMotor(rollerConfig);
    rollerOutput.configure();

    state = IntakeState.START;

    pivotVoltageOut = Volts.mutable(0);
    rollerVoltageOut = Volts.mutable(0);

    pivotVoltageSet = false;
    rollerVoltageSet = false;

    pivotSetpoint = IntakeState.START.pivotSetpoint.mutableCopy();
    rollerSetpoint = IntakeState.START.rollerSetpoint.mutableCopy();

    pivotFeedback = pivotConfig.feedbackControllerConfig().createPIDController();
    rollerFeedback = rollerConfig.feedbackControllerConfig().createPIDController();
    pivotFeedforward = pivotConfig.feedforwardControllerConfig().createArmFeedforward();
    rollerFeedforward = rollerConfig.feedforwardControllerConfig().createSimpleMotorFeedforward();

    if (pivotOutput instanceof DiscreteMotorOutputSim discreteSim) {
      discreteSim.usePosition(() -> pivotSetpoint);
    }

    if (rollerOutput instanceof DiscreteMotorOutputSim discreteSim) {
      discreteSim.useVelocity(() -> rollerSetpoint);
    }
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
    pivotStateTab.addDouble("Setpoint (rot)", () -> pivotSetpoint.in(Rotations));

    ShuffleboardLayout rollerStateTab = tab.getLayout("Current roller state", BuiltInLayouts.kList);

    rollerStateTab.addDouble("Motor voltage (V)", () -> rollerValues.motorVoltage.in(Volts));
    rollerStateTab.addDouble("Supply voltage (V)", () -> rollerValues.supplyVoltage.in(Volts));
    rollerStateTab.addDouble("Stator current (A)", () -> rollerValues.statorCurrent.in(Amps));
    rollerStateTab.addDouble("Supply current (A)", () -> rollerValues.supplyCurrent.in(Amps));
    rollerStateTab.addDouble("Position (rot)", () -> rollerValues.position.in(Rotations));
    rollerStateTab.addDouble("Velocity (rotps)", () -> rollerValues.velocity.in(RotationsPerSecond));
    rollerStateTab.addDouble("Acceleration (rotpsps)", () -> rollerValues.acceleration.in(RotationsPerSecondPerSecond));
    rollerStateTab.addDouble("Setpoint (rotps)", () -> rollerSetpoint.in(RotationsPerSecond));

    tab.addString("Current state", () -> state.name());
  }

  @Override
  public void periodic() {
    pivotOutput.updateValues(pivotValues, RobotConstants.PERIODIC_DURATION);
    rollerOutput.updateValues(rollerValues, RobotConstants.PERIODIC_DURATION);

    // Define control to use for each state
    if (state == IntakeState.AGITATE) {
      double oscillationAmplitude = 0.1; // Oscillation amplitude in seconds
      double oscillationFrequency = 1.5; // Oscillation frequency in cycles per second

      double oscillation = Math.sin(6.28 * oscillationFrequency * RobotController.getMeasureTime().in(Seconds));
      double oscillatingSetpointRotations = oscillation * oscillationAmplitude + IntakeState.OUT.pivotSetpoint.in(Rotations) + oscillationAmplitude;

      pivotSetpoint.mut_replace(oscillatingSetpointRotations, Rotations);
      rollerSetpoint.mut_replace(state.rollerSetpoint);
    } else {
      pivotSetpoint.mut_replace(state.pivotSetpoint);
      rollerSetpoint.mut_replace(state.rollerSetpoint);
    }

    // Pivot control
    double pivotSetpointRotations = pivotSetpoint.in(Rotations);
    double pivotPositionRotations = pivotValues.position.in(Rotations);

    if (!pivotVoltageSet) {
      double feedbackVolts = pivotFeedback.calculate(pivotPositionRotations, pivotSetpointRotations);
      feedbackVolts = Math.copySign(Math.min(Math.abs(feedbackVolts), 4), feedbackVolts);
      double feedforwardVolts = pivotFeedforward.getKg() * Math.cos(pivotValues.position.in(Radians));

      pivotVoltageOut.mut_replace(feedbackVolts + feedforwardVolts, Volts);
    }

    pivotOutput.setVoltage(pivotVoltageOut);

    // Roller control
    double rollerSetpointRotationsPerSecond = rollerSetpoint.in(RotationsPerSecond);
    double rollerVelocityRotationsPerSecond = rollerValues.velocity.in(RotationsPerSecond);

    if (!rollerVoltageSet) {
      double feedbackVolts = rollerFeedback.calculate(rollerSetpointRotationsPerSecond, rollerVelocityRotationsPerSecond);
      double feedforwardVolts = rollerFeedforward.calculate(rollerSetpointRotationsPerSecond);

      rollerVoltageOut.mut_replace(feedbackVolts + feedforwardVolts, Volts);
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

  public Command setState(IntakeState intakeState) {
    return Commands.runOnce(() -> state = intakeState);
  }

  public boolean atTargetState() {
    if (state == IntakeState.AGITATE) {
      return pivotValues.position.lte(Rotations.of(0.2)) 
          && MathUtil.isNear(rollerSetpoint.in(RotationsPerSecond), rollerValues.velocity.in(RotationsPerSecond), 1);
    } else {
      return MathUtil.isNear(pivotSetpoint.in(Rotations), pivotValues.position.in(Rotations), 0.05)
          && MathUtil.isNear(rollerSetpoint.in(RotationsPerSecond), rollerValues.velocity.in(RotationsPerSecond), 1);
    }
  }

  public IntakeState getState() {
    return state;
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
