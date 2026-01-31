package frc.robot.azimuth;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.LimelightHelpers;
import frc.robot.RobotConstants;
import frc.robot.LimelightHelpers.PoseEstimate;

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

  /** True if a manual voltage is set by runAtVoltage command */
  private boolean voltageSet;
  
  /** Output voltage */
  private final MutVoltage voltageOut;
  
  /** Feedback controller */
  private PIDController feedback;

  /** Feedforward controller */
  private SimpleMotorFeedforward feedforward;

  /** Chassis IMU reference */
  private final Pigeon2 pigeon;

  /** Status signal that gets yaw velocity */
  private final StatusSignal<AngularVelocity> omega;

  /** Previous yaw velocity */
  private AngularVelocity previousOmega;

  /** Shooter mechanism configuration that provides default values for motor control configuration and motor configuration */
  private MechanismConfig config =
    MechanismBuilder.defaults()
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.defaults()
          .kV(7.216)
          .kA(0.20182)
          .kS(0.20099)
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
          .sensorToMechRatio(5*14)
          .neutralBrake(true)
          .statorCurrentLimit(240)
          .supplyCurrentLimit(120)
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
    motorOutput.setPosition(Rotations.of(0.0));

    setpoint = Rotations.mutable(0);
    voltageSet = false;
    voltageOut = Volts.mutable(0);

    feedback = config.feedbackControllerConfig().createPIDController();
    feedforward = config.feedforwardControllerConfig().createSimpleMotorFeedforward();

    pigeon = new Pigeon2(0, "swerve");
    omega = pigeon.getAngularVelocityZWorld();
    previousOmega = RotationsPerSecond.of(0.0);

    LimelightHelpers.setCameraPose_RobotSpace("limelight-turret", 0.146, -0.219075, 0.3937, 0, 0, 0);
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

    omega.refresh();
    AngularVelocity yawVelocity = omega.getValue();
    AngularAcceleration yawAcceleration = yawVelocity.minus(previousOmega).div(RobotConstants.FAST_PERIODIC_DURATION);

    SmartDashboard.putNumber("Raw angular velocity", yawVelocity.in(RotationsPerSecond));
    SmartDashboard.putNumber("Raw angular acceleration", yawAcceleration.in(RotationsPerSecondPerSecond));

    Voltage holdVoltage = Volts.of(0.0);
    if (positionRotations > -0.5 && positionRotations < 0.5) { 
      double velocityVolts = yawVelocity.timesRatio(Volts.per(RotationsPerSecond).ofNative(config.feedforwardControllerConfig().kV())).in(Volts);
      double accelerationVolts = yawAcceleration.timesRatio(Volts.per(RotationsPerSecondPerSecond).ofNative(config.feedforwardControllerConfig().kA())).in(Volts);

      holdVoltage = Volts.of(velocityVolts + accelerationVolts);
    }

    System.out.println(holdVoltage.in(Volts));
    motorOutput.setVoltage(voltageOut.minus(holdVoltage));

    previousOmega = omega.getValue();
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
      voltageOut.mut_replace(voltageSupplier.get());
    }).finallyDo(() -> voltageSet = false);
  }

  public Command turnToTarget(Translation2d target) {
    return Commands.run(() -> {
      PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-turret");

      StructPublisher<Pose2d> posePub = NetworkTableInstance.getDefault()
        .getStructTopic("pose estimate", Pose2d.struct).publish();

      posePub.set(poseEstimate.pose);

      posePub.close();

      if (poseEstimate.tagCount > 1) {
        Angle yaw = poseEstimate.pose.getRotation().getMeasure();

        Translation2d translationToTarget = target.minus(poseEstimate.pose.getTranslation());
        Angle yawToTarget = new Rotation2d(translationToTarget.getX(), translationToTarget.getY()).getMeasure();

        double yawRotations = yaw.in(Rotations);
        double yawToTargetRotations = yawToTarget.in(Rotations);
        double diffRotations = (yawToTargetRotations - yawRotations + 0.5) % 1 - 0.5;

        Angle yawError = Rotations.of((diffRotations < -0.5) ? diffRotations + 1 : diffRotations);

        if (yawError.abs(Rotations) > 0.005) {
          Angle newSetpoint = Rotations.of(MathUtil.clamp(motorValues.position.plus(yawError).in(Rotations), -0.5, 0.5));
          setSetpoint(newSetpoint);
        }
      }
    });
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
