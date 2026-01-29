package frc.lib.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Subsystem;

/** Command that runs full sysid (quasistatic and dynamic forwards and reverse) */
public class runFullSysId extends SequentialCommandGroup {
  
  /**
   * Constructor for runFullSysId command that runs quasistatic and dynamic tests forwards and backwards sequentially
   * 
   * @param quasistatic function that returns quasistatic sysid command given test direction
   * @param dynamic function that returns dynamic sysid command given test direction
   * @param settledCondition boolean supplier that should return true when the next test should be run, like when the motor is at a low enough speed (will wait 1 second after this returns true to run the next test)
   * @param subsystems subsystems to require
   */
  public runFullSysId(
      Function<SysIdRoutine.Direction, Command> quasistatic, 
      Function<SysIdRoutine.Direction, Command> dynamic,
      BooleanSupplier settledCondition,
      Subsystem... subsystems) {

    addRequirements(subsystems);
    addCommands(
      quasistatic.apply(SysIdRoutine.Direction.kForward),
      Commands.waitUntil(settledCondition),
      Commands.waitSeconds(1),
      quasistatic.apply(SysIdRoutine.Direction.kReverse),
      Commands.waitUntil(settledCondition),
      Commands.waitSeconds(1),
      dynamic.apply(SysIdRoutine.Direction.kForward),
      Commands.waitUntil(settledCondition),
      Commands.waitSeconds(1),
      dynamic.apply(SysIdRoutine.Direction.kReverse)
    );
  }
}
