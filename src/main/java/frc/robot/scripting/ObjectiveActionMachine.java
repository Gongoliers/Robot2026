package frc.robot.scripting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Function;

public class ObjectiveActionMachine {

    public static List<Objective> walk(Objective initial, Action[] actions) {
        ArrayList<Objective> objectives = new ArrayList<>();
        // NOTE: The initial objective is never present in the final objectives
        Objective objective = initial;
        for (Action action : actions) {
            if (action == null) {
                continue;
            }
            objective = objective.transition(action);
            objectives.add(objective);
        }
        return objectives;
    }

}
