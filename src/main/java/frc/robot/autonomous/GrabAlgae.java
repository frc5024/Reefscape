package frc.robot.autonomous;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

/**
 * 
 */
public class GrabAlgae {
    private final AlgaeIntakeSubsystem algaeIntakeSubsystem;

    /**
     * 
     */
    public GrabAlgae(AlgaeIntakeSubsystem algaeIntakeSubsystem) {
        this.algaeIntakeSubsystem = algaeIntakeSubsystem;
    }

    /**
     * 
     */
    public Command getAutoCommand() {
        List<PathPlannerPath> pathGroup = null;

        try {

            pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("GrabAlgae");

        } catch (Exception e) {
        }

        if (pathGroup == null) {
            return Commands.print("*** Failed to build GrabAlgae");
        }

        Command command = Commands.sequence(
                Commands.print("*** Starting GrabAlgae ***"),

                AutoBuilder.followPath(pathGroup.get(0)),
                new InstantCommand(() -> {
                    this.algaeIntakeSubsystem.addAction(AlgaeIntakeSubsystem.Action.INTAKE);
                }),
                new WaitUntilCommand(this.algaeIntakeSubsystem::hasAlgae),
                AutoBuilder.followPath(pathGroup.get(1)),
                new InstantCommand(() -> {
                    this.algaeIntakeSubsystem.addAction(AlgaeIntakeSubsystem.Action.EJECT);
                }),

                Commands.print("*** Finished GrabAlgae ***"));

        return command;
    }
}
