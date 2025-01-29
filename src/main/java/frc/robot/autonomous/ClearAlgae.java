package frc.robot.autonomous;

import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

/**
 * 
 */
public class ClearAlgae {
    private final AlgaeIntakeSubsystem algaeIntakeSubsystem;

    /**
     * 
     */
    public ClearAlgae(AlgaeIntakeSubsystem algaeIntakeSubsystem) {
        this.algaeIntakeSubsystem = algaeIntakeSubsystem;
    }

    /**
     * 
     */
    public Command getAutoCommand() {
        List<PathPlannerPath> pathGroup = null;

        try {

            pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("ClearAlgae");

        } catch (Exception e) {
        }

        if (pathGroup == null) {
            return Commands.print("*** Failed to build ClearAlgae");
        }

        Command command = Commands.sequence(
                Commands.print("*** Starting ClearAlgae ***"),

                AutoBuilder.followPath(pathGroup.get(0)),
                // new WaitUntilCommand(3), // Drop Coral
                new ParallelCommandGroup(
                        AutoBuilder.followPath(pathGroup.get(1))
                /* Move Elevator to Low Position */
                ),
                new InstantCommand(() -> {
                    this.algaeIntakeSubsystem.addAction(AlgaeIntakeSubsystem.Action.INTAKE);
                }),
                new WaitUntilCommand(this.algaeIntakeSubsystem::hasAlgae),
                new ParallelCommandGroup(
                        AutoBuilder.followPath(pathGroup.get(2)),
                        new InstantCommand(() -> {
                            this.algaeIntakeSubsystem.addAction(AlgaeIntakeSubsystem.Action.EJECT);
                        })),
                new InstantCommand(() -> {
                    this.algaeIntakeSubsystem.addAction(AlgaeIntakeSubsystem.Action.INTAKE);
                }),

                Commands.print("*** Finished ClearAlgae ***"));

        return command;
    }
}
