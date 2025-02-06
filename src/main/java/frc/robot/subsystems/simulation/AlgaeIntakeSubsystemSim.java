package frc.robot.subsystems.simulation;

import frc.lib.statemachine.StateMetadata;
import frc.robot.modules.algae.AlgaeIntakeModuleIO;
import frc.robot.modules.elevator.ElevatorVisualizer;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.utils.MapleSimUtil;

public class AlgaeIntakeSubsystemSim extends AlgaeIntakeSubsystem {
    /**
     * 
     */
    public AlgaeIntakeSubsystemSim(AlgaeIntakeModuleIO intakeModule) {
        super(intakeModule);
    }

    @Override
    protected void handleEject(StateMetadata<Action> stateMetadata) {
        super.handleEject(stateMetadata);

        if (stateMetadata.isFirstRun()) {
            MapleSimUtil.getAlgaeIntakeSimulation().obtainGamePieceFromIntake();
            MapleSimUtil.ejectAlgae(ElevatorVisualizer.getAlgaeTransform("Measured"));
        }
    }

    @Override
    protected void handleIntake(StateMetadata<Action> stateMetadata) {
        super.handleIntake(stateMetadata);

        if (stateMetadata.isFirstRun()) {
            MapleSimUtil.getAlgaeIntakeSimulation().startIntake();
        }
    }

    @Override
    protected void handleStop(StateMetadata<Action> stateMetadata) {
        super.handleStop(stateMetadata);

        if (stateMetadata.isFirstRun()) {
            MapleSimUtil.getAlgaeIntakeSimulation().stopIntake();
        }
    }

    @Override
    public boolean hasAlgae() {
        return MapleSimUtil.getAlgaeIntakeSimulation().getGamePiecesAmount() != 0;
    }

    @Override
    public boolean hasEjected() {
        MapleSimUtil.getAlgaeIntakeSimulation().obtainGamePieceFromIntake();
        return !this.hasAlgae();
    }

    /**
     * Used in autonomous simulations
     */
    public void setHasAlgae(boolean has_algae) {
        boolean added = MapleSimUtil.getAlgaeIntakeSimulation().addGamePieceToIntake();
        super.setHasAlgae(added);
    }
}
