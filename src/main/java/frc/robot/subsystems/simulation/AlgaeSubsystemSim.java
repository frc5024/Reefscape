package frc.robot.subsystems.simulation;

import frc.lib.statemachine.StateMetadata;
import frc.robot.modules.algae.AlgaeModuleIO;
import frc.robot.modules.elevator.ElevatorVisualizer;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.utils.MapleSimUtil;

public class AlgaeSubsystemSim extends AlgaeSubsystem {
    /**
     * 
     */
    public AlgaeSubsystemSim(AlgaeModuleIO algaeModuleIO) {
        super(algaeModuleIO);
    }

    @Override
    protected void handleEject(StateMetadata<Action> stateMetadata) {
        super.handleEject(stateMetadata);

        if (stateMetadata.isFirstRun()) {
            if (MapleSimUtil.getAlgaeIntakeSimulation().obtainGamePieceFromIntake()) {
                MapleSimUtil.ejectAlgae(ElevatorVisualizer.getAlgaePose("Measured"));
            }
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
        boolean added = false;
        if (has_algae) {
            added = MapleSimUtil.getAlgaeIntakeSimulation().addGamePieceToIntake();
        }
        super.setHasAlgae(has_algae && added);
    }
}
