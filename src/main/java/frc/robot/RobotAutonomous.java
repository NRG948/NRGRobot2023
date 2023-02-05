package frc.robot;

import java.util.EnumSet;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class creates and manages the user interface operators use to select
 * and configure autonomous routines.
 */
public class RobotAutonomous {

    /**
     * An enum representing the delay, in seconds, before the autonomous routine
     * starts. This can be used to avoid collisions with alliance robots when we
     * know their autonomous path would cross ours.
     */
    private static enum ChooseAutoDelay {
        NO_DELAY(0),
        DELAY_2_SECONDS(2),
        DELAY_5_SECONDS(5);

        private int delay;

        /**
         * Creates a new ChooseAuthoDelay.
         * 
         * @param delay The delay in seconds.
         */
        ChooseAutoDelay(int delay) {
            this.delay = delay;
        }

        /**
         * Returns the delay in seconds.
         * 
         * @return The delay in seconds.
         */
        public int getDelay() {
            return delay;
        }

        /**
         * Returns the name of this constant.
         */
        @Override
        public String toString() {
            return String.format("%d seconds", delay);
        }
    }

    private static SendableChooser<ChooseAutoDelay> chooseAutoDelay = new SendableChooser<>();

    /**
     * Returns a {@link WaitCommand} used to wait before starting the autonomous routine.
     * 
     * @return A {@link WaitCommand} that waits for the selected delay.
     */
    private static Command getSelectedDelayCommand() {
        ChooseAutoDelay delayChoice = chooseAutoDelay.getSelected();
        return new WaitCommand(delayChoice.getDelay());
    }

    /**
     * Adds the autonomous layout to the Shuffleboard tab.
     * 
     * @param layout The layout to add user interface elements.
     */
    public static void addShuffleboardLayout(ShuffleboardLayout layout) {
        for (ChooseAutoDelay delay : EnumSet.allOf(ChooseAutoDelay.class)) {
            chooseAutoDelay.addOption(delay.toString(), delay);
        }

        chooseAutoDelay.setDefaultOption(ChooseAutoDelay.NO_DELAY.toString(), ChooseAutoDelay.NO_DELAY);

        layout.add("Delay", chooseAutoDelay);
    }
}
