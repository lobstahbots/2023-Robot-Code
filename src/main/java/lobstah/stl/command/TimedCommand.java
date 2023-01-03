
package lobstah.stl.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * A wrapper that runs a command for a certain amount of time.
 */
public class TimedCommand extends ParallelDeadlineGroup {

  /**
   * Creates a new TimedCommand.
   *
   * @param command The command to run.
   * @param timeout The amount of time to run the command for.
   */
  public TimedCommand(double seconds, Command command) {
    super(new WaitCommand(seconds), command);
  }
}
