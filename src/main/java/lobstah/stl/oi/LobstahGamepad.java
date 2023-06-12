
package lobstah.stl.oi;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

/**
 * An extension of the {@link CommandGenericHID} class that adds some useful utilities.
 */
public class LobstahGamepad extends CommandGenericHID {

  /**
   * Creates a new LobstahGamepad.
   *
   * @param port The port index on the Driver Station that the device is plugged into.
   */
  public LobstahGamepad(int port) {
    super(port);
  }
}
