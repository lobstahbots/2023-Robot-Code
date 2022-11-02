
package lobstahbots.stl.io;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * An extension of the {@link GenericHID} class that adds some useful utilities.
 */
public class LobstahBotsController extends GenericHID {

  /**
   * Creates a new LobstahBotsController.
   *
   * @param port The port index on the Driver Station that the device is plugged into.
   */
  public LobstahBotsController(int port) {
    super(port);
  }

  /**
   * Creates a new {@link JoystickButton} from this controller.
   *
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int)})
   * @return The new {@link JoystickButton}
   */
  public JoystickButton button(int buttonNumber) {
    return new JoystickButton(this, buttonNumber);
  }
}
