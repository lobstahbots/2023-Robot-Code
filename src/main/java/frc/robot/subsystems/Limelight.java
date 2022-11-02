
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final NetworkTable table;
  private final NetworkTableEntry tx;
  private final NetworkTableEntry ty;
  private final NetworkTableEntry tvert;
  private final NetworkTableEntry tv;
  private final NetworkTableEntry led;

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tvert = table.getEntry("tvert");
    tv = table.getEntry("tv");
    led = table.getEntry("ledMode");
  }

  public double getTx() {
    return tx.getDouble(0.0);
  }

  public double getTy() {
    return ty.getDouble(0.0);
  }

  public boolean hasTarget() {
    return tv.getDouble(0.0) == 1;
  }

  public double getTargetHeight() {
    return tvert.getDouble(0.0);
  }

  public void setLEDOff() {
    led.setNumber(1);
  }

  public void setLEDOn() {
    led.setNumber(3);
  }


}
