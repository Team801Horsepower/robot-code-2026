// Copyright (c) 2026 Team 801 Horsepower
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GatherConstants;

public class Launch extends SubsystemBase {
  private final Spindex m_spindex;
  private final Feeder  m_feeder;
  private final Gather  m_gather;

  public Launch(Spindex spindex, Feeder feeder, Gather gather) {
    m_spindex = spindex;
    m_feeder  = feeder;
    m_gather  = gather;
  }

  /** Runs spindexer + feeder + reverse gather to launch game pieces. */
  public void launch() {
    m_spindex.spin();
    m_feeder.spin();
    m_gather.gather(GatherConstants.kDefaultPower * 0.5);
  }

  /** Stops spindexer and feeder. */
  public void stop() {
    m_spindex.rest();
    m_feeder.rest();
  }
}
