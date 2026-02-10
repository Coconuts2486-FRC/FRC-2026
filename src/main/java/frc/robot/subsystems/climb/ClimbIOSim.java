package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ClimbIOSim implements ClimbIO {

  private final DCMotor m_gearbox = DCMotor.getNEO(1);
  private final LinearSystem<N1, N1, N1> m_plant =
      LinearSystemId.createFlywheelSystem(m_gearbox, 1.0, 1.0);

  private final FlywheelSim sim = new FlywheelSim(m_plant, m_gearbox);
  private PIDController pid = new PIDController(0, 0, 0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
      sim.setInputVoltage(appliedVolts);
    }
  }
}
