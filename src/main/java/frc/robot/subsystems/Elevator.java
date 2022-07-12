// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  Matrix<N2, N1> MODEL_ACCURACY = VecBuilder.fill(3, 3);
  Matrix<N1, N1> ENCODER_ACCURACY = VecBuilder.fill(0.01);
  double LOOP_TIME = 0.025;

  WPI_TalonFX motor;

  double POSITION_ERROR_TOLOERANCE = 3;
  double VELOCITY_ERROR_TOLOERANCE = 3;
  double MAX_VOLTAGE = 12; // Max control effort for voltage input

  LinearSystem<N2, N1, N1> linearSystem = LinearSystemId.createElevatorSystem(
      DCMotor.getFalcon500(2),
      2.26796,
      (Units.inchesToMeters(2.36) / 2),
      (1));

  LinearQuadraticRegulator<N2, N1, N1> lqr = new LinearQuadraticRegulator<>(
      linearSystem,
      VecBuilder.fill(POSITION_ERROR_TOLOERANCE, VELOCITY_ERROR_TOLOERANCE),
      VecBuilder.fill(MAX_VOLTAGE),
      LOOP_TIME);

  KalmanFilter<N2, N1, N1> observer = new KalmanFilter<>(
      Nat.N2(),
      Nat.N1(),
      linearSystem,
      MODEL_ACCURACY,
      ENCODER_ACCURACY,
      LOOP_TIME);

  LinearSystemLoop<N2, N1, N1> controlLoop = new LinearSystemLoop<>(
      linearSystem,
      lqr,
      observer,
      12.0,
      LOOP_TIME);

  LinearSystemSim<N2, N1, N1> systemSim = new LinearSystemSim<>(linearSystem);

  ElevatorSim sim = new ElevatorSim(DCMotor.getFalcon500(2), (1 / 1), 3.5, Units.inchesToMeters(2.75),
      Units.inchesToMeters(50), Units.inchesToMeters(100));
  Encoder encoder = new Encoder(1, 2);
  EncoderSim encoder_sim = new EncoderSim(encoder);

  Mechanism2d mechanism2d = new Mechanism2d(20, 50);
  MechanismRoot2d root = mechanism2d.getRoot("Elevator Root", 10, 0);
  MechanismLigament2d elevatorMechanismLigament = root
      .append(new MechanismLigament2d("Elevator", Units.metersToInches(sim.getPositionMeters()), 90));

  /** Creates a new Elevator. */
  public Elevator() {
    motor = new WPI_TalonFX(1);
    encoder.setDistancePerPulse(2 * Math.PI * 2.75 / 4096);
    SmartDashboard.putData("Elevator Sim", mechanism2d);
  }

  public void start() {
    motor.set(ControlMode.PercentOutput, 1);
    sim.setInputVoltage(12);
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
    root.setPosition(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    sim.setInput(motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    sim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    encoder_sim.setDistance(sim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

    // Update elevator visualization with simulated position
    elevatorMechanismLigament.setLength(Units.metersToInches(sim.getPositionMeters()));
  }
}
