// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  Matrix<N2, N1> MODEL_ACCURACY = VecBuilder.fill(3, 3);
  Matrix<N1, N1> ENCODER_ACCURACY = VecBuilder.fill(0.01);
  double LOOP_TIME = 0.025;

  double VELOCITY_ERROR_TOLOERANCE = 3;
  double CONTROL_EFFORT_TOLOERANCE = 3;

  LinearSystem<N2, N1, N1> linearSystem = LinearSystemId.createElevatorSystem(
    DCMotor.getFalcon500(2),
    2.26796,
    (Units.inchesToMeters(1.5) / 2),
    (1 / 1)
  );

  LinearQuadraticRegulator<N2, N1, N1> regulator = new LinearQuadraticRegulator<>(
    linearSystem,
    VecBuilder.fill(VELOCITY_ERROR_TOLOERANCE, VELOCITY_ERROR_TOLOERANCE),
    VecBuilder.fill(CONTROL_EFFORT_TOLOERANCE),
    LOOP_TIME
  );

  KalmanFilter<N2, N1, N1> observer = new KalmanFilter<>(
    Nat.N2(),
    Nat.N1(),
    linearSystem,
    MODEL_ACCURACY,
    ENCODER_ACCURACY,
    LOOP_TIME
  );

  LinearSystemLoop<N2, N1, N1> controlLoop = new LinearSystemLoop<>(
    linearSystem,
    regulator,
    observer,
    12.0,
    LOOP_TIME
  );

  /** Creates a new Elevator. */
  public Elevator() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
