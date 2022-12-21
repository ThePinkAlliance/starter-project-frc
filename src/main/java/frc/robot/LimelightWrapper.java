// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ThePinkAlliance.core.limelight.Limelight;
import com.ThePinkAlliance.core.limelight.LimelightConstants;
import com.ThePinkAlliance.core.math.LinearInterpolationTable;
import edu.wpi.first.wpilibj.drive.Vector2d;

public class LimelightWrapper extends Limelight {

  List<Vector2d> points = List.of(
      new Vector2d(120, 125),
      new Vector2d(110, 115),
      new Vector2d(100, 105),
      new Vector2d(130, 135),
      new Vector2d(140, 145),
      new Vector2d(150, 155));

  LinearInterpolationTable table = new LinearInterpolationTable(points);

  public LimelightWrapper(double height_from_floor, double mounted_angle) {
    super(height_from_floor, mounted_angle);
  }

  public LimelightWrapper(double height_from_floor, double mounted_angle, double horizontal_offset) {
    super(height_from_floor, mounted_angle, horizontal_offset);
  }

  public LimelightWrapper(LimelightConstants constants) {
    super(constants);
  }

  @Override
  public double calculateDistance() {
    double distance = super.calculateDistance();

    return table.interp(distance);
  }
}
