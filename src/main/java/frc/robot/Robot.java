// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private Drivetrain drivetrain = new Drivetrain();
  private TestAuton testAuto = new TestAuton(drivetrain);

  // Handle robot simulation
  private final DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
    DCMotor.getCIM(2),
    7.29,
    7.5,
    60,
    Units.inchesToMeters(3),
    Constants.Drivetrain.kTrackWidth,
    null);

  private Field2d field2d = new Field2d();

  // trajectory constraints
  private final RamseteController ramsete = new RamseteController();
  private final Timer timer = new Timer();
  private Trajectory trajectory;

  DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.Drivetrain.ksVolts,
            Constants.Drivetrain.kvVoltSecondsPerMeter,
            Constants.Drivetrain.kaVoltSecondsSquaredPerMeter),
            Constants.Drivetrain.kinematics,
            10);

  // Create config for trajectory
  TrajectoryConfig config =
  new TrajectoryConfig(Constants.Drivetrain.kMaxSpeedMetersPerSecond,
                        Constants.Drivetrain.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(Constants.Drivetrain.kinematics)
      // Apply the voltage constraint
      .addConstraint(autoVoltageConstraint);

  @Override
  public void robotInit() {
    SmartDashboard.putData("Field", field2d);

    setNetworkTablesFlushEnabled(true);

    trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(2, 2, new Rotation2d()),
            List.of(),
            new Pose2d(6, 2, new Rotation2d()),
            config);

    List<Pose2d> poses = new ArrayList<>();
    for (Trajectory.State state : trajectory.getStates()) {
      poses.add(state.poseMeters);
    }

    field2d.getObject("foo").setPoses(poses);
  }

  @Override
  public void robotPeriodic() {
    drivetrain.updateOdometry();
    SmartDashboard.putNumber("LeftMotor", drivetrain.getLeftMotor());
    SmartDashboard.putNumber("RightMotor", drivetrain.getRightMotor());
    SmartDashboard.putNumber("Robot X", drivetrain.getOdometry().getPoseMeters().getX());
    SmartDashboard.putNumber("Robot Y", drivetrain.getOdometry().getPoseMeters().getY());
    field2d.setRobotPose(drivetrain.getOdometry().getPoseMeters());
  }

  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
    drivetrain.resetOdometry(trajectory.getInitialPose());
  }

  @Override
  public void autonomousPeriodic() {
    var elapsed = timer.get();
    Trajectory.State reference = trajectory.sample(elapsed);
    ChassisSpeeds speeds = ramsete.calculate(drivetrain.getOdometry().getPoseMeters(), reference);
    SmartDashboard.putNumber("Expected X", reference.poseMeters.getX());
    drivetrain.setSpeeds(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    drivetrain.drive();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    drivetrain.setOutput(0, 0);
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    driveSim.setInputs(drivetrain.getLeftMotor() * RobotController.getInputVoltage(),
                       drivetrain.getRightMotor() * RobotController.getInputVoltage());

    driveSim.update(0.02);

    drivetrain.simulationPeriodic(driveSim);

    var navxSim = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    var angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(navxSim, "Yaw"));
    angle.set(-driveSim.getHeading().getDegrees());
  }
}
