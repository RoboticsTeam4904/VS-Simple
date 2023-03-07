/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc4904.robot;


import org.usfirst.frc4904.standard.CommandRobotBase;
import org.usfirst.frc4904.standard.LogKitten;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandJoystick;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.custom.motorcontrollers.TalonMotorController;
import org.usfirst.frc4904.standard.subsystems.chassis.WestCoastDrive;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.subsystems.chassis.WestCoastDrive;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends CommandRobotBase {
  public static CommandXboxController controller;
  public static TalonMotorSubsystem leftMotors;
  public static TalonMotorSubsystem rightMotors;
  public static WestCoastDrive<TalonMotorController> drive;

  @Override
  public void initialize() {
    Robot.controller = new CommandXboxController(1);
    Robot.leftMotors  = new TalonMotorSubsystem("left  motors", NeutralMode.Brake, 10, new CANTalonFX(1, InvertType.InvertMotorOutput), new CANTalonFX(2, InvertType.FollowMaster));
    Robot.rightMotors = new TalonMotorSubsystem("right motors", NeutralMode.Brake, 10, new CANTalonFX(3, InvertType.None), new CANTalonFX(4, InvertType.FollowMaster));
    Robot.drive = new WestCoastDrive<TalonMotorController>(Units.inchesToMeters(19.5), 11, Units.inchesToMeters(5), leftMotors, rightMotors);

    Robot.leftMotors.configPIDF(0.1, 0., 0., 0., 100, 1, null);
    Robot.rightMotors.configPIDF(0.1, 0., 0., 0., 100, 1, null);
  }

  @Override
  public void teleopInitialize() {
    // TODO Auto-generated method stub
  }

  @Override
  public void teleopExecute() {
    // TODO Auto-generated method stub
  }

  @Override
  public void autonomousInitialize() {
    // TODO Auto-generated method stub
  }

  @Override
  public void autonomousExecute() {
    // TODO Auto-generated method stub
  }

  @Override
  public void disabledInitialize() {
    // TODO Auto-generated method stub
  }

  @Override
  public void disabledExecute() {
    // TODO Auto-generated method stub
  }

  @Override
  public void testInitialize() {
    // drive.c_controlChassisVelocity(() -> new ChassisSpeeds(
    //   (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis())*10,
    //   0, 
    //   controller.getLeftX()*Math.PI/2
    // )).schedule();
  }

  @Override
  public void testExecute() {
    // drive.setWheelVoltages(new DifferentialDriveWheelVoltages((controller.getRightTriggerAxis() - controller.getLeftTriggerAxis())*10, 0, controller.getRightX()*Math.PI/2));
    drive.setWheelVoltages(new DifferentialDriveWheelVoltages(
      (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis() + controller.getLeftX())*10,
      (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis() - controller.getLeftX())*10
      ));
    LogKitten.wtf(controller.getRightTriggerAxis());
  }

  @Override
  public void alwaysExecute() {
    // TODO Auto-generated method stub
  }

}