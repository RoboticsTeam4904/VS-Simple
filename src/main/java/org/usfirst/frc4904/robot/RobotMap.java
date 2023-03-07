package org.usfirst.frc4904.robot;

import java.awt.Component;

import org.usfirst.frc4904.standard.custom.controllers.CustomCommandJoystick;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.custom.motorcontrollers.TalonMotorController;
import org.usfirst.frc4904.standard.custom.sensors.NavX;
import org.usfirst.frc4904.standard.custom.sensors.PDP;
import org.usfirst.frc4904.standard.subsystems.chassis.WestCoastDrive;
import org.usfirst.frc4904.standard.subsystems.motor.TalonMotorSubsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotMap {
    public static class Port {
        public static class HumanInput {
            public static final int JOYSTICK = 0;
            public static final int XBOX_CONTROLLER = 1;
        }

        public static class CANMotor {

            public static final int LEFT_DRIVE_A = 0;
            public static final int RIGHT_DRIVE_A = 0;
            public static final int RIGHT_DRIVE_B = 0;
            public static final int LEFT_DRIVE_B = 0;
        }

        public static class PWM {
        }

        public static class CAN {
        }

        public static class Pneumatics {
        }

        public static class Digital {
        }
    }

    public static class Metrics {
        public static class Chassis {
            // 2023 constants
            public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(19.5);
            public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(5);
            public static final double MOTOR_TO_WHEEL_GEAR_RATIO = 496/45;    // https://www.desmos.com/calculator/llz7giggcf
            // public static final double TICKS_PER_REVOLUTION = -1; // TODO: CHANGE CONSTS
            // public static final double DIAMETER_INCHES = -1;
            // public static final double CIRCUMFERENCE_INCHES = Metrics.Chassis.DIAMETER_INCHES * Math.PI;
            // public static final double TICKS_PER_INCH = Metrics.Chassis.TICKS_PER_REVOLUTION
            //         / Metrics.Chassis.CIRCUMFERENCE_INCHES;
            // public static final double DISTANCE_FRONT_BACK = -1;
            // public static final double DISTANCE_SIDE_SIDE = -1;
            // public static final double INCHES_PER_TICK = Metrics.Chassis.CIRCUMFERENCE_INCHES
            //         / Metrics.Chassis.TICKS_PER_REVOLUTION;
        }

        public static class Encoders {
            public static class TalonEncoders {
                public static final double REVOLUTIONS_PER_TICK = 2048;
            }
        }
    }

    public static class DriveConstants {
        public static final boolean kGyroReversed = false;
        public static final double ksVolts = -1;
        public static final double kvVoltSecondsPerMeter = -1;
        public static final double kaVoltSecondsSquaredPerMeter = -1;
        public static final double kTrackwidthMeters = -1;  // redundant with Metrics.Chassis.TRACK_WIDTH_METERS
        public static final double kPDriveVel = -1;
    }

    public static class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = -1;
        public static final double kMaxAccelerationMetersPerSecondSquared = -1;
        public static final double kRamseteB = -1;
        public static final double kRamseteZeta = -1;
    }

    public static class PID {
        public static class Drive {
        }

        public static class Turn {
        }

    }

    public static class Component {
        public static PDP pdp;
        public static NavX navx;
        public static TalonMotorSubsystem leftDriveMotors;
        public static TalonMotorSubsystem rightDriveMotors;
        public static WestCoastDrive<TalonMotorController> Chassis;
    }

    public static class Input {
    }

    public static class HumanInput {
        public static class Driver {
            public static CommandXboxController xbox;
        }

        public static class Operator {
            public static CustomCommandJoystick joystick;
        }
    }

    /**
     * 
     */
    public RobotMap() {
        HumanInput.Driver.xbox = new CommandXboxController(Port.HumanInput.XBOX_CONTROLLER);
        HumanInput.Operator.joystick = new CustomCommandJoystick(Port.HumanInput.JOYSTICK);

        /* General */
        Component.pdp = new PDP();
        Component.navx = new NavX(SerialPort.Port.kMXP);
        /* Drive Train */

        // Wheels
        CANTalonFX rightWheelATalon = new CANTalonFX(Port.CANMotor.RIGHT_DRIVE_A, InvertType.None);
        CANTalonFX rightWheelBTalon = new CANTalonFX(Port.CANMotor.RIGHT_DRIVE_B, InvertType.FollowMaster);
        CANTalonFX leftWheelATalon  = new CANTalonFX(Port.CANMotor.LEFT_DRIVE_A, InvertType.InvertMotorOutput);
        CANTalonFX leftWheelBTalon  = new CANTalonFX(Port.CANMotor.LEFT_DRIVE_B, InvertType.FollowMaster);

        Component.leftDriveMotors  = new TalonMotorSubsystem("left drive motors",  NeutralMode.Brake, 10,  leftWheelATalon,  leftWheelBTalon);
        Component.rightDriveMotors = new TalonMotorSubsystem("right drive motors", NeutralMode.Brake, 10, rightWheelATalon, rightWheelBTalon);
        Component.Chassis = new WestCoastDrive<TalonMotorController>(Metrics.Chassis.TRACK_WIDTH_METERS, Metrics.Chassis.MOTOR_TO_WHEEL_GEAR_RATIO, Metrics.Chassis.WHEEL_DIAMETER_METERS, Component.leftDriveMotors, Component.rightDriveMotors);

        // Wheel Encoders
        // Component.leftWheelTalonEncoder = new CANTalonEncoder("Leftwheel", leftWheelATalon, true,
        //         Metrics.Encoders.TalonEncoders.REVOLUTIONS_PER_TICK, CustomPIDSourceType.kDisplacement,
        //         FeedbackDevice.IntegratedSensor);
        // Component.rightWheelTalonEncoder = new CANTalonEncoder("rightWheel", rightWheelATalon, true,
        //         Metrics.Encoders.TalonEncoders.REVOLUTIONS_PER_TICK, CustomPIDSourceType.kDisplacement,
        //         FeedbackDevice.IntegratedSensor);

        // Component.leftWheelCANCoder = new CustomCANCoder(Port.CAN.LEFT_WHEEL_ENCODER,
        //         Metrics.Chassis.CAN_CODER_METERS_PER_TICK);
        // Component.rightWheelCANCoder = new CustomCANCoder(Port.CAN.RIGHT_WHEEL_ENCODER,
        //         Metrics.Chassis.CAN_CODER_METERS_PER_TICK);

        // Component.chassisTalonEncoders = new EncoderPair(Component.leftWheelTalonEncoder, Component.rightWheelCANCoder);
        // Component.chassisCANCoders = new EncoderPair(Component.leftWheelCANCoder, Component.rightWheelCANCoder);

        // // General Chassis
        // Component.chassis = new TankDrive("Blinky-Chassis", Component.leftWheelA, Component.leftWheelB,
        //         Component.rightWheelA, Component.rightWheelB);
        // Component.chassis.setDefaultCommand(new ChassisMove(Component.chassis, new NathanGain()));
    }
}
