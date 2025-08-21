package org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_startingpos;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake.OverHeadTakeSampPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LynxModuleImuType;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ControllersRelated.SimplyfiedControllers;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.TeleopsStarter;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.CachedMotor;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.RGBsensor;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.IMUBNO085;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.LimitSwitch;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.RangeSensor;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.ServoPlus;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.PtoAndWheelie;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;

import java.util.List;

@Config
public class RobotInitializers {
    public static List<LynxModule> hubs;

    public static Telemetry Dashtelemetry;
    public static IMU imu;
    public static DcMotorController ControlHubMotors, ExpansionHubMotors;
    public static ServoController ControlHubServos, ExpansionHubServos, ServoHub;
    public static DigitalChannelController ControlHubDigital,ExpansionHubDigital;
    public static double VOLTAGE = 12;
    public static boolean isDisabled(){
        return !hubs.get(0).isEngaged();
    }
    public static void disable(){
        for(LynxModule l : hubs){
            l.disengage();
        }
    }
    public static void enable(){
        for(LynxModule l : hubs){
            l.engage();
        }
    }
    public static void enableDashTelemetry(){
        Dashtelemetry = FtcDashboard.getInstance().getTelemetry();
    }
    public static void InitializeHubs(HardwareMap hm){
        InitializeHubs(hm, false);
    }

    public static void InitializeHubs(HardwareMap hm, boolean b){
        enableDashTelemetry();
        if(hubs != null) return;
        hubs = hm.getAll(LynxModule.class);
//        if(b)
//            disable();

        ControlHubMotors = hm.get(DcMotorController.class, "Control Hub");
        ExpansionHubMotors = hm.get(DcMotorController.class, "Expansion Hub 2");

        ControlHubServos = hm.get(ServoController.class, "Control Hub");
        ExpansionHubServos = hm.get(ServoController.class, "Expansion Hub 2");

        ControlHubDigital = hm.get(DigitalChannelController.class,"Control Hub");
        ExpansionHubDigital = hm.get(DigitalChannelController.class,"Expansion Hub 2");

        IMUBNO085.controller = hm.get(DigitalChannelController.class, "Expansion Hub 2");
        try {
            ServoHub = hm.get(ServoController.class, "Servo Hub 1");
        } catch (Exception ignored){
            ServoHub = ControlHubServos;
        }
        MotorConfigurationType mct;

        for(int i = 0; i < 4; i++){
            mct = ControlHubMotors.getMotorType(i);
            mct.setAchieveableMaxRPMFraction(1);

            ControlHubMotors.setMotorType(i, mct);

            mct = ExpansionHubMotors.getMotorType(i);
            mct.setAchieveableMaxRPMFraction(1);

            ExpansionHubMotors.setMotorType(i, mct);
        }


//        imu = hm.get(IMUBNO085.class, "extIMU");
        imu = hm.get(IMU.class, "imuProst");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));
//        Orientation o = new Orientation(
//                AxesReference.EXTRINSIC,
//                AxesOrder.ZXY,
//                AngleUnit.DEGREES,
//                0, 0, 0, 0
//        );
//        Robot.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(o)));

        imu.resetYaw();

        VOLTAGE = hm.getAll(VoltageSensor.class).get(0).getVoltage();

        for(LynxModule l : hubs){
            l.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    public static void clearCache(){
        clearCache(true);
    }
    public static long loopTime = 0;
    private static ElapsedTime logFreq = new ElapsedTime();
    public static void clearCache(boolean update){
        for(LynxModule l : hubs){
            l.clearBulkCache();
        }
        if(update)
            Dashtelemetry.update();
        if(logFreq.seconds() >= 1) {
            RobotLog.ii("frequency", String.valueOf(1000.f / (System.currentTimeMillis() - loopTime)));
            logFreq.reset();
        }
        loopTime = System.currentTimeMillis();
    }


    public static void InitializeFull(HardwareMap hm){
        InitializeHubs(hm, true);
        InitializeChassis();
        InitializeLocalizer(hm);
        InitializeIntake(hm);
        InitializeExtendo();
        InitializeLift();
        InitializeOuttake();
        InitializeClimbRelated();
        changeDirectionForClimb();
    }

    public static void InitializeCamera(HardwareMap hm){
        Limelight3A c = hm.get(Limelight3A.class, "camera");
        c.shutdown();
    }

    public static void changeDirectionForClimb(){
        PtoAndWheelie.FL = new CachedMotor(ControlHubMotors, 1, DcMotorSimple.Direction.FORWARD);
        PtoAndWheelie.FR = new CachedMotor(ExpansionHubMotors, 0, DcMotorSimple.Direction.FORWARD);
        PtoAndWheelie.BL = new CachedMotor(ControlHubMotors, 0, DcMotorSimple.Direction.REVERSE);
        PtoAndWheelie.BR = new CachedMotor(ExpansionHubMotors, 2, DcMotorSimple.Direction.REVERSE);
    }
    public static void InitializeChassis(){
        Chassis.FL = new CachedMotor(ControlHubMotors, 1, DcMotorSimple.Direction.FORWARD);
        Chassis.FR = new CachedMotor(ExpansionHubMotors, 0, DcMotorSimple.Direction.REVERSE);
        Chassis.BL = new CachedMotor(ControlHubMotors, 0, DcMotorSimple.Direction.FORWARD);
        Chassis.BR = new CachedMotor(ExpansionHubMotors, 2, DcMotorSimple.Direction.REVERSE);

        Chassis.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Chassis.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Chassis.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Chassis.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public static void InitializeLocalizer(HardwareMap hm){
        Localizer.Initialize(hm);
    }

    public static void InitializeIntake(HardwareMap hm){
        Intake.colorsensor = hm.get(RGBsensor.class, "Storage");
        Intake.rangesensor = hm.get(RangeSensor.class, "Range");
        Intake.dropdownServo = new ServoPlus(ControlHubServos, 1, Servo.Direction.FORWARD);
        Intake.spinner = new CachedMotor(ControlHubMotors, 3, DcMotorSimple.Direction.REVERSE);
        Intake.blocker = new ServoPlus(ControlHubServos, 2, Servo.Direction.FORWARD); // TODO: portul bun
    }

    public static void InitializeExtendo(){
        Extendo.motor = new CachedMotor(ExpansionHubMotors, 3, DcMotorSimple.Direction.FORWARD);
        Extendo.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Extendo.encoder = new CachedMotor(ExpansionHubMotors, 1, DcMotorSimple.Direction.FORWARD);
        Extendo.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Extendo.lm = new LimitSwitch(ExpansionHubDigital,1);
        MotorConfigurationType m = Extendo.motor.getMotorType();
        m.setAchieveableMaxRPMFraction(1.0);
        Extendo.motor.setMotorType(m);
    }
    public static void InitializeLift(){
        Lift.motor1 = new CachedMotor(ExpansionHubMotors, 1, DcMotorSimple.Direction.FORWARD);
        Lift.motor2 = new CachedMotor(ControlHubMotors, 2, DcMotorSimple.Direction.REVERSE);
        Lift.encoder = new CachedMotor(ControlHubMotors, 2, DcMotorSimple.Direction.REVERSE);
        Lift.lm = new LimitSwitch(ControlHubDigital,1);
        Lift.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public static void InitializeOuttake(){
        Outtake.Overheads2 = new ServoPlus(ControlHubServos, 0, Servo.Direction.REVERSE);
        Outtake.Overheads1 = new ServoPlus(ServoHub, 2, Servo.Direction.REVERSE);
        Outtake.Claw = new ServoPlus(ServoHub, 0, Servo.Direction.FORWARD);
        Outtake.Extension = new ServoPlus(ServoHub, 1, Servo.Direction.FORWARD);
    }

    public static void InitializeClimbRelated(){
        PtoAndWheelie.W1 = new ServoPlus(ServoHub, 3, Servo.Direction.FORWARD);
        PtoAndWheelie.W2 = new ServoPlus(ServoHub, 4, Servo.Direction.FORWARD);
        PtoAndWheelie.PTO = new ServoPlus(ServoHub, 5, Servo.Direction.FORWARD);
    }

    public static void InitializeForOperationTeleop(){
        Lift.state = Lift.LIFTSTATES.RETRACTING;
        Extendo.state = Extendo.ExtendoStates.RETRACTING;
        Outtake.armProfile.setInstant(OverHeadTakeSampPos-1);
        Outtake.OverHead_TAKESAMPLE();
        Outtake.setExtensionPos(0);
        Outtake.closeClaw();
        Intake.DropUp();
        Intake.Unblock();
    }

    public static void InitializeForOperationAuto(){
        Lift.state = Lift.LIFTSTATES.RETRACTING;
        Extendo.state = Extendo.ExtendoStates.RETRACTING;
        Outtake.armProfile.setInstant(OVERHEAD_startingpos-1);
        Outtake.setOverHeadPos(OVERHEAD_startingpos);
        Outtake.setExtensionPos(0);
        Outtake.closeClaw();
        Intake.DropUp();
        Intake.Unblock();
    }
}