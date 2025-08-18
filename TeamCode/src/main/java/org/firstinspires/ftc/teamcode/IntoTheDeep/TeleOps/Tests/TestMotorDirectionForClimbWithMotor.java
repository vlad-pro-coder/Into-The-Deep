package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.PtoAndWheelie;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;
import org.intellij.lang.annotations.JdkConstants;

@TeleOp
@Config
public class TestMotorDirectionForClimbWithMotor extends LinearOpMode {

    public static double power = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);


        waitForStart();

        while (opModeIsActive()){
            RobotInitializers.clearCache();
            PtoAndWheelie.powerToChassis(power);
        }
    }
}
