package org.firstinspires.ftc.teamcode.boboci;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@TeleOp(name="mama mea")
public class boboci extends LinearOpMode {

    Sasiu sasiu;
    DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {


        sasiu = new Sasiu(hardwareMap);
        while(opModeInInit())
        {

        }

        waitForStart();

        while (opModeIsActive()){
            sasiu.Drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.left_trigger-gamepad1.right_trigger);
        }
    }
}