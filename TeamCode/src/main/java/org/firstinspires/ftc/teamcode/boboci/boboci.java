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

    public double ceva = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Sasiu chassis = new Sasiu(hardwareMap);

        sasiu = new Sasiu(hardwareMap);
        while(opModeInInit())
        {
            Sasiu.malin = 4;
        }

        waitForStart();

        while (opModeIsActive()){
            sasiu.Drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.left_trigger-gamepad1.right_trigger);
        }
    }
}