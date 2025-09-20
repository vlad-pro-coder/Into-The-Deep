package org.firstinspires.ftc.teamcode.boboci;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class PowerMotor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor m = hardwareMap.get(DcMotor.class,"ceva");

        waitForStart();

        while(opModeIsActive()){
            m.setPower(-gamepad1.right_stick_y);
        }
    }
}
