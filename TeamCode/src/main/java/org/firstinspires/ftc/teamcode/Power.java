package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Config
public class Power extends LinearOpMode {
    DcMotor motor;
    public static double power = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class,"test");

        waitForStart();
        while(opModeIsActive()){
            motor.setPower(power);
        }
    }
}
