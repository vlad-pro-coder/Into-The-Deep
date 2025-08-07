package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.RangeSensor;

@TeleOp
@Config
public class TestDistance extends LinearOpMode {

    RangeSensor range;

    @Override
    public void runOpMode() throws InterruptedException {

        range = hardwareMap.get(RangeSensor.class,"Range");

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("distance",range.getDist());
            telemetry.update();
        }
    }
}
