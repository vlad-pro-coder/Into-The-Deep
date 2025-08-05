package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@TeleOp
public class TestLocalization extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            RobotInitializers.Dashtelemetry.addData("x", Localizer.getCurrentPosition().x);
            RobotInitializers.Dashtelemetry.addData("y", Localizer.getCurrentPosition().y);
            RobotInitializers.Dashtelemetry.addData("h", Localizer.getCurrentPosition().h);

            Localizer.Update();
            RobotInitializers.Dashtelemetry.update();
        }
    }
}
