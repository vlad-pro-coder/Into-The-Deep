package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ControllersRelated.SimplyfiedControllers;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class GamepadTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);
        SimplyfiedControllers gm1 = new SimplyfiedControllers(gamepad1);

        while(opModeInInit()){

            RobotInitializers.Dashtelemetry.addData("true cross",gamepad1.cross);

            RobotInitializers.Dashtelemetry.addData("cross",gm1.cross);
            RobotInitializers.Dashtelemetry.addData("cross was pressed",gm1.crossWasPressed());
            RobotInitializers.Dashtelemetry.addData("held cross",gm1.IsCrossHeld());

            RobotInitializers.Dashtelemetry.update();
            gm1.update();
        }

        waitForStart();

        while(opModeIsActive()){

        }
    }
}
