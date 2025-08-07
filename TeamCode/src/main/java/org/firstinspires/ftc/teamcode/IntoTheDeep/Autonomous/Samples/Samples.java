package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions.AfterThirdSampleGoToSubmersible.AfterThirdSampleGoToSubmersibleActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions.TakeSubmersibleSampleToHighBasket.GoingBackAndForthToHighBasketAndSubmersibleActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.CHASSIS_sample1pos;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.CHASSIS_sample2pos;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.CHASSIS_sample3pos;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENDO_sample1;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENDO_sample2;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENDO_sample3;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_idle;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.FOURpreloadsLogic.PutLastSamplePredefined.PutLastSamplePredefinedActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.FOURpreloadsLogic.SimultaniosSampleGrabingScoring.SimultaniosSampleGrabingScoringActions;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@Autonomous
@Config
public class Samples extends LinearOpMode {
    public enum RobotStates{
        OCCUPIEDPRELOADS,
        DETECTIONGRAB,
        FAILEDTOGRABFROMSUBMERSIBLE,
        SUBMERSIBLESAMPLEDOCYCLE,
    }
    public boolean WereInstructionGiven = false;
    public RobotStates robotstate = RobotStates.OCCUPIEDPRELOADS;
    public boolean everpurepersuit = false;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);
        RobotInitializers.InitializeForOperation();
        Scheduler tasks = new Scheduler();
        /*tasks.AddAnotherScheduler(SimultaniosSampleGrabingScoringActions(CHASSIS_sample1pos,EXTENDO_sample1,1));
        tasks.AddAnotherScheduler(SimultaniosSampleGrabingScoringActions(CHASSIS_sample2pos,EXTENDO_sample2,1));
        tasks.AddAnotherScheduler(SimultaniosSampleGrabingScoringActions(CHASSIS_sample3pos,EXTENDO_sample3,1));
        tasks.AddAnotherScheduler(PutLastSamplePredefinedActions);*/
        tasks.AddAnotherScheduler(AfterThirdSampleGoToSubmersibleActions);
        tasks.AddAnotherScheduler(GoingBackAndForthToHighBasketAndSubmersibleActions);

        Chassis.usedTrajectory = Chassis.trajectoryStates.FREEWILL;
        Lift.DoingAuto = true;
        Extendo.DoingAuto = true;

        while(opModeInInit()){
            RobotInitializers.clearCache(false);
            Outtake.setOverHeadPos(OVERHEAD_idle);
            Intake.Block();
            Outtake.openClaw();

            Lift.update();
            Extendo.update();
            Outtake.update();

        }

        waitForStart();

        while(opModeIsActive())
        {
            RobotInitializers.clearCache();

            RobotInitializers.Dashtelemetry.addData("is Scheduler done", tasks.tasks.size());

            RobotInitializers.Dashtelemetry.addData("location done",Chassis.IsPositionDone(500));
            RobotInitializers.Dashtelemetry.addData("angle done",Chassis.IsHeadingDone(30));

            Chassis.update();
            Lift.update();
            Extendo.update();
            Outtake.update();
            tasks.update();
            Localizer.Update();

        }

    }
}
