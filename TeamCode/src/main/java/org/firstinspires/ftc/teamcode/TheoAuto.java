package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="TheleAuto")
public class TheoAuto extends LinearOpMode {

    String PolePOS = "";
    PolePipeline detector;
    PipelineNew detector2;
    private OpenCvCamera webcam;
    private DcMotor motor1 = null;

    SampleMecanumDrive drive;
    enum autoStage {
        one,
        two
    }
    public double speedmulti = (double) 0.2;
    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware rh = new RobotHardware();
        rh.initialize(hardwareMap);
        autoStage stage = autoStage.one;
        //set to starting positon and angle.toCaps()
        Pose2d startPose = new Pose2d(-36, -36, Math.toRadians(0));
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);
        OpenCvWebcam webcam;
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "Webcam 1"), cameraMonitorViewId);
        PolePipeline detector = new PolePipeline(telemetry);
        webcam.setPipeline(detector2);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        Trajectory traj1 = drive.trajectoryBuilder(startPose, false)
        .forward(24)
        .build();
        Trajectory traj11 = drive.trajectoryBuilder(startPose, false)
                .strafeRight(12)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), false)
        .back(6)
        .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end().plus(new Pose2d(0, 0, Math.toRadians(90))), false)
        .back(24)
        .build();
        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d())
        .strafeRight(6)
        .build();
        waitForStart();
        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d())
        .strafeLeft(24)
        .build();
        waitForStart();
        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d())
        .strafeRight(24)
        .build();
        waitForStart();

        PipelineNew.ParkingPosition parkingPosition = detector2.getPosition();
        sleep(1000);
        webcam.setPipeline(detector);
       // dashboard = FtcDashboard.getInstance();

            drive.followTrajectory(traj1);
        drive.followTrajectory(traj11);
            drive.followTrajectory(traj4);
            for (double i = 1; i < 5; i++){
            while (stage == autoStage.one){

            lineuppole:
            while (true){
            switch (detector.getCoords()) {
                case LEFT:
                    goRight();
                    PolePOS = "left";
                    break;
                case RIGHT:
                    goLeft();
                    // ... turn left
                    PolePOS = "right";
                    break;
                case CENTER:
                    // ...break
                    goStop();
                    PolePOS = "Perfect";
                    break lineuppole;

            }
        }

            polewidther:
            while (true){
            switch (detector.getWidth()) {
                case LEFT:
                    //  turn right
                    goforward();

                    break;
                case RIGHT:
                    // ... turn left
                    goBack();
                    break;
                case CENTER:
                    // ...break
                    goStop();
                    stage = autoStage.two;
                    break polewidther;

            }
            sertSlide(0.5,1000);
            sertBar(1.0, (int) (rh.ticks_in_degree * 180));
            sleep(1500);
          //  robot.claw.setPower(1);
            sleep(500);
           // robot.claw.setPower(0);
            //TODO:slide up
            //TODO: find the encoders per inch for linear slide... NOW
            //bar up
            //drop and reset
            sertSlide(-0.5,000);
            sertBar(-1,000);
        }
    } while (stage == autoStage.two){

        if ( i == 1){
            drive.followTrajectory(traj2);
            drive.turn(Math.toRadians(-90));
    }
        else if (i > 1){
            drive.followTrajectory(traj4);
        }
                        sertSlide(0.5, (int) (1000 - (i * 20                                           )));
        drive.followTrajectory(traj3);
        //robot.claw.setPower(1);
        //TODO:set slide and grab

        stage = autoStage.one;
    }
}

    switch (parkingPosition) {
        case LEFT:
            // ...
            drive.followTrajectory(traj5);
            break;
        case RIGHT:
            // ...
            drive.followTrajectory(traj6);
            stop();
            break;
        case CENTER:
            /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            stop();
    }
    webcam.stopStreaming();
        //handleDashboard();
        telemetry.addData("PolePOS", PolePOS);
        }
        /*
    private void handleDashboard() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Frame Count", webcam.getFrameCount());
        packet.put("FPS", String.format("%.2f", webcam.getFps()));
        packet.put("Total frame time ms", webcam.getTotalFrameTimeMs());
        packet.put("Pipeline time ms", webcam.getPipelineTimeMs());
        packet.put("Overhead time ms", webcam.getOverheadTimeMs());
        packet.put("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

        dashboard.sendTelemetryPacket(packet);
    }
    */
    private void goLeft(){

        drive.setMotorPowers(-speedmulti, speedmulti, -speedmulti, speedmulti);
    }
    private void goRight(){
        drive.setMotorPowers(speedmulti, -speedmulti, speedmulti, -speedmulti);
    }
    private void goStop(){
        drive.setMotorPowers(0, 0, 0, 0);
    }
    private void goforward(){
        drive.setMotorPowers(speedmulti, speedmulti, speedmulti, speedmulti);
    }
    private void goBack(){
        drive.setMotorPowers(-speedmulti, -speedmulti, -speedmulti, -speedmulti);
    }
    private void sertSlide(double slidePower, int slideTop){
   /**     robot.lSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lSlide1.setTargetPosition(slideTop);
            robot.lSlide2.setTargetPosition(slideTop);
            robot.lSlide1.setPower(slidePower);
            robot.lSlide2.setPower(slidePower); **/
    }
    private void sertBar(double barPower, int barTop){
       // robot.v4bar1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       //     robot.v4bar1.setTargetPosition(barTop);
        //    robot.v4bar1.setPower(barPower);
    }

}