package eu.qrobotics.centerstage.teamcode.opmode.teleop;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Timer;

import eu.qrobotics.centerstage.teamcode.cv.PoseProcessor;
import eu.qrobotics.centerstage.teamcode.subsystems.Robot;
import eu.qrobotics.centerstage.teamcode.util.MecanumUtil;

@TeleOp
@Config
public class TrackingOpMode extends LinearOpMode {

    private PoseProcessor poseProcessor;
    private Robot robot;

    public static PIDCoefficients forwardCoeff=new PIDCoefficients(0.005,0,0);
    public static PIDCoefficients turnCoeff=new PIDCoefficients(0.0008,0,0);

    public static PIDFController forwardPID;
    public static PIDFController turnPID;

    public ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {

        MultipleTelemetry telemetry=new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        forwardPID=new PIDFController(forwardCoeff);
        turnPID=new PIDFController(turnCoeff);

        poseProcessor=new PoseProcessor();
        robot=new Robot(this, false);

        robot.start();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .addProcessor(poseProcessor)
                .enableLiveView(true)
                .build();

        telemetry.setMsTransmissionInterval(50);

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Webcam 1", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                telemetry.addData("Webcam 1", "Waiting");
                telemetry.addData("State", visionPortal.getCameraState());
                telemetry.update();
                sleep(50);
            }
            telemetry.addData("Webcam 1", "Ready");
            telemetry.update();
        }

        timer=new ElapsedTime();

        while (!isStarted()&&!isStopRequested()){
            if(poseProcessor.hasData) {
                try {
                    telemetry.addData("Center X", poseProcessor.getCenterPose().getX());
                    telemetry.addData("Center Y", poseProcessor.getCenterPose().getY());
                }
                catch (Exception e){

                }
            }
        }

        while (!isStopRequested()){
            setMotorPowers();

            if(poseProcessor.hasData) {
                try {
                    telemetry.addData("Center X", poseProcessor.getCenterPose().getX());
                    telemetry.addData("Center Y", poseProcessor.getCenterPose().getY());
                    telemetry.addData("Vertical distance",poseProcessor.getVerticalDistance());
                    telemetry.addData("Vertical target",poseProcessor.verticalTarget*poseProcessor.height);
                }
                catch (Exception e){

                }
            }
            else{
                telemetry.addLine("NO DATA");
            }
            telemetry.update();
        }

        poseProcessor.close();
        visionPortal.close();
        robot.stop();

    }

    public void setMotorPowers() {
        MecanumUtil.Motion motion;

        if(timer.milliseconds()>400){
            stopMotors();
        }

        if(!poseProcessor.hasData) {
            return;
        }
        try {

            double left_stick_x = 0;
            double left_stick_y = -forwardPID.update(poseProcessor.getVerticalError());
            double right_stick_x = turnPID.update(poseProcessor.getHorizontalError());
            double right_stick_y = 0;

            motion = MecanumUtil.joystickToMotion(left_stick_x, left_stick_y,
                    right_stick_x, right_stick_y, false, false);


            MecanumUtil.Wheels wh = MecanumUtil.motionToWheelsFullSpeed(motion).scaleWheelPower(1); // Use full forward speed on 19:1 motors
            robot.drive.setMotorPowers(wh.frontLeft, wh.backLeft, wh.backRight, wh.frontRight);
            timer.reset();
        }catch (Exception e){
        }
    }

    void stopMotors(){
        robot.drive.setMotorPowers(0,0,0,0);
    }

}
