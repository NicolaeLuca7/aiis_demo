package eu.qrobotics.centerstage.teamcode.cv;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PointF;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.google.mlkit.vision.common.InputImage;
import com.google.mlkit.vision.pose.PoseDetection;
import com.google.mlkit.vision.pose.PoseDetector;
import com.google.mlkit.vision.pose.PoseLandmark;
import com.google.mlkit.vision.pose.defaults.PoseDetectorOptions;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

@Config
public class PoseProcessor implements VisionProcessor {

    public static double verticalTarget=0.3;

    public double width=0;
    public double height=0;

    public Boolean hasData=false;

    private PoseDetectorOptions options;
    private PoseDetector poseDetector;

    private List<PoseLandmark> landmarks=null;

    private Vector2d centerPose=null;

    private boolean processing=false;

    @Override
    public void init(int width, int height, CameraCalibration calib) {
        this.width=width;
        this.height=height;
        options =
                new PoseDetectorOptions.Builder()
                        .setDetectorMode(PoseDetectorOptions.CPU_GPU)//Maybe SINGLE_IMAGE_MODE
                        .build();
        poseDetector =  PoseDetection.getClient(options);
    }

    @Override
    public Object processFrame (Mat input, long captureTimeNanos) {
        Bitmap bitmap = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(input, bitmap);
        InputImage image = InputImage.fromBitmap(bitmap, 0);

        if(!processing) {
            processing=true;
            poseDetector.process(image).addOnSuccessListener(
                            pose -> {
                                landmarks = pose.getAllPoseLandmarks();
                                if (landmarks != null && landmarks.size() == 33) {
                                    hasData = true;
                                    List<Vector2d> points = new ArrayList<>();
                                    points.add(new Vector2d(landmarks.get(12).getPosition().x, landmarks.get(12).getPosition().y));
                                    points.add(new Vector2d(landmarks.get(11).getPosition().x, landmarks.get(12).getPosition().y));
                                    points.add(new Vector2d(landmarks.get(23).getPosition().x, landmarks.get(12).getPosition().y));
                                    points.add(new Vector2d(landmarks.get(24).getPosition().x, landmarks.get(12).getPosition().y));
                                    PoseLandmark rightShoulder = pose.getPoseLandmark(PoseLandmark.RIGHT_SHOULDER);
                                    PoseLandmark leftHip = pose.getPoseLandmark(PoseLandmark.LEFT_HIP);
                                    centerPose = new Vector2d((rightShoulder.getPosition().x+leftHip.getPosition().x)/2,(rightShoulder.getPosition().y+leftHip.getPosition().y)/2);
                                }
                                else{
                                    hasData = false;
                                    landmarks = null;
                                    centerPose = null;
                                }
                                processing = false;
                            })
                    .addOnFailureListener(
                            e -> {
                                hasData = false;
                                landmarks = null;
                                centerPose = null;
                                processing=false;
                            });
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if(!hasData) {
            return;
        }

        Paint paint=new Paint();
        paint.setColor(Color.RED);

        for(PoseLandmark landmark:landmarks){
            PointF pose=landmark.getPosition();
            canvas.drawCircle(pose.x,pose.y,3,paint);
        }

        paint.setColor(Color.YELLOW);
        canvas.drawCircle((float)centerPose.getX(),(float)centerPose.getY(),30,paint);
    }

    public void close(){
        poseDetector.close();
    }


    private Vector2d computeCenterPose(List<Vector2d> points){
        int i;
        double sumx=0,sumy=0;
        for(i=1;i< points.size();i++){
            sumx-=(points.get(i).getX()-points.get(i-1).getX())*(points.size()-i);
            sumy-=(points.get(i).getY()-points.get(i-1).getY())*(points.size()-i);
        }
        sumx/=points.size();
        sumy/=points.size();
        return new Vector2d(sumx,sumy);
    }

    public Vector2d getCenterPose(){return centerPose;}

    public double getVerticalDistance(){
        double dist=pointDist(landmarks.get(24).getPosition(),landmarks.get(12).getPosition());
        return dist;
    }

    public double getVerticalError(){
        return verticalTarget*height-getVerticalDistance();
    }

    public double getHorizontalError(){
        double dist=width/2-centerPose.getX();
        return dist;
    }

    public double pointDist(PointF p1,PointF p2){
        return Math.sqrt(Math.pow(p1.x-p2.x,2)+Math.pow(p1.y-p2.y,2));
    }

}
