package org.firstinspires.ftc.teamcode.core;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.core.Pose2d;

/***
 * This function will take inputs from odometry and calculate a change in position.
 * It can also be used to calculate position using only motor encoders directly attached to the drivetrain.
 * This PDF gives a very helpful overview of the math used in odometry: https://chsftcrobotics.weebly.com/uploads/1/2/3/6/123696510/odometry.pdf
 */

public class PositionCalc {
    double trackWidth = DriveConstants.trackWidth;
    double backRadius = DriveConstants.backRadius;
    double turnRadius;
    double strafeRadius;
    double theta;
    public Pose2d calculatePose(double Ld, double Rd, double Md) {
        theta  = (Rd - Ld) / trackWidth;
        turnRadius = (trackWidth / 2 * ((Rd + Ld) / (Rd - Ld)));
        strafeRadius = (Md / theta - backRadius);
        if(theta == 0) {
            return new Pose2d(Md, (Rd + Ld) / 2, 0);
        } else {
            return new Pose2d( turnRadius * (Math.cos(theta) - 1) + strafeRadius * Math.sin(theta),turnRadius * Math.sin(theta) + strafeRadius * (1 - Math.cos(theta)), theta);
        }
    }
}
