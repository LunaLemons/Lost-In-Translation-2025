/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR1
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot;

 import edu.wpi.first.apriltag.AprilTagFieldLayout;
 import edu.wpi.first.apriltag.AprilTagFields;
 import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.controller.SimpleMotorFeedforward;
 import edu.wpi.first.math.geometry.Rotation3d;
 import edu.wpi.first.math.geometry.Transform3d;
 import edu.wpi.first.math.geometry.Translation2d;
 import edu.wpi.first.math.geometry.Translation3d;
 import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
 import edu.wpi.first.math.util.Units;
 
 public class Constants {

        /** Robot length, mm */
        public static final double ROBOT_X = 408;
        /** Robot width, mm */
        public static final double ROBOT_Y = 808;


        public static class Vision {
         // please remember to change this before comp (meow)
         public static final String kCameraName = "Arducam_OV9782_USB_Camera (1)";
         
         // measurements courtesy of Abel, of questionable accuracy
         public static final Transform3d kRobotToCam =
                 new Transform3d(new Translation3d(0.2875, 0.021, 0.25), new Rotation3d(0, 0, 0));
        

        public static final Transform3d kRobotToCam2 =
                 new Transform3d(new Translation3d(0.310, 0.208, 0.175), new Rotation3d(0, 0, 0));
 
         // The layout of the AprilTags on the field
         public static final AprilTagFieldLayout kTagLayout =
                 AprilTagFields.kDefaultField.loadAprilTagLayoutField();
 
         // The standard deviations of our vision estimated poses, which affect correction rate
         // (Fake values. Experiment and determine estimation noise on an actual robot.)
         public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
         public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

         
     }
    }