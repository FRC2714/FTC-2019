package org.firstinspires.ftc.teamcode.localization;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.ArrayList;
import java.util.List;

public abstract class TwoTrackingWheelLocalizer {
    private static final double TAU = Math.PI * 2;
    private Pose2d poseEstimate;
    private List<Double> lastWheelPositions;
    private double lastHeading;

    private DecompositionSolver forwardSolver;


    public TwoTrackingWheelLocalizer(List<Pose2d> wheelPoses) {
        lastWheelPositions = new ArrayList<>();
        lastHeading = Double.NaN;

        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3,3);

        for (int i = 0; i < 2; i++) {
            Vector2d orientationVector = wheelPoses.get(i).headingVec();
            Vector2d positionVector = wheelPoses.get(i).vec();
            inverseMatrix.setEntry(i, 0, orientationVector.x);
            inverseMatrix.setEntry(i, 1, orientationVector.y);
            inverseMatrix.setEntry(i, 2, positionVector.x * orientationVector.y - positionVector.y * orientationVector.x);
        }

        inverseMatrix.setEntry(2, 2, 1.0);
        forwardSolver = new LUDecomposition(inverseMatrix).getSolver();
    }

    public void update() {
        List<Double> wheelPositions = getWheelPositions();
        double heading = getHeading();
        if (lastWheelPositions.size() != 0) {
            List<Double> wheelDeltas = new ArrayList<>();
            for (int i = 0; i < lastWheelPositions.size(); i++)
                wheelDeltas.add(wheelPositions.get(i) - lastWheelPositions.get(i));
            double angleDelta = normDelta(heading - lastHeading);
            double[][] input = new double[wheelDeltas.size()][1];
            for (int i = 0; i < wheelDeltas.size(); i++)
                input[i][0] = wheelDeltas.get(i) + angleDelta;
             RealMatrix rawPoseDelta = forwardSolver.solve(MatrixUtils.createRealMatrix(input).transpose());

             Pose2d robotPoseDelta = new Pose2d(
                     rawPoseDelta.getEntry(0, 0),
                     rawPoseDelta.getEntry(1, 0),
                     rawPoseDelta.getEntry(2,0)
             );

             poseEstimate = Kinematics.odometryUpdate(poseEstimate, robotPoseDelta);

        }
        lastWheelPositions = wheelPositions;
        lastHeading = heading;
    }

    double norm(double angle) {
        double modifiedAngle = angle % TAU;

        modifiedAngle = (modifiedAngle + TAU) % TAU;

        return modifiedAngle;
    }

    private double normDelta(double angleDelta) {
        double modifiedAngleDelta = norm(angleDelta);

        if (modifiedAngleDelta > Math.PI) {
            modifiedAngleDelta -= TAU;
        }

        return modifiedAngleDelta;
    }

    public abstract List<Double> getWheelPositions();
    public abstract double getHeading();
}




