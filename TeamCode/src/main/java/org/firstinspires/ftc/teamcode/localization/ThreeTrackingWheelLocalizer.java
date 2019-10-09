package org.firstinspires.ftc.teamcode.localization;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import java.util.ArrayList;
import java.util.List;

public abstract class ThreeTrackingWheelLocalizer {
	private Pose2d poseEstimate;
	private List<Double> lastWheelPositions;
	private DecompositionSolver forwardSolver;

	public ThreeTrackingWheelLocalizer(List<Pose2d> wheelPoses) {
		poseEstimate = new Pose2d();
		lastWheelPositions = new ArrayList<>();
		if (wheelPoses.size() != 3) {
			throw new IllegalArgumentException("3 wheel positions must be provided");
		}

		RealMatrix inverseMatrix = new Array2DRowRealMatrix(3,3);
		for (int i = 0; i < 2; i++) {
			Vector2d orientationVector = wheelPoses.get(i).headingVec();
			Vector2d positionVector = wheelPoses.get(i).vec();

			inverseMatrix.setEntry(i, 0, orientationVector.x);
			inverseMatrix.setEntry(i, 1, orientationVector.y);
			inverseMatrix.setEntry(i, 2,
					positionVector.x * orientationVector.y - positionVector.y * orientationVector.x);
		}

		forwardSolver = new LUDecomposition(inverseMatrix).getSolver();
		if (!forwardSolver.isNonSingular()) {
			throw new IllegalArgumentException("The specified configuration cannot support full localization");
		}
	}

	public void update() {
		List<Double> wheelPostions = getWheelPositions();
		if (lastWheelPositions.size() > 0) {
			RealMatrix wheelDeltas = new Array2DRowRealMatrix(3,1);
			for (int i = 0; i < 3; i++) {
				wheelDeltas.setEntry(i, 0,wheelPostions.get(i) - lastWheelPositions.get(i));
			}
			RealMatrix rawPoseDelta = forwardSolver.solve(wheelDeltas.transpose());
			Pose2d robotPoseDelta = new Pose2d(
					rawPoseDelta.getEntry(0,0),
					rawPoseDelta.getEntry(1,0),
					rawPoseDelta.getEntry(2,0)
			);

			poseEstimate = Kinematics.odometryUpdate(poseEstimate, robotPoseDelta);
		}

		lastWheelPositions = wheelPostions;
	}

	abstract List<Double> getWheelPositions();
}
