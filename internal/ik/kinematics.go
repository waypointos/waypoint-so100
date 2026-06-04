package ik

// JointLink is one revolute joint's fixed transform + local rotation axis.
// Fixed is applied first (URDF <origin>), then a rotation about Axis by q.
type JointLink struct {
	Fixed Mat4
	Axis  Vec3 // unit axis in the joint's local frame
}

type Kinematics struct {
	Links []JointLink // arm chain joints 1..5 (gripper excluded)
}

// fixedFromURDF builds a URDF <origin xyz rpy> as Translate * Rz(yaw) * Ry(pitch) * Rx(roll).
// URDF rpy is intrinsic XYZ, i.e. R = Rz * Ry * Rx.
func fixedFromURDF(x, y, z, roll, pitch, yaw float64) Mat4 {
	return Translate(x, y, z).Mul(RotZ(yaw)).Mul(RotY(pitch)).Mul(RotX(roll))
}

// SO100Kinematics returns the SO-100 arm chain transcribed from
// dashboard/public/models/so101/so101.urdf (joints 1..5, gripper excluded).
// Every joint rotates about its local z-axis per the URDF <axis xyz="0 0 1"/>.
func SO100Kinematics() Kinematics {
	return Kinematics{Links: []JointLink{
		// Joint 1 shoulder_pan (base->shoulder)
		{Fixed: fixedFromURDF(0.0388353, -8.97657e-09, 0.0624, 3.14159, 4.18253e-17, -3.14159), Axis: Vec3{0, 0, 1}},
		// Joint 2 shoulder_lift (shoulder->upper_arm)
		{Fixed: fixedFromURDF(-0.0303992, -0.0182778, -0.0542, -1.5708, -1.5708, 0), Axis: Vec3{0, 0, 1}},
		// Joint 3 elbow_flex (upper_arm->lower_arm)
		{Fixed: fixedFromURDF(-0.11257, -0.028, 1.73763e-16, -3.63608e-16, 8.74301e-16, 1.5708), Axis: Vec3{0, 0, 1}},
		// Joint 4 wrist_flex (lower_arm->wrist)
		{Fixed: fixedFromURDF(-0.1349, 0.0052, 3.62355e-17, 4.02456e-15, 8.67362e-16, -1.5708), Axis: Vec3{0, 0, 1}},
		// Joint 5 wrist_roll (wrist->gripper)
		{Fixed: fixedFromURDF(5.55112e-17, -0.0611, 0.0181, 1.5708, 0.0486795, 3.14159), Axis: Vec3{0, 0, 1}},
	}}
}

func axisRot(axis Vec3, q float64) Mat4 {
	switch {
	case axis.X != 0:
		return RotX(q * sign(axis.X))
	case axis.Y != 0:
		return RotY(q * sign(axis.Y))
	default:
		return RotZ(q * sign(axis.Z))
	}
}
func sign(v float64) float64 {
	if v < 0 {
		return -1
	}
	return 1
}

// frames returns the cumulative base->joint_i transform for each joint and the
// final EE transform.
func (k Kinematics) frames(q [5]float64) (origins []Vec3, axes []Vec3, ee Mat4) {
	t := Identity()
	for i, link := range k.Links {
		t = t.Mul(link.Fixed)
		origins = append(origins, t.Origin())
		axes = append(axes, axisWorld(t, link.Axis))
		t = t.Mul(axisRot(link.Axis, q[i]))
	}
	return origins, axes, t
}

func axisWorld(t Mat4, a Vec3) Vec3 {
	// rotate the local axis by t's rotation block
	return Vec3{
		t[0]*a.X + t[1]*a.Y + t[2]*a.Z,
		t[4]*a.X + t[5]*a.Y + t[6]*a.Z,
		t[8]*a.X + t[9]*a.Y + t[10]*a.Z,
	}
}

func (k Kinematics) FK(q [5]float64) Mat4 {
	_, _, ee := k.frames(q)
	return ee
}

// Jacobian returns the 6x5 geometric Jacobian (rows: vx,vy,vz,wx,wy,wz).
func (k Kinematics) Jacobian(q [5]float64) [6][5]float64 {
	origins, axes, ee := k.frames(q)
	pe := ee.Origin()
	var J [6][5]float64
	for i := 0; i < len(k.Links); i++ {
		lin := Cross(axes[i], Sub(pe, origins[i]))
		J[0][i], J[1][i], J[2][i] = lin.X, lin.Y, lin.Z
		J[3][i], J[4][i], J[5][i] = axes[i].X, axes[i].Y, axes[i].Z
	}
	return J
}
