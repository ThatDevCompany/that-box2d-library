/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

import {
	B2_linearSlop,
	B2_maxManifoldPoints,
	B2_velocityThreshold,
	B2_maxLinearCorrection,
	B2_baumgarte,
	B2_toiBaumgarte,
	B2MakeArray
} from '../../Common/b2Settings';
import {B2Min, B2Max, B2Clamp, B2Vec2, B2Mat22, B2Rot, B2Transform} from '../../Common/b2Math';
import {B2Manifold} from '../../Collision/b2Collision';
import {B2ManifoldPoint} from '../../Collision/b2Collision';
import {B2WorldManifold} from '../../Collision/b2Collision';
import {B2ManifoldType} from '../../Collision/b2Collision';
import {B2Shape} from '../../Collision/Shapes/b2Shape';
import {B2Contact} from './b2Contact';
import {B2Body} from '../b2Body';
import {B2Fixture} from '../b2Fixture';
import {B2TimeStep, B2Position, B2Velocity} from '../b2TimeStep';

export class B2VelocityConstraintPoint {
	public rA: B2Vec2 = new B2Vec2();
	public rB: B2Vec2 = new B2Vec2();
	public normalImpulse: number = 0;
	public tangentImpulse: number = 0;
	public normalMass: number = 0;
	public tangentMass: number = 0;
	public velocityBias: number = 0;

	public static MakeArray(length: number): B2VelocityConstraintPoint[] {
		return B2MakeArray(length, function (i) {
			return new B2VelocityConstraintPoint();
		});
	}
}

export class B2ContactVelocityConstraint {
	public points: B2VelocityConstraintPoint[] = B2VelocityConstraintPoint.MakeArray(B2_maxManifoldPoints);
	public normal: B2Vec2 = new B2Vec2();
	public tangent: B2Vec2 = new B2Vec2();
	public normalMass: B2Mat22 = new B2Mat22();
	public K: B2Mat22 = new B2Mat22();
	public indexA: number = 0;
	public indexB: number = 0;
	public invMassA: number = 0;
	public invMassB: number = 0;
	public invIA: number = 0;
	public invIB: number = 0;
	public friction: number = 0;
	public restitution: number = 0;
	public tangentSpeed: number = 0;
	public pointCount: number = 0;
	public contactIndex: number = 0;

	public static MakeArray(length: number): B2ContactVelocityConstraint[] {
		return B2MakeArray(length, function (i) {
			return new B2ContactVelocityConstraint();
		});
	}
}

export class B2ContactPositionConstraint {
	public localPoints: B2Vec2[] = B2Vec2.MakeArray(B2_maxManifoldPoints);
	public localNormal: B2Vec2 = new B2Vec2();
	public localPoint: B2Vec2 = new B2Vec2();
	public indexA: number = 0;
	public indexB: number = 0;
	public invMassA: number = 0;
	public invMassB: number = 0;
	public localCenterA: B2Vec2 = new B2Vec2();
	public localCenterB: B2Vec2 = new B2Vec2();
	public invIA: number = 0;
	public invIB: number = 0;
	public type: B2ManifoldType = B2ManifoldType.e_unknown;
	public radiusA: number = 0;
	public radiusB: number = 0;
	public pointCount: number = 0;

	public static MakeArray(length: number): B2ContactPositionConstraint[] {
		return B2MakeArray(length, function (i) {
			return new B2ContactPositionConstraint();
		});
	}
}

export class B2ContactSolverDef {
	public step: B2TimeStep = new B2TimeStep();
	public contacts: B2Contact[] = null;
	public count: number = 0;
	public positions: B2Position[] = null;
	public velocities: B2Velocity[] = null;
	public allocator: any = null;
}

export class B2PositionSolverManifold {
	public normal: B2Vec2 = new B2Vec2();
	public point: B2Vec2 = new B2Vec2();
	public separation: number = 0;

	private static Initialize_s_pointA = new B2Vec2();
	private static Initialize_s_pointB = new B2Vec2();
	private static Initialize_s_planePoint = new B2Vec2();
	private static Initialize_s_clipPoint = new B2Vec2();

	public Initialize(pc: B2ContactPositionConstraint, xfA: B2Transform, xfB: B2Transform, index: number): void {
		const pointA: B2Vec2 = B2PositionSolverManifold.Initialize_s_pointA;
		const pointB: B2Vec2 = B2PositionSolverManifold.Initialize_s_pointB;
		const planePoint: B2Vec2 = B2PositionSolverManifold.Initialize_s_planePoint;
		const clipPoint: B2Vec2 = B2PositionSolverManifold.Initialize_s_clipPoint;

		/// b2Assert(pc.pointCount > 0);

		switch (pc.type) {
			case B2ManifoldType.e_circles: {
				// B2Vec2 pointA = B2Mul(xfA, pc->localPoint);
				B2Transform.MulXV(xfA, pc.localPoint, pointA);
				// B2Vec2 pointB = B2Mul(xfB, pc->localPoints[0]);
				B2Transform.MulXV(xfB, pc.localPoints[0], pointB);
				// normal = pointB - pointA;
				// normal.Normalize();
				B2Vec2.SubVV(pointB, pointA, this.normal).SelfNormalize();
				// point = 0.5f * (pointA + pointB);
				B2Vec2.MidVV(pointA, pointB, this.point);
				// separation = B2Dot(pointB - pointA, normal) - pc->radius;
				this.separation = B2Vec2.DotVV(B2Vec2.SubVV(pointB, pointA, B2Vec2.s_t0), this.normal) - pc.radiusA - pc.radiusB;
			}
				break;

			case B2ManifoldType.e_faceA: {
				// normal = B2Mul(xfA.q, pc->localNormal);
				B2Rot.MulRV(xfA.q, pc.localNormal, this.normal);
				// B2Vec2 planePoint = B2Mul(xfA, pc->localPoint);
				B2Transform.MulXV(xfA, pc.localPoint, planePoint);

				// B2Vec2 clipPoint = B2Mul(xfB, pc->localPoints[index]);
				B2Transform.MulXV(xfB, pc.localPoints[index], clipPoint);
				// separation = B2Dot(clipPoint - planePoint, normal) - pc->radius;
				this.separation = B2Vec2.DotVV(B2Vec2.SubVV(clipPoint, planePoint, B2Vec2.s_t0), this.normal) - pc.radiusA - pc.radiusB;
				// point = clipPoint;
				this.point.Copy(clipPoint);
			}
				break;

			case B2ManifoldType.e_faceB: {
				// normal = B2Mul(xfB.q, pc->localNormal);
				B2Rot.MulRV(xfB.q, pc.localNormal, this.normal);
				// B2Vec2 planePoint = B2Mul(xfB, pc->localPoint);
				B2Transform.MulXV(xfB, pc.localPoint, planePoint);

				// B2Vec2 clipPoint = B2Mul(xfA, pc->localPoints[index]);
				B2Transform.MulXV(xfA, pc.localPoints[index], clipPoint);
				// separation = B2Dot(clipPoint - planePoint, normal) - pc->radius;
				this.separation = B2Vec2.DotVV(B2Vec2.SubVV(clipPoint, planePoint, B2Vec2.s_t0), this.normal) - pc.radiusA - pc.radiusB;
				// point = clipPoint;
				this.point.Copy(clipPoint);

				// Ensure normal points from A to B
				// normal = -normal;
				this.normal.SelfNeg();
			}
				break;
		}
	}
}

export class B2ContactSolver {
	public m_step: B2TimeStep = new B2TimeStep();
	public m_positions: B2Position[] = null;
	public m_velocities: B2Velocity[] = null;
	public m_allocator: any = null;
	public m_positionConstraints: B2ContactPositionConstraint[] = B2ContactPositionConstraint.MakeArray(1024); // TODO: B2Settings
	public m_velocityConstraints: B2ContactVelocityConstraint[] = B2ContactVelocityConstraint.MakeArray(1024); // TODO: B2Settings
	public m_contacts: B2Contact[] = null;
	public m_count: number = 0;

	public Initialize(def: B2ContactSolverDef): B2ContactSolver {
		this.m_step.Copy(def.step);
		this.m_allocator = def.allocator;
		this.m_count = def.count;
		// TODO:
		if (this.m_positionConstraints.length < this.m_count) {
			const new_length: number = B2Max(this.m_positionConstraints.length * 2, this.m_count);
			while (this.m_positionConstraints.length < new_length) {
				this.m_positionConstraints[this.m_positionConstraints.length] = new B2ContactPositionConstraint();
			}
		}
		// TODO:
		if (this.m_velocityConstraints.length < this.m_count) {
			const new_length: number = B2Max(this.m_velocityConstraints.length * 2, this.m_count);
			while (this.m_velocityConstraints.length < new_length) {
				this.m_velocityConstraints[this.m_velocityConstraints.length] = new B2ContactVelocityConstraint();
			}
		}
		this.m_positions = def.positions;
		this.m_velocities = def.velocities;
		this.m_contacts = def.contacts;

		// Initialize position independent portions of the constraints.
		for (let i: number = 0; i < this.m_count; ++i) {
			const contact: B2Contact = this.m_contacts[i];

			const fixtureA: B2Fixture = contact.m_fixtureA;
			const fixtureB: B2Fixture = contact.m_fixtureB;
			const shapeA: B2Shape = fixtureA.GetShape();
			const shapeB: B2Shape = fixtureB.GetShape();
			const radiusA: number = shapeA.m_radius;
			const radiusB: number = shapeB.m_radius;
			const bodyA: B2Body = fixtureA.GetBody();
			const bodyB: B2Body = fixtureB.GetBody();
			const manifold: B2Manifold = contact.GetManifold();

			const pointCount: number = manifold.pointCount;
			/// b2Assert(pointCount > 0);

			const vc: B2ContactVelocityConstraint = this.m_velocityConstraints[i];
			vc.friction = contact.m_friction;
			vc.restitution = contact.m_restitution;
			vc.tangentSpeed = contact.m_tangentSpeed;
			vc.indexA = bodyA.m_islandIndex;
			vc.indexB = bodyB.m_islandIndex;
			vc.invMassA = bodyA.m_invMass;
			vc.invMassB = bodyB.m_invMass;
			vc.invIA = bodyA.m_invI;
			vc.invIB = bodyB.m_invI;
			vc.contactIndex = i;
			vc.pointCount = pointCount;
			vc.K.SetZero();
			vc.normalMass.SetZero();

			const pc: B2ContactPositionConstraint = this.m_positionConstraints[i];
			pc.indexA = bodyA.m_islandIndex;
			pc.indexB = bodyB.m_islandIndex;
			pc.invMassA = bodyA.m_invMass;
			pc.invMassB = bodyB.m_invMass;
			pc.localCenterA.Copy(bodyA.m_sweep.localCenter);
			pc.localCenterB.Copy(bodyB.m_sweep.localCenter);
			pc.invIA = bodyA.m_invI;
			pc.invIB = bodyB.m_invI;
			pc.localNormal.Copy(manifold.localNormal);
			pc.localPoint.Copy(manifold.localPoint);
			pc.pointCount = pointCount;
			pc.radiusA = radiusA;
			pc.radiusB = radiusB;
			pc.type = manifold.type;

			for (let j: number = 0; j < pointCount; ++j) {
				const cp: B2ManifoldPoint = manifold.points[j];
				const vcp: B2VelocityConstraintPoint = vc.points[j];

				if (this.m_step.warmStarting) {
					vcp.normalImpulse = this.m_step.dtRatio * cp.normalImpulse;
					vcp.tangentImpulse = this.m_step.dtRatio * cp.tangentImpulse;
				} else {
					vcp.normalImpulse = 0;
					vcp.tangentImpulse = 0;
				}

				vcp.rA.SetZero();
				vcp.rB.SetZero();
				vcp.normalMass = 0;
				vcp.tangentMass = 0;
				vcp.velocityBias = 0;

				pc.localPoints[j].Copy(cp.localPoint);
			}
		}

		return this;
	}

	private static InitializeVelocityConstraints_s_xfA = new B2Transform();
	private static InitializeVelocityConstraints_s_xfB = new B2Transform();
	private static InitializeVelocityConstraints_s_worldManifold = new B2WorldManifold();

	public InitializeVelocityConstraints(): void {
		const xfA: B2Transform = B2ContactSolver.InitializeVelocityConstraints_s_xfA;
		const xfB: B2Transform = B2ContactSolver.InitializeVelocityConstraints_s_xfB;
		const worldManifold: B2WorldManifold = B2ContactSolver.InitializeVelocityConstraints_s_worldManifold;

		const k_maxConditionNumber: number = 1000;

		for (let i: number = 0; i < this.m_count; ++i) {
			const vc: B2ContactVelocityConstraint = this.m_velocityConstraints[i];
			const pc: B2ContactPositionConstraint = this.m_positionConstraints[i];

			const radiusA: number = pc.radiusA;
			const radiusB: number = pc.radiusB;
			const manifold: B2Manifold = this.m_contacts[vc.contactIndex].GetManifold();

			const indexA: number = vc.indexA;
			const indexB: number = vc.indexB;

			const mA: number = vc.invMassA;
			const mB: number = vc.invMassB;
			const iA: number = vc.invIA;
			const iB: number = vc.invIB;
			const localCenterA: B2Vec2 = pc.localCenterA;
			const localCenterB: B2Vec2 = pc.localCenterB;

			const cA: B2Vec2 = this.m_positions[indexA].c;
			const aA: number = this.m_positions[indexA].a;
			const vA: B2Vec2 = this.m_velocities[indexA].v;
			const wA: number = this.m_velocities[indexA].w;

			const cB: B2Vec2 = this.m_positions[indexB].c;
			const aB: number = this.m_positions[indexB].a;
			const vB: B2Vec2 = this.m_velocities[indexB].v;
			const wB: number = this.m_velocities[indexB].w;

			/// b2Assert(manifold.pointCount > 0);

			xfA.q.SetAngle(aA);
			xfB.q.SetAngle(aB);
			B2Vec2.SubVV(cA, B2Rot.MulRV(xfA.q, localCenterA, B2Vec2.s_t0), xfA.p);
			B2Vec2.SubVV(cB, B2Rot.MulRV(xfB.q, localCenterB, B2Vec2.s_t0), xfB.p);

			worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB);

			vc.normal.Copy(worldManifold.normal);
			B2Vec2.CrossVOne(vc.normal, vc.tangent); // compute from normal

			const pointCount: number = vc.pointCount;
			for (let j: number = 0; j < pointCount; ++j) {
				const vcp: B2VelocityConstraintPoint = vc.points[j];

				// vcp->rA = worldManifold.points[j] - cA;
				B2Vec2.SubVV(worldManifold.points[j], cA, vcp.rA);
				// vcp->rB = worldManifold.points[j] - cB;
				B2Vec2.SubVV(worldManifold.points[j], cB, vcp.rB);

				const rnA: number = B2Vec2.CrossVV(vcp.rA, vc.normal);
				const rnB: number = B2Vec2.CrossVV(vcp.rB, vc.normal);

				const kNormal: number = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

				vcp.normalMass = kNormal > 0 ? 1 / kNormal : 0;

				// B2Vec2 tangent = B2Cross(vc->normal, 1.0f);
				const tangent: B2Vec2 = vc.tangent; // precomputed from normal

				const rtA: number = B2Vec2.CrossVV(vcp.rA, tangent);
				const rtB: number = B2Vec2.CrossVV(vcp.rB, tangent);

				const kTangent: number = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

				vcp.tangentMass = kTangent > 0 ? 1 / kTangent : 0;

				// Setup a velocity bias for restitution.
				vcp.velocityBias = 0;
				// float32 vRel = B2Dot(vc->normal, vB + B2Cross(wB, vcp->rB) - vA - B2Cross(wA, vcp->rA));
				const vRel: number = B2Vec2.DotVV(
					vc.normal,
					B2Vec2.SubVV(
						B2Vec2.AddVCrossSV(vB, wB, vcp.rB, B2Vec2.s_t0),
						B2Vec2.AddVCrossSV(vA, wA, vcp.rA, B2Vec2.s_t1),
						B2Vec2.s_t0));
				if (vRel < (-B2_velocityThreshold)) {
					vcp.velocityBias += (-vc.restitution * vRel);
				}
			}

			// If we have two points, then prepare the block solver.
			if (vc.pointCount === 2) {
				const vcp1: B2VelocityConstraintPoint = vc.points[0];
				const vcp2: B2VelocityConstraintPoint = vc.points[1];

				const rn1A: number = B2Vec2.CrossVV(vcp1.rA, vc.normal);
				const rn1B: number = B2Vec2.CrossVV(vcp1.rB, vc.normal);
				const rn2A: number = B2Vec2.CrossVV(vcp2.rA, vc.normal);
				const rn2B: number = B2Vec2.CrossVV(vcp2.rB, vc.normal);

				const k11: number = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
				const k22: number = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
				const k12: number = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

				// Ensure a reasonable condition number.
				// float32 k_maxConditionNumber = 1000.0f;
				if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
					// K is safe to invert.
					vc.K.ex.Set(k11, k12);
					vc.K.ey.Set(k12, k22);
					vc.K.GetInverse(vc.normalMass);
				} else {
					// The constraints are redundant, just use one.
					// TODO_ERIN use deepest?
					vc.pointCount = 1;
				}
			}
		}
	}

	private static WarmStart_s_P = new B2Vec2();

	public WarmStart(): void {
		const P: B2Vec2 = B2ContactSolver.WarmStart_s_P;

		// Warm start.
		for (let i: number = 0; i < this.m_count; ++i) {
			const vc: B2ContactVelocityConstraint = this.m_velocityConstraints[i];

			const indexA: number = vc.indexA;
			const indexB: number = vc.indexB;
			const mA: number = vc.invMassA;
			const iA: number = vc.invIA;
			const mB: number = vc.invMassB;
			const iB: number = vc.invIB;
			const pointCount: number = vc.pointCount;

			const vA: B2Vec2 = this.m_velocities[indexA].v;
			let wA: number = this.m_velocities[indexA].w;
			const vB: B2Vec2 = this.m_velocities[indexB].v;
			let wB: number = this.m_velocities[indexB].w;

			const normal: B2Vec2 = vc.normal;
			// B2Vec2 tangent = B2Cross(normal, 1.0f);
			const tangent: B2Vec2 = vc.tangent; // precomputed from normal

			for (let j: number = 0; j < pointCount; ++j) {
				const vcp: B2VelocityConstraintPoint = vc.points[j];
				// B2Vec2 P = vcp->normalImpulse * normal + vcp->tangentImpulse * tangent;
				B2Vec2.AddVV(
					B2Vec2.MulSV(vcp.normalImpulse, normal, B2Vec2.s_t0),
					B2Vec2.MulSV(vcp.tangentImpulse, tangent, B2Vec2.s_t1),
					P);
				// wA -= iA * B2Cross(vcp->rA, P);
				wA -= iA * B2Vec2.CrossVV(vcp.rA, P);
				// vA -= mA * P;
				vA.SelfMulSub(mA, P);
				// wB += iB * B2Cross(vcp->rB, P);
				wB += iB * B2Vec2.CrossVV(vcp.rB, P);
				// vB += mB * P;
				vB.SelfMulAdd(mB, P);
			}

			// this.m_velocities[indexA].v = vA;
			this.m_velocities[indexA].w = wA;
			// this.m_velocities[indexB].v = vB;
			this.m_velocities[indexB].w = wB;
		}
	}

	private static SolveVelocityConstraints_s_dv = new B2Vec2();
	private static SolveVelocityConstraints_s_dv1 = new B2Vec2();
	private static SolveVelocityConstraints_s_dv2 = new B2Vec2();
	private static SolveVelocityConstraints_s_P = new B2Vec2();
	private static SolveVelocityConstraints_s_a = new B2Vec2();
	private static SolveVelocityConstraints_s_b = new B2Vec2();
	private static SolveVelocityConstraints_s_x = new B2Vec2();
	private static SolveVelocityConstraints_s_d = new B2Vec2();
	private static SolveVelocityConstraints_s_P1 = new B2Vec2();
	private static SolveVelocityConstraints_s_P2 = new B2Vec2();
	private static SolveVelocityConstraints_s_P1P2 = new B2Vec2();

	public SolveVelocityConstraints(): void {
		const dv: B2Vec2 = B2ContactSolver.SolveVelocityConstraints_s_dv;
		const dv1: B2Vec2 = B2ContactSolver.SolveVelocityConstraints_s_dv1;
		const dv2: B2Vec2 = B2ContactSolver.SolveVelocityConstraints_s_dv2;
		const P: B2Vec2 = B2ContactSolver.SolveVelocityConstraints_s_P;
		const a: B2Vec2 = B2ContactSolver.SolveVelocityConstraints_s_a;
		const b: B2Vec2 = B2ContactSolver.SolveVelocityConstraints_s_b;
		const x: B2Vec2 = B2ContactSolver.SolveVelocityConstraints_s_x;
		const d: B2Vec2 = B2ContactSolver.SolveVelocityConstraints_s_d;
		const P1: B2Vec2 = B2ContactSolver.SolveVelocityConstraints_s_P1;
		const P2: B2Vec2 = B2ContactSolver.SolveVelocityConstraints_s_P2;
		const P1P2: B2Vec2 = B2ContactSolver.SolveVelocityConstraints_s_P1P2;

		for (let i: number = 0; i < this.m_count; ++i) {
			const vc: B2ContactVelocityConstraint = this.m_velocityConstraints[i];

			const indexA: number = vc.indexA;
			const indexB: number = vc.indexB;
			const mA: number = vc.invMassA;
			const iA: number = vc.invIA;
			const mB: number = vc.invMassB;
			const iB: number = vc.invIB;
			const pointCount: number = vc.pointCount;

			const vA: B2Vec2 = this.m_velocities[indexA].v;
			let wA: number = this.m_velocities[indexA].w;
			const vB: B2Vec2 = this.m_velocities[indexB].v;
			let wB: number = this.m_velocities[indexB].w;

			// B2Vec2 normal = vc->normal;
			const normal: B2Vec2 = vc.normal;
			// B2Vec2 tangent = B2Cross(normal, 1.0f);
			const tangent: B2Vec2 = vc.tangent; // precomputed from normal
			const friction: number = vc.friction;

			/// b2Assert(pointCount === 1 || pointCount === 2);

			// Solve tangent constraints first because non-penetration is more important
			// than friction.
			for (let j: number = 0; j < pointCount; ++j) {
				const vcp: B2VelocityConstraintPoint = vc.points[j];

				// Relative velocity at contact
				// B2Vec2 dv = vB + B2Cross(wB, vcp->rB) - vA - B2Cross(wA, vcp->rA);
				B2Vec2.SubVV(
					B2Vec2.AddVCrossSV(vB, wB, vcp.rB, B2Vec2.s_t0),
					B2Vec2.AddVCrossSV(vA, wA, vcp.rA, B2Vec2.s_t1),
					dv);

				// Compute tangent force
				// float32 vt = B2Dot(dv, tangent) - vc->tangentSpeed;
				const vt: number = B2Vec2.DotVV(dv, tangent) - vc.tangentSpeed;
				let lambda: number = vcp.tangentMass * (-vt);

				// B2Clamp the accumulated force
				const maxFriction: number = friction * vcp.normalImpulse;
				const newImpulse: number = B2Clamp(vcp.tangentImpulse + lambda, (-maxFriction), maxFriction);
				lambda = newImpulse - vcp.tangentImpulse;
				vcp.tangentImpulse = newImpulse;

				// Apply contact impulse
				// B2Vec2 P = lambda * tangent;
				B2Vec2.MulSV(lambda, tangent, P);

				// vA -= mA * P;
				vA.SelfMulSub(mA, P);
				// wA -= iA * B2Cross(vcp->rA, P);
				wA -= iA * B2Vec2.CrossVV(vcp.rA, P);

				// vB += mB * P;
				vB.SelfMulAdd(mB, P);
				// wB += iB * B2Cross(vcp->rB, P);
				wB += iB * B2Vec2.CrossVV(vcp.rB, P);
			}

			// Solve normal constraints
			if (vc.pointCount === 1) {
				const vcp: B2VelocityConstraintPoint = vc.points[0];

				// Relative velocity at contact
				// B2Vec2 dv = vB + B2Cross(wB, vcp->rB) - vA - B2Cross(wA, vcp->rA);
				B2Vec2.SubVV(
					B2Vec2.AddVCrossSV(vB, wB, vcp.rB, B2Vec2.s_t0),
					B2Vec2.AddVCrossSV(vA, wA, vcp.rA, B2Vec2.s_t1),
					dv);

				// Compute normal impulse
				// float32 vn = B2Dot(dv, normal);
				const vn: number = B2Vec2.DotVV(dv, normal);
				let lambda: number = (-vcp.normalMass * (vn - vcp.velocityBias));

				// B2Clamp the accumulated impulse
				// float32 newImpulse = B2Max(vcp->normalImpulse + lambda, 0.0f);
				const newImpulse: number = B2Max(vcp.normalImpulse + lambda, 0);
				lambda = newImpulse - vcp.normalImpulse;
				vcp.normalImpulse = newImpulse;

				// Apply contact impulse
				// B2Vec2 P = lambda * normal;
				B2Vec2.MulSV(lambda, normal, P);
				// vA -= mA * P;
				vA.SelfMulSub(mA, P);
				// wA -= iA * B2Cross(vcp->rA, P);
				wA -= iA * B2Vec2.CrossVV(vcp.rA, P);

				// vB += mB * P;
				vB.SelfMulAdd(mB, P);
				// wB += iB * B2Cross(vcp->rB, P);
				wB += iB * B2Vec2.CrossVV(vcp.rB, P);
			} else {
				// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
				// Build the mini LCP for this contact patch
				//
				// vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
				//
				// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
				// b = vn0 - velocityBias
				//
				// The system is solved using the 'Total enumeration method' (s. Murty). The complementary constraint vn_i * x_i
				// implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
				// vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
				// solution that satisfies the problem is chosen.
				//
				// In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
				// that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
				//
				// Substitute:
				//
				// x = a + d
				//
				// a := old total impulse
				// x := new total impulse
				// d := incremental impulse
				//
				// For the current iteration we extend the formula for the incremental impulse
				// to compute the new total impulse:
				//
				// vn = A * d + b
				//    = A * (x - a) + b
				//    = A * x + b - A * a
				//    = A * x + b'
				// b' = b - A * a;

				const cp1: B2VelocityConstraintPoint = vc.points[0];
				const cp2: B2VelocityConstraintPoint = vc.points[1];

				// B2Vec2 a(cp1->normalImpulse, cp2->normalImpulse);
				a.Set(cp1.normalImpulse, cp2.normalImpulse);
				/// b2Assert(a.x >= 0 && a.y >= 0);

				// Relative velocity at contact
				// B2Vec2 dv1 = vB + B2Cross(wB, cp1->rB) - vA - B2Cross(wA, cp1->rA);
				B2Vec2.SubVV(
					B2Vec2.AddVCrossSV(vB, wB, cp1.rB, B2Vec2.s_t0),
					B2Vec2.AddVCrossSV(vA, wA, cp1.rA, B2Vec2.s_t1),
					dv1);
				// B2Vec2 dv2 = vB + B2Cross(wB, cp2->rB) - vA - B2Cross(wA, cp2->rA);
				B2Vec2.SubVV(
					B2Vec2.AddVCrossSV(vB, wB, cp2.rB, B2Vec2.s_t0),
					B2Vec2.AddVCrossSV(vA, wA, cp2.rA, B2Vec2.s_t1),
					dv2);

				// Compute normal velocity
				// float32 vn1 = B2Dot(dv1, normal);
				let vn1: number = B2Vec2.DotVV(dv1, normal);
				// float32 vn2 = B2Dot(dv2, normal);
				let vn2: number = B2Vec2.DotVV(dv2, normal);

				// B2Vec2 b;
				b.x = vn1 - cp1.velocityBias;
				b.y = vn2 - cp2.velocityBias;

				// Compute b'
				// b -= B2Mul(vc->K, a);
				b.SelfSub(B2Mat22.MulMV(vc.K, a, B2Vec2.s_t0));

				/*
				#if B2_DEBUG_SOLVER === 1
				const k_errorTol: number = 0.001;
				#endif
				*/

				for (; ;) {
					//
					// Case 1: vn = 0
					//
					// 0 = A * x + b'
					//
					// Solve for x:
					//
					// x = - inv(A) * b'
					//
					// B2Vec2 x = - B2Mul(vc->normalMass, b);
					B2Mat22.MulMV(vc.normalMass, b, x).SelfNeg();

					if (x.x >= 0 && x.y >= 0) {
						// Get the incremental impulse
						// B2Vec2 d = x - a;
						B2Vec2.SubVV(x, a, d);

						// Apply incremental impulse
						// B2Vec2 P1 = d.x * normal;
						B2Vec2.MulSV(d.x, normal, P1);
						// B2Vec2 P2 = d.y * normal;
						B2Vec2.MulSV(d.y, normal, P2);
						B2Vec2.AddVV(P1, P2, P1P2);
						// vA -= mA * (P1 + P2);
						vA.SelfMulSub(mA, P1P2);
						// wA -= iA * (B2Cross(cp1->rA, P1) + B2Cross(cp2->rA, P2));
						wA -= iA * (B2Vec2.CrossVV(cp1.rA, P1) + B2Vec2.CrossVV(cp2.rA, P2));

						// vB += mB * (P1 + P2);
						vB.SelfMulAdd(mB, P1P2);
						// wB += iB * (B2Cross(cp1->rB, P1) + B2Cross(cp2->rB, P2));
						wB += iB * (B2Vec2.CrossVV(cp1.rB, P1) + B2Vec2.CrossVV(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

						/*
						#if B2_DEBUG_SOLVER === 1
						// Postconditions
						dv1 = vB + B2Cross(wB, cp1->rB) - vA - B2Cross(wA, cp1->rA);
						dv2 = vB + B2Cross(wB, cp2->rB) - vA - B2Cross(wA, cp2->rA);

						// Compute normal velocity
						vn1 = B2Dot(dv1, normal);
						vn2 = B2Dot(dv2, normal);

						/// b2Assert(B2Abs(vn1 - cp1->velocityBias) < k_errorTol);
						/// b2Assert(B2Abs(vn2 - cp2->velocityBias) < k_errorTol);
						#endif
						*/
						break;
					}

					//
					// Case 2: vn1 = 0 and x2 = 0
					//
					//   0 = a11 * x1 + a12 * 0 + b1'
					// vn2 = a21 * x1 + a22 * 0 + B2'
					//
					x.x = (-cp1.normalMass * b.x);
					x.y = 0;
					vn1 = 0;
					vn2 = vc.K.ex.y * x.x + b.y;

					if (x.x >= 0 && vn2 >= 0) {
						// Get the incremental impulse
						// B2Vec2 d = x - a;
						B2Vec2.SubVV(x, a, d);

						// Apply incremental impulse
						// B2Vec2 P1 = d.x * normal;
						B2Vec2.MulSV(d.x, normal, P1);
						// B2Vec2 P2 = d.y * normal;
						B2Vec2.MulSV(d.y, normal, P2);
						B2Vec2.AddVV(P1, P2, P1P2);
						// vA -= mA * (P1 + P2);
						vA.SelfMulSub(mA, P1P2);
						// wA -= iA * (B2Cross(cp1->rA, P1) + B2Cross(cp2->rA, P2));
						wA -= iA * (B2Vec2.CrossVV(cp1.rA, P1) + B2Vec2.CrossVV(cp2.rA, P2));

						// vB += mB * (P1 + P2);
						vB.SelfMulAdd(mB, P1P2);
						// wB += iB * (B2Cross(cp1->rB, P1) + B2Cross(cp2->rB, P2));
						wB += iB * (B2Vec2.CrossVV(cp1.rB, P1) + B2Vec2.CrossVV(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

						/*
						#if B2_DEBUG_SOLVER === 1
						// Postconditions
						dv1 = vB + B2Cross(wB, cp1->rB) - vA - B2Cross(wA, cp1->rA);

						// Compute normal velocity
						vn1 = B2Dot(dv1, normal);

						/// b2Assert(B2Abs(vn1 - cp1->velocityBias) < k_errorTol);
						#endif
						*/
						break;
					}


					//
					// Case 3: vn2 = 0 and x1 = 0
					//
					// vn1 = a11 * 0 + a12 * x2 + b1'
					//   0 = a21 * 0 + a22 * x2 + B2'
					//
					x.x = 0;
					x.y = (-cp2.normalMass * b.y);
					vn1 = vc.K.ey.x * x.y + b.x;
					vn2 = 0;

					if (x.y >= 0 && vn1 >= 0) {
						// Resubstitute for the incremental impulse
						// B2Vec2 d = x - a;
						B2Vec2.SubVV(x, a, d);

						// Apply incremental impulse
						// B2Vec2 P1 = d.x * normal;
						B2Vec2.MulSV(d.x, normal, P1);
						// B2Vec2 P2 = d.y * normal;
						B2Vec2.MulSV(d.y, normal, P2);
						B2Vec2.AddVV(P1, P2, P1P2);
						// vA -= mA * (P1 + P2);
						vA.SelfMulSub(mA, P1P2);
						// wA -= iA * (B2Cross(cp1->rA, P1) + B2Cross(cp2->rA, P2));
						wA -= iA * (B2Vec2.CrossVV(cp1.rA, P1) + B2Vec2.CrossVV(cp2.rA, P2));

						// vB += mB * (P1 + P2);
						vB.SelfMulAdd(mB, P1P2);
						// wB += iB * (B2Cross(cp1->rB, P1) + B2Cross(cp2->rB, P2));
						wB += iB * (B2Vec2.CrossVV(cp1.rB, P1) + B2Vec2.CrossVV(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

						/*
						#if B2_DEBUG_SOLVER === 1
						// Postconditions
						dv2 = vB + B2Cross(wB, cp2->rB) - vA - B2Cross(wA, cp2->rA);

						// Compute normal velocity
						vn2 = B2Dot(dv2, normal);

						/// b2Assert(B2Abs(vn2 - cp2->velocityBias) < k_errorTol);
						#endif
						*/
						break;
					}

					//
					// Case 4: x1 = 0 and x2 = 0
					//
					// vn1 = b1
					// vn2 = B2;
					x.x = 0;
					x.y = 0;
					vn1 = b.x;
					vn2 = b.y;

					if (vn1 >= 0 && vn2 >= 0) {
						// Resubstitute for the incremental impulse
						// B2Vec2 d = x - a;
						B2Vec2.SubVV(x, a, d);

						// Apply incremental impulse
						// B2Vec2 P1 = d.x * normal;
						B2Vec2.MulSV(d.x, normal, P1);
						// B2Vec2 P2 = d.y * normal;
						B2Vec2.MulSV(d.y, normal, P2);
						B2Vec2.AddVV(P1, P2, P1P2);
						// vA -= mA * (P1 + P2);
						vA.SelfMulSub(mA, P1P2);
						// wA -= iA * (B2Cross(cp1->rA, P1) + B2Cross(cp2->rA, P2));
						wA -= iA * (B2Vec2.CrossVV(cp1.rA, P1) + B2Vec2.CrossVV(cp2.rA, P2));

						// vB += mB * (P1 + P2);
						vB.SelfMulAdd(mB, P1P2);
						// wB += iB * (B2Cross(cp1->rB, P1) + B2Cross(cp2->rB, P2));
						wB += iB * (B2Vec2.CrossVV(cp1.rB, P1) + B2Vec2.CrossVV(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

						break;
					}

					// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
					break;
				}
			}

			// this.m_velocities[indexA].v = vA;
			this.m_velocities[indexA].w = wA;
			// this.m_velocities[indexB].v = vB;
			this.m_velocities[indexB].w = wB;
		}
	}

	public StoreImpulses(): void {
		for (let i: number = 0; i < this.m_count; ++i) {
			let vc: B2ContactVelocityConstraint = this.m_velocityConstraints[i];
			let manifold: B2Manifold = this.m_contacts[vc.contactIndex].GetManifold();

			for (let j: number = 0; j < vc.pointCount; ++j) {
				manifold.points[j].normalImpulse = vc.points[j].normalImpulse;
				manifold.points[j].tangentImpulse = vc.points[j].tangentImpulse;
			}
		}
	}

	private static SolvePositionConstraints_s_xfA = new B2Transform();
	private static SolvePositionConstraints_s_xfB = new B2Transform();
	private static SolvePositionConstraints_s_psm = new B2PositionSolverManifold();
	private static SolvePositionConstraints_s_rA = new B2Vec2();
	private static SolvePositionConstraints_s_rB = new B2Vec2();
	private static SolvePositionConstraints_s_P = new B2Vec2();

	public SolvePositionConstraints(): boolean {
		const xfA: B2Transform = B2ContactSolver.SolvePositionConstraints_s_xfA;
		const xfB: B2Transform = B2ContactSolver.SolvePositionConstraints_s_xfB;
		const psm: B2PositionSolverManifold = B2ContactSolver.SolvePositionConstraints_s_psm;
		const rA: B2Vec2 = B2ContactSolver.SolvePositionConstraints_s_rA;
		const rB: B2Vec2 = B2ContactSolver.SolvePositionConstraints_s_rB;
		const P: B2Vec2 = B2ContactSolver.SolvePositionConstraints_s_P;

		let minSeparation: number = 0;

		for (let i: number = 0; i < this.m_count; ++i) {
			const pc: B2ContactPositionConstraint = this.m_positionConstraints[i];

			const indexA: number = pc.indexA;
			const indexB: number = pc.indexB;
			const localCenterA: B2Vec2 = pc.localCenterA;
			const mA: number = pc.invMassA;
			const iA: number = pc.invIA;
			const localCenterB: B2Vec2 = pc.localCenterB;
			const mB: number = pc.invMassB;
			const iB: number = pc.invIB;
			const pointCount: number = pc.pointCount;

			const cA: B2Vec2 = this.m_positions[indexA].c;
			let aA: number = this.m_positions[indexA].a;

			const cB: B2Vec2 = this.m_positions[indexB].c;
			let aB: number = this.m_positions[indexB].a;

			// Solve normal constraints
			for (let j: number = 0; j < pointCount; ++j) {
				xfA.q.SetAngle(aA);
				xfB.q.SetAngle(aB);
				B2Vec2.SubVV(cA, B2Rot.MulRV(xfA.q, localCenterA, B2Vec2.s_t0), xfA.p);
				B2Vec2.SubVV(cB, B2Rot.MulRV(xfB.q, localCenterB, B2Vec2.s_t0), xfB.p);

				psm.Initialize(pc, xfA, xfB, j);
				const normal: B2Vec2 = psm.normal;

				const point: B2Vec2 = psm.point;
				const separation: number = psm.separation;

				// B2Vec2 rA = point - cA;
				B2Vec2.SubVV(point, cA, rA);
				// B2Vec2 rB = point - cB;
				B2Vec2.SubVV(point, cB, rB);

				// Track max constraint error.
				minSeparation = B2Min(minSeparation, separation);

				// Prevent large corrections and allow slop.
				const C: number = B2Clamp(B2_baumgarte * (separation + B2_linearSlop), (-B2_maxLinearCorrection), 0);

				// Compute the effective mass.
				// float32 rnA = B2Cross(rA, normal);
				const rnA: number = B2Vec2.CrossVV(rA, normal);
				// float32 rnB = B2Cross(rB, normal);
				const rnB: number = B2Vec2.CrossVV(rB, normal);
				// float32 K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
				const K: number = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

				// Compute normal impulse
				const impulse: number = K > 0 ? -C / K : 0;

				// B2Vec2 P = impulse * normal;
				B2Vec2.MulSV(impulse, normal, P);

				// cA -= mA * P;
				cA.SelfMulSub(mA, P);
				// aA -= iA * B2Cross(rA, P);
				aA -= iA * B2Vec2.CrossVV(rA, P);

				// cB += mB * P;
				cB.SelfMulAdd(mB, P);
				// aB += iB * B2Cross(rB, P);
				aB += iB * B2Vec2.CrossVV(rB, P);
			}

			// this.m_positions[indexA].c = cA;
			this.m_positions[indexA].a = aA;

			// this.m_positions[indexB].c = cB;
			this.m_positions[indexB].a = aB;
		}

		// We can't expect minSpeparation >= -B2_linearSlop because we don't
		// push the separation above -B2_linearSlop.
		return minSeparation > (-3 * B2_linearSlop);
	}

	private static SolveTOIPositionConstraints_s_xfA = new B2Transform();
	private static SolveTOIPositionConstraints_s_xfB = new B2Transform();
	private static SolveTOIPositionConstraints_s_psm = new B2PositionSolverManifold();
	private static SolveTOIPositionConstraints_s_rA = new B2Vec2();
	private static SolveTOIPositionConstraints_s_rB = new B2Vec2();
	private static SolveTOIPositionConstraints_s_P = new B2Vec2();

	public SolveTOIPositionConstraints(toiIndexA: number, toiIndexB: number): boolean {
		const xfA: B2Transform = B2ContactSolver.SolveTOIPositionConstraints_s_xfA;
		const xfB: B2Transform = B2ContactSolver.SolveTOIPositionConstraints_s_xfB;
		const psm: B2PositionSolverManifold = B2ContactSolver.SolveTOIPositionConstraints_s_psm;
		const rA: B2Vec2 = B2ContactSolver.SolveTOIPositionConstraints_s_rA;
		const rB: B2Vec2 = B2ContactSolver.SolveTOIPositionConstraints_s_rB;
		const P: B2Vec2 = B2ContactSolver.SolveTOIPositionConstraints_s_P;

		let minSeparation: number = 0;

		for (let i: number = 0; i < this.m_count; ++i) {
			const pc: B2ContactPositionConstraint = this.m_positionConstraints[i];

			const indexA: number = pc.indexA;
			const indexB: number = pc.indexB;
			const localCenterA: B2Vec2 = pc.localCenterA;
			const localCenterB: B2Vec2 = pc.localCenterB;
			const pointCount: number = pc.pointCount;

			let mA: number = 0;
			let iA: number = 0;
			if (indexA === toiIndexA || indexA === toiIndexB) {
				mA = pc.invMassA;
				iA = pc.invIA;
			}

			let mB: number = 0;
			let iB: number = 0;
			if (indexB === toiIndexA || indexB === toiIndexB) {
				mB = pc.invMassB;
				iB = pc.invIB;
			}

			const cA: B2Vec2 = this.m_positions[indexA].c;
			let aA: number = this.m_positions[indexA].a;

			const cB: B2Vec2 = this.m_positions[indexB].c;
			let aB: number = this.m_positions[indexB].a;

			// Solve normal constraints
			for (let j: number = 0; j < pointCount; ++j) {
				xfA.q.SetAngle(aA);
				xfB.q.SetAngle(aB);
				B2Vec2.SubVV(cA, B2Rot.MulRV(xfA.q, localCenterA, B2Vec2.s_t0), xfA.p);
				B2Vec2.SubVV(cB, B2Rot.MulRV(xfB.q, localCenterB, B2Vec2.s_t0), xfB.p);

				psm.Initialize(pc, xfA, xfB, j);
				const normal: B2Vec2 = psm.normal;

				const point: B2Vec2 = psm.point;
				const separation: number = psm.separation;

				// B2Vec2 rA = point - cA;
				B2Vec2.SubVV(point, cA, rA);
				// B2Vec2 rB = point - cB;
				B2Vec2.SubVV(point, cB, rB);

				// Track max constraint error.
				minSeparation = B2Min(minSeparation, separation);

				// Prevent large corrections and allow slop.
				const C: number = B2Clamp(B2_toiBaumgarte * (separation + B2_linearSlop), (-B2_maxLinearCorrection), 0);

				// Compute the effective mass.
				// float32 rnA = B2Cross(rA, normal);
				const rnA: number = B2Vec2.CrossVV(rA, normal);
				// float32 rnB = B2Cross(rB, normal);
				const rnB: number = B2Vec2.CrossVV(rB, normal);
				// float32 K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
				const K: number = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

				// Compute normal impulse
				const impulse: number = K > 0 ? -C / K : 0;

				// B2Vec2 P = impulse * normal;
				B2Vec2.MulSV(impulse, normal, P);

				// cA -= mA * P;
				cA.SelfMulSub(mA, P);
				// aA -= iA * B2Cross(rA, P);
				aA -= iA * B2Vec2.CrossVV(rA, P);

				// cB += mB * P;
				cB.SelfMulAdd(mB, P);
				// aB += iB * B2Cross(rB, P);
				aB += iB * B2Vec2.CrossVV(rB, P);
			}

			// this.m_positions[indexA].c = cA;
			this.m_positions[indexA].a = aA;

			// this.m_positions[indexB].c = cB;
			this.m_positions[indexB].a = aB;
		}

		// We can't expect minSpeparation >= -B2_linearSlop because we don't
		// push the separation above -B2_linearSlop.
		return minSeparation >= -1.5 * B2_linearSlop;
	}
}
