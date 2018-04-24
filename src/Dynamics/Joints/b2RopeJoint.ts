/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

import {B2_linearSlop, B2_maxLinearCorrection} from '../../Common/b2Settings';
import {B2Min, B2Clamp, B2Vec2, B2Rot} from '../../Common/b2Math';
import {B2Joint, B2JointDef, B2JointType, B2LimitState} from './b2Joint';
import {B2SolverData} from '../b2TimeStep';

///  Rope joint definition. This requires two body anchor points and
///  a maximum lengths.
///  Note: by default the connected objects will not collide.
///  see collideConnected in B2JointDef.
export class B2RopeJointDef extends B2JointDef {
	public localAnchorA: B2Vec2 = new B2Vec2(-1, 0);

	public localAnchorB: B2Vec2 = new B2Vec2(1, 0);

	public maxLength: number = 0;

	constructor() {
		super(B2JointType.e_ropeJoint);
	}
}

export class B2RopeJoint extends B2Joint {
	// Solver shared
	public m_localAnchorA: B2Vec2 = new B2Vec2();
	public m_localAnchorB: B2Vec2 = new B2Vec2();
	public m_maxLength: number = 0;
	public m_length: number = 0;
	public m_impulse: number = 0;

	// Solver temp
	public m_indexA: number = 0;
	public m_indexB: number = 0;
	public m_u: B2Vec2 = new B2Vec2();
	public m_rA: B2Vec2 = new B2Vec2();
	public m_rB: B2Vec2 = new B2Vec2();
	public m_localCenterA: B2Vec2 = new B2Vec2();
	public m_localCenterB: B2Vec2 = new B2Vec2();
	public m_invMassA: number = 0;
	public m_invMassB: number = 0;
	public m_invIA: number = 0;
	public m_invIB: number = 0;
	public m_mass: number = 0;
	public m_state = B2LimitState.e_inactiveLimit;

	public m_qA: B2Rot = new B2Rot();
	public m_qB: B2Rot = new B2Rot();
	public m_lalcA: B2Vec2 = new B2Vec2();
	public m_lalcB: B2Vec2 = new B2Vec2();

	constructor(def: B2RopeJointDef) {
		super(def);

		this.m_localAnchorA.Copy(def.localAnchorA);
		this.m_localAnchorB.Copy(def.localAnchorB);
		this.m_maxLength = def.maxLength;
	}

	private static InitVelocityConstraints_s_P = new B2Vec2();

	public InitVelocityConstraints(data: B2SolverData): void {
		this.m_indexA = this.m_bodyA.m_islandIndex;
		this.m_indexB = this.m_bodyB.m_islandIndex;
		this.m_localCenterA.Copy(this.m_bodyA.m_sweep.localCenter);
		this.m_localCenterB.Copy(this.m_bodyB.m_sweep.localCenter);
		this.m_invMassA = this.m_bodyA.m_invMass;
		this.m_invMassB = this.m_bodyB.m_invMass;
		this.m_invIA = this.m_bodyA.m_invI;
		this.m_invIB = this.m_bodyB.m_invI;

		const cA: B2Vec2 = data.positions[this.m_indexA].c;
		const aA: number = data.positions[this.m_indexA].a;
		const vA: B2Vec2 = data.velocities[this.m_indexA].v;
		let wA: number = data.velocities[this.m_indexA].w;

		const cB: B2Vec2 = data.positions[this.m_indexB].c;
		const aB: number = data.positions[this.m_indexB].a;
		const vB: B2Vec2 = data.velocities[this.m_indexB].v;
		let wB: number = data.velocities[this.m_indexB].w;

		const qA: B2Rot = this.m_qA.SetAngle(aA), qB: B2Rot = this.m_qB.SetAngle(aB);

		// this.m_rA = B2Mul(qA, this.m_localAnchorA - this.m_localCenterA);
		B2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
		B2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
		// this.m_rB = B2Mul(qB, this.m_localAnchorB - this.m_localCenterB);
		B2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
		B2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
		// this.m_u = cB + this.m_rB - cA - this.m_rA;
		this.m_u.Copy(cB).SelfAdd(this.m_rB).SelfSub(cA).SelfSub(this.m_rA);

		this.m_length = this.m_u.Length();

		const C: number = this.m_length - this.m_maxLength;
		if (C > 0) {
			this.m_state = B2LimitState.e_atUpperLimit;
		} else {
			this.m_state = B2LimitState.e_inactiveLimit;
		}

		if (this.m_length > B2_linearSlop) {
			this.m_u.SelfMul(1 / this.m_length);
		} else {
			this.m_u.SetZero();
			this.m_mass = 0;
			this.m_impulse = 0;
			return;
		}

		// Compute effective mass.
		const crA: number = B2Vec2.CrossVV(this.m_rA, this.m_u);
		const crB: number = B2Vec2.CrossVV(this.m_rB, this.m_u);
		const invMass: number = this.m_invMassA + this.m_invIA * crA * crA + this.m_invMassB + this.m_invIB * crB * crB;

		this.m_mass = invMass !== 0 ? 1 / invMass : 0;

		if (data.step.warmStarting) {
			// Scale the impulse to support a variable time step.
			this.m_impulse *= data.step.dtRatio;

			// B2Vec2 P = m_impulse * m_u;
			const P: B2Vec2 = B2Vec2.MulSV(this.m_impulse, this.m_u, B2RopeJoint.InitVelocityConstraints_s_P);
			// vA -= m_invMassA * P;
			vA.SelfMulSub(this.m_invMassA, P);
			wA -= this.m_invIA * B2Vec2.CrossVV(this.m_rA, P);
			// vB += m_invMassB * P;
			vB.SelfMulAdd(this.m_invMassB, P);
			wB += this.m_invIB * B2Vec2.CrossVV(this.m_rB, P);
		} else {
			this.m_impulse = 0;
		}

		// data.velocities[this.m_indexA].v = vA;
		data.velocities[this.m_indexA].w = wA;
		// data.velocities[this.m_indexB].v = vB;
		data.velocities[this.m_indexB].w = wB;
	}

	private static SolveVelocityConstraints_s_vpA = new B2Vec2();
	private static SolveVelocityConstraints_s_vpB = new B2Vec2();
	private static SolveVelocityConstraints_s_P = new B2Vec2();

	public SolveVelocityConstraints(data: B2SolverData): void {
		const vA: B2Vec2 = data.velocities[this.m_indexA].v;
		let wA: number = data.velocities[this.m_indexA].w;
		const vB: B2Vec2 = data.velocities[this.m_indexB].v;
		let wB: number = data.velocities[this.m_indexB].w;

		// Cdot = dot(u, v + cross(w, r))
		// B2Vec2 vpA = vA + B2Cross(wA, m_rA);
		const vpA: B2Vec2 = B2Vec2.AddVCrossSV(vA, wA, this.m_rA, B2RopeJoint.SolveVelocityConstraints_s_vpA);
		// B2Vec2 vpB = vB + B2Cross(wB, m_rB);
		const vpB: B2Vec2 = B2Vec2.AddVCrossSV(vB, wB, this.m_rB, B2RopeJoint.SolveVelocityConstraints_s_vpB);
		// float32 C = m_length - m_maxLength;
		const C: number = this.m_length - this.m_maxLength;
		// float32 Cdot = B2Dot(m_u, vpB - vpA);
		let Cdot: number = B2Vec2.DotVV(this.m_u, B2Vec2.SubVV(vpB, vpA, B2Vec2.s_t0));

		// Predictive constraint.
		if (C < 0) {
			Cdot += data.step.inv_dt * C;
		}

		let impulse: number = -this.m_mass * Cdot;
		const oldImpulse: number = this.m_impulse;
		this.m_impulse = B2Min(0, this.m_impulse + impulse);
		impulse = this.m_impulse - oldImpulse;

		// B2Vec2 P = impulse * m_u;
		const P: B2Vec2 = B2Vec2.MulSV(impulse, this.m_u, B2RopeJoint.SolveVelocityConstraints_s_P);
		// vA -= m_invMassA * P;
		vA.SelfMulSub(this.m_invMassA, P);
		wA -= this.m_invIA * B2Vec2.CrossVV(this.m_rA, P);
		// vB += m_invMassB * P;
		vB.SelfMulAdd(this.m_invMassB, P);
		wB += this.m_invIB * B2Vec2.CrossVV(this.m_rB, P);

		// data.velocities[this.m_indexA].v = vA;
		data.velocities[this.m_indexA].w = wA;
		// data.velocities[this.m_indexB].v = vB;
		data.velocities[this.m_indexB].w = wB;
	}

	private static SolvePositionConstraints_s_P = new B2Vec2();

	public SolvePositionConstraints(data: B2SolverData): boolean {
		const cA: B2Vec2 = data.positions[this.m_indexA].c;
		let aA: number = data.positions[this.m_indexA].a;
		const cB: B2Vec2 = data.positions[this.m_indexB].c;
		let aB: number = data.positions[this.m_indexB].a;

		const qA: B2Rot = this.m_qA.SetAngle(aA), qB: B2Rot = this.m_qB.SetAngle(aB);

		// B2Vec2 rA = B2Mul(qA, this.m_localAnchorA - this.m_localCenterA);
		B2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
		const rA: B2Vec2 = B2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
		// B2Vec2 rB = B2Mul(qB, this.m_localAnchorB - this.m_localCenterB);
		B2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
		const rB: B2Vec2 = B2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
		// B2Vec2 u = cB + rB - cA - rA;
		const u: B2Vec2 = this.m_u.Copy(cB).SelfAdd(rB).SelfSub(cA).SelfSub(rA);

		const length: number = u.Normalize();
		let C: number = length - this.m_maxLength;

		C = B2Clamp(C, 0, B2_maxLinearCorrection);

		const impulse: number = -this.m_mass * C;
		// B2Vec2 P = impulse * u;
		const P: B2Vec2 = B2Vec2.MulSV(impulse, u, B2RopeJoint.SolvePositionConstraints_s_P);

		// cA -= m_invMassA * P;
		cA.SelfMulSub(this.m_invMassA, P);
		aA -= this.m_invIA * B2Vec2.CrossVV(rA, P);
		// cB += m_invMassB * P;
		cB.SelfMulAdd(this.m_invMassB, P);
		aB += this.m_invIB * B2Vec2.CrossVV(rB, P);

		// data.positions[this.m_indexA].c = cA;
		data.positions[this.m_indexA].a = aA;
		// data.positions[this.m_indexB].c = cB;
		data.positions[this.m_indexB].a = aB;

		return length - this.m_maxLength < B2_linearSlop;
	}

	public GetAnchorA(out: B2Vec2): B2Vec2 {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
	}

	public GetAnchorB(out: B2Vec2): B2Vec2 {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
	}

	public GetReactionForce(inv_dt: number, out: B2Vec2): B2Vec2 {
		const F: B2Vec2 = B2Vec2.MulSV((inv_dt * this.m_impulse), this.m_u, out);
		return F;
		// return out.Set(inv_dt * this.m_linearImpulse.x, inv_dt * this.m_linearImpulse.y);
	}

	public GetReactionTorque(inv_dt: number): number {
		return 0;
	}

	public GetLocalAnchorA(): B2Vec2 {
		return this.m_localAnchorA;
	}

	public GetLocalAnchorB(): B2Vec2 {
		return this.m_localAnchorB;
	}

	public SetMaxLength(length: number): void {
		this.m_maxLength = length;
	}

	public GetMaxLength(): number {
		return this.m_maxLength;
	}

	public GetLimitState(): B2LimitState {
		return this.m_state;
	}

	public Dump(log: (format: string, ...args: any[]) => void): void {
		const indexA = this.m_bodyA.m_islandIndex;
		const indexB = this.m_bodyB.m_islandIndex;

		log('  const jd: B2RopeJointDef = new B2RopeJointDef();\n');
		log('  jd.bodyA = bodies[%d];\n', indexA);
		log('  jd.bodyB = bodies[%d];\n', indexB);
		log('  jd.collideConnected = %s;\n', (this.m_collideConnected) ? ('true') : ('false'));
		log('  jd.localAnchorA.Set(%.15f, %.15f);\n', this.m_localAnchorA.x, this.m_localAnchorA.y);
		log('  jd.localAnchorB.Set(%.15f, %.15f);\n', this.m_localAnchorB.x, this.m_localAnchorB.y);
		log('  jd.maxLength = %.15f;\n', this.m_maxLength);
		log('  joints[%d] = this.m_world.CreateJoint(jd);\n', this.m_index);
	}
}
