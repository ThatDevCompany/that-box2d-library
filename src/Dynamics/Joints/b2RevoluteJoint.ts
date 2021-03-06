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

import {
	B2_linearSlop,
	B2_angularSlop,
	B2_maxAngularCorrection
} from '../../Common/b2Settings'
import {
	B2Abs,
	B2Clamp,
	B2Vec2,
	B2Mat22,
	B2Vec3,
	B2Mat33,
	B2Rot
} from '../../Common/b2Math'
import { B2Body } from '../b2Body'
import { B2Joint, B2JointDef, B2JointType, B2LimitState } from './b2Joint'
import { B2SolverData } from '../b2TimeStep'

///  Revolute joint definition. This requires defining an
///  anchor point where the bodies are joined. The definition
///  uses local anchor points so that the initial configuration
///  can violate the constraint slightly. You also need to
///  specify the initial relative angle for joint limits. This
///  helps when saving and loading a game.
///  The local anchor points are measured from the body's origin
///  rather than the center of mass because:
///  1. you might not know where the center of mass will be.
///  2. if you add/remove shapes from a body and recompute the mass,
///     the joints will be broken.
export class B2RevoluteJointDef extends B2JointDef {
	public localAnchorA: B2Vec2 = new B2Vec2(0, 0)

	public localAnchorB: B2Vec2 = new B2Vec2(0, 0)

	public referenceAngle: number = 0

	public enableLimit = false

	public lowerAngle: number = 0

	public upperAngle: number = 0

	public enableMotor = false

	public motorSpeed: number = 0

	public maxMotorTorque: number = 0

	constructor() {
		super(B2JointType.e_revoluteJoint)
	}

	public Initialize(bA: B2Body, bB: B2Body, anchor: B2Vec2): void {
		this.bodyA = bA
		this.bodyB = bB
		this.bodyA.GetLocalPoint(anchor, this.localAnchorA)
		this.bodyB.GetLocalPoint(anchor, this.localAnchorB)
		this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle()
	}
}

export class B2RevoluteJoint extends B2Joint {
	// Solver shared
	public m_localAnchorA: B2Vec2 = new B2Vec2()
	public m_localAnchorB: B2Vec2 = new B2Vec2()
	public m_impulse: B2Vec3 = new B2Vec3()
	public m_motorImpulse: number = 0

	public m_enableMotor: boolean = false
	public m_maxMotorTorque: number = 0
	public m_motorSpeed: number = 0

	public m_enableLimit: boolean = false
	public m_referenceAngle: number = 0
	public m_lowerAngle: number = 0
	public m_upperAngle: number = 0

	// Solver temp
	public m_indexA: number = 0
	public m_indexB: number = 0
	public m_rA: B2Vec2 = new B2Vec2()
	public m_rB: B2Vec2 = new B2Vec2()
	public m_localCenterA: B2Vec2 = new B2Vec2()
	public m_localCenterB: B2Vec2 = new B2Vec2()
	public m_invMassA: number = 0
	public m_invMassB: number = 0
	public m_invIA: number = 0
	public m_invIB: number = 0
	public m_mass: B2Mat33 = new B2Mat33() // effective mass for point-to-point constraint.
	public m_motorMass: number = 0 // effective mass for motor/limit angular constraint.
	public m_limitState: B2LimitState = B2LimitState.e_inactiveLimit

	public m_qA: B2Rot = new B2Rot()
	public m_qB: B2Rot = new B2Rot()
	public m_lalcA: B2Vec2 = new B2Vec2()
	public m_lalcB: B2Vec2 = new B2Vec2()
	public m_K: B2Mat22 = new B2Mat22()

	constructor(def: B2RevoluteJointDef) {
		super(def)

		this.m_localAnchorA.Copy(def.localAnchorA)
		this.m_localAnchorB.Copy(def.localAnchorB)
		this.m_referenceAngle = def.referenceAngle

		this.m_impulse.SetZero()
		this.m_motorImpulse = 0

		this.m_lowerAngle = def.lowerAngle
		this.m_upperAngle = def.upperAngle
		this.m_maxMotorTorque = def.maxMotorTorque
		this.m_motorSpeed = def.motorSpeed
		this.m_enableLimit = def.enableLimit
		this.m_enableMotor = def.enableMotor
		this.m_limitState = B2LimitState.e_inactiveLimit
	}

	private static InitVelocityConstraints_s_P = new B2Vec2()

	public InitVelocityConstraints(data: B2SolverData): void {
		this.m_indexA = this.m_bodyA.m_islandIndex
		this.m_indexB = this.m_bodyB.m_islandIndex
		this.m_localCenterA.Copy(this.m_bodyA.m_sweep.localCenter)
		this.m_localCenterB.Copy(this.m_bodyB.m_sweep.localCenter)
		this.m_invMassA = this.m_bodyA.m_invMass
		this.m_invMassB = this.m_bodyB.m_invMass
		this.m_invIA = this.m_bodyA.m_invI
		this.m_invIB = this.m_bodyB.m_invI

		const aA: number = data.positions[this.m_indexA].a
		const vA: B2Vec2 = data.velocities[this.m_indexA].v
		let wA: number = data.velocities[this.m_indexA].w

		const aB: number = data.positions[this.m_indexB].a
		const vB: B2Vec2 = data.velocities[this.m_indexB].v
		let wB: number = data.velocities[this.m_indexB].w

		// B2Rot qA(aA), qB(aB);
		const qA: B2Rot = this.m_qA.SetAngle(aA),
			qB: B2Rot = this.m_qB.SetAngle(aB)

		// m_rA = B2Mul(qA, m_localAnchorA - m_localCenterA);
		B2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA)
		B2Rot.MulRV(qA, this.m_lalcA, this.m_rA)
		// m_rB = B2Mul(qB, m_localAnchorB - m_localCenterB);
		B2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB)
		B2Rot.MulRV(qB, this.m_lalcB, this.m_rB)

		// J = [-I -r1_skew I r2_skew]
		//     [ 0       -1 0       1]
		// r_skew = [-ry; rx]

		// Matlab
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
		//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

		const mA: number = this.m_invMassA,
			mB: number = this.m_invMassB
		const iA: number = this.m_invIA,
			iB: number = this.m_invIB

		const fixedRotation: boolean = iA + iB === 0

		this.m_mass.ex.x =
			mA + mB + this.m_rA.y * this.m_rA.y * iA + this.m_rB.y * this.m_rB.y * iB
		this.m_mass.ey.x =
			-this.m_rA.y * this.m_rA.x * iA - this.m_rB.y * this.m_rB.x * iB
		this.m_mass.ez.x = -this.m_rA.y * iA - this.m_rB.y * iB
		this.m_mass.ex.y = this.m_mass.ey.x
		this.m_mass.ey.y =
			mA + mB + this.m_rA.x * this.m_rA.x * iA + this.m_rB.x * this.m_rB.x * iB
		this.m_mass.ez.y = this.m_rA.x * iA + this.m_rB.x * iB
		this.m_mass.ex.z = this.m_mass.ez.x
		this.m_mass.ey.z = this.m_mass.ez.y
		this.m_mass.ez.z = iA + iB

		this.m_motorMass = iA + iB
		if (this.m_motorMass > 0) {
			this.m_motorMass = 1 / this.m_motorMass
		}

		if (!this.m_enableMotor || fixedRotation) {
			this.m_motorImpulse = 0
		}

		if (this.m_enableLimit && !fixedRotation) {
			const jointAngle: number = aB - aA - this.m_referenceAngle
			if (B2Abs(this.m_upperAngle - this.m_lowerAngle) < 2 * B2_angularSlop) {
				this.m_limitState = B2LimitState.e_equalLimits
			} else if (jointAngle <= this.m_lowerAngle) {
				if (this.m_limitState !== B2LimitState.e_atLowerLimit) {
					this.m_impulse.z = 0
				}
				this.m_limitState = B2LimitState.e_atLowerLimit
			} else if (jointAngle >= this.m_upperAngle) {
				if (this.m_limitState !== B2LimitState.e_atUpperLimit) {
					this.m_impulse.z = 0
				}
				this.m_limitState = B2LimitState.e_atUpperLimit
			} else {
				this.m_limitState = B2LimitState.e_inactiveLimit
				this.m_impulse.z = 0
			}
		} else {
			this.m_limitState = B2LimitState.e_inactiveLimit
		}

		if (data.step.warmStarting) {
			// Scale impulses to support a variable time step.
			this.m_impulse.SelfMul(data.step.dtRatio)
			this.m_motorImpulse *= data.step.dtRatio

			// B2Vec2 P(m_impulse.x, m_impulse.y);
			const P: B2Vec2 = B2RevoluteJoint.InitVelocityConstraints_s_P.Set(
				this.m_impulse.x,
				this.m_impulse.y
			)

			// vA -= mA * P;
			vA.SelfMulSub(mA, P)
			wA -=
				iA *
				(B2Vec2.CrossVV(this.m_rA, P) + this.m_motorImpulse + this.m_impulse.z)

			// vB += mB * P;
			vB.SelfMulAdd(mB, P)
			wB +=
				iB *
				(B2Vec2.CrossVV(this.m_rB, P) + this.m_motorImpulse + this.m_impulse.z)
		} else {
			this.m_impulse.SetZero()
			this.m_motorImpulse = 0
		}

		// data.velocities[this.m_indexA].v = vA;
		data.velocities[this.m_indexA].w = wA
		// data.velocities[this.m_indexB].v = vB;
		data.velocities[this.m_indexB].w = wB
	}

	private static SolveVelocityConstraints_s_P: B2Vec2 = new B2Vec2()
	private static SolveVelocityConstraints_s_Cdot_v2: B2Vec2 = new B2Vec2()
	private static SolveVelocityConstraints_s_Cdot1: B2Vec2 = new B2Vec2()
	private static SolveVelocityConstraints_s_impulse_v3: B2Vec3 = new B2Vec3()
	private static SolveVelocityConstraints_s_reduced_v2: B2Vec2 = new B2Vec2()
	private static SolveVelocityConstraints_s_impulse_v2: B2Vec2 = new B2Vec2()

	public SolveVelocityConstraints(data: B2SolverData): void {
		const vA: B2Vec2 = data.velocities[this.m_indexA].v
		let wA: number = data.velocities[this.m_indexA].w
		const vB: B2Vec2 = data.velocities[this.m_indexB].v
		let wB: number = data.velocities[this.m_indexB].w

		const mA: number = this.m_invMassA,
			mB: number = this.m_invMassB
		const iA: number = this.m_invIA,
			iB: number = this.m_invIB

		const fixedRotation: boolean = iA + iB === 0

		// Solve motor constraint.
		if (
			this.m_enableMotor &&
			this.m_limitState !== B2LimitState.e_equalLimits &&
			!fixedRotation
		) {
			const Cdot: number = wB - wA - this.m_motorSpeed
			let impulse: number = -this.m_motorMass * Cdot
			const oldImpulse: number = this.m_motorImpulse
			const maxImpulse: number = data.step.dt * this.m_maxMotorTorque
			this.m_motorImpulse = B2Clamp(
				this.m_motorImpulse + impulse,
				-maxImpulse,
				maxImpulse
			)
			impulse = this.m_motorImpulse - oldImpulse

			wA -= iA * impulse
			wB += iB * impulse
		}

		// Solve limit constraint.
		if (
			this.m_enableLimit &&
			this.m_limitState !== B2LimitState.e_inactiveLimit &&
			!fixedRotation
		) {
			// B2Vec2 Cdot1 = vB + B2Cross(wB, m_rB) - vA - B2Cross(wA, m_rA);
			const Cdot1: B2Vec2 = B2Vec2.SubVV(
				B2Vec2.AddVCrossSV(vB, wB, this.m_rB, B2Vec2.s_t0),
				B2Vec2.AddVCrossSV(vA, wA, this.m_rA, B2Vec2.s_t1),
				B2RevoluteJoint.SolveVelocityConstraints_s_Cdot1
			)
			const Cdot2: number = wB - wA
			// B2Vec3 Cdot(Cdot1.x, Cdot1.y, Cdot2);

			// B2Vec3 impulse = -this.m_mass.Solve33(Cdot);
			const impulse_v3: B2Vec3 = this.m_mass
				.Solve33(
					Cdot1.x,
					Cdot1.y,
					Cdot2,
					B2RevoluteJoint.SolveVelocityConstraints_s_impulse_v3
				)
				.SelfNeg()

			if (this.m_limitState === B2LimitState.e_equalLimits) {
				this.m_impulse.SelfAdd(impulse_v3)
			} else if (this.m_limitState === B2LimitState.e_atLowerLimit) {
				const newImpulse: number = this.m_impulse.z + impulse_v3.z
				if (newImpulse < 0) {
					// B2Vec2 rhs = -Cdot1 + m_impulse.z * B2Vec2(m_mass.ez.x, m_mass.ez.y);
					const rhs_x = -Cdot1.x + this.m_impulse.z * this.m_mass.ez.x
					const rhs_y = -Cdot1.y + this.m_impulse.z * this.m_mass.ez.y
					const reduced_v2: B2Vec2 = this.m_mass.Solve22(
						rhs_x,
						rhs_y,
						B2RevoluteJoint.SolveVelocityConstraints_s_reduced_v2
					)
					impulse_v3.x = reduced_v2.x
					impulse_v3.y = reduced_v2.y
					impulse_v3.z = -this.m_impulse.z
					this.m_impulse.x += reduced_v2.x
					this.m_impulse.y += reduced_v2.y
					this.m_impulse.z = 0
				} else {
					this.m_impulse.SelfAdd(impulse_v3)
				}
			} else if (this.m_limitState === B2LimitState.e_atUpperLimit) {
				const newImpulse: number = this.m_impulse.z + impulse_v3.z
				if (newImpulse > 0) {
					// B2Vec2 rhs = -Cdot1 + m_impulse.z * B2Vec2(m_mass.ez.x, m_mass.ez.y);
					const rhs_x = -Cdot1.x + this.m_impulse.z * this.m_mass.ez.x
					const rhs_y = -Cdot1.y + this.m_impulse.z * this.m_mass.ez.y
					const reduced_v2: B2Vec2 = this.m_mass.Solve22(
						rhs_x,
						rhs_y,
						B2RevoluteJoint.SolveVelocityConstraints_s_reduced_v2
					)
					impulse_v3.x = reduced_v2.x
					impulse_v3.y = reduced_v2.y
					impulse_v3.z = -this.m_impulse.z
					this.m_impulse.x += reduced_v2.x
					this.m_impulse.y += reduced_v2.y
					this.m_impulse.z = 0
				} else {
					this.m_impulse.SelfAdd(impulse_v3)
				}
			}

			// B2Vec2 P(impulse.x, impulse.y);
			const P: B2Vec2 = B2RevoluteJoint.SolveVelocityConstraints_s_P.Set(
				impulse_v3.x,
				impulse_v3.y
			)

			// vA -= mA * P;
			vA.SelfMulSub(mA, P)
			wA -= iA * (B2Vec2.CrossVV(this.m_rA, P) + impulse_v3.z)

			// vB += mB * P;
			vB.SelfMulAdd(mB, P)
			wB += iB * (B2Vec2.CrossVV(this.m_rB, P) + impulse_v3.z)
		} else {
			// Solve point-to-point constraint
			// B2Vec2 Cdot = vB + B2Cross(wB, m_rB) - vA - B2Cross(wA, m_rA);
			const Cdot_v2: B2Vec2 = B2Vec2.SubVV(
				B2Vec2.AddVCrossSV(vB, wB, this.m_rB, B2Vec2.s_t0),
				B2Vec2.AddVCrossSV(vA, wA, this.m_rA, B2Vec2.s_t1),
				B2RevoluteJoint.SolveVelocityConstraints_s_Cdot_v2
			)
			// B2Vec2 impulse = m_mass.Solve22(-Cdot);
			const impulse_v2: B2Vec2 = this.m_mass.Solve22(
				-Cdot_v2.x,
				-Cdot_v2.y,
				B2RevoluteJoint.SolveVelocityConstraints_s_impulse_v2
			)

			this.m_impulse.x += impulse_v2.x
			this.m_impulse.y += impulse_v2.y

			// vA -= mA * impulse;
			vA.SelfMulSub(mA, impulse_v2)
			wA -= iA * B2Vec2.CrossVV(this.m_rA, impulse_v2)

			// vB += mB * impulse;
			vB.SelfMulAdd(mB, impulse_v2)
			wB += iB * B2Vec2.CrossVV(this.m_rB, impulse_v2)
		}

		// data.velocities[this.m_indexA].v = vA;
		data.velocities[this.m_indexA].w = wA
		// data.velocities[this.m_indexB].v = vB;
		data.velocities[this.m_indexB].w = wB
	}

	private static SolvePositionConstraints_s_C_v2 = new B2Vec2()
	private static SolvePositionConstraints_s_impulse = new B2Vec2()

	public SolvePositionConstraints(data: B2SolverData): boolean {
		const cA: B2Vec2 = data.positions[this.m_indexA].c
		let aA: number = data.positions[this.m_indexA].a
		const cB: B2Vec2 = data.positions[this.m_indexB].c
		let aB: number = data.positions[this.m_indexB].a

		// B2Rot qA(aA), qB(aB);
		const qA: B2Rot = this.m_qA.SetAngle(aA),
			qB: B2Rot = this.m_qB.SetAngle(aB)

		let angularError: number = 0
		let positionError: number = 0

		const fixedRotation: boolean = this.m_invIA + this.m_invIB === 0

		// Solve angular limit constraint.
		if (
			this.m_enableLimit &&
			this.m_limitState !== B2LimitState.e_inactiveLimit &&
			!fixedRotation
		) {
			const angle: number = aB - aA - this.m_referenceAngle
			let limitImpulse: number = 0

			if (this.m_limitState === B2LimitState.e_equalLimits) {
				// Prevent large angular corrections
				const C: number = B2Clamp(
					angle - this.m_lowerAngle,
					-B2_maxAngularCorrection,
					B2_maxAngularCorrection
				)
				limitImpulse = -this.m_motorMass * C
				angularError = B2Abs(C)
			} else if (this.m_limitState === B2LimitState.e_atLowerLimit) {
				let C: number = angle - this.m_lowerAngle
				angularError = -C

				// Prevent large angular corrections and allow some slop.
				C = B2Clamp(C + B2_angularSlop, -B2_maxAngularCorrection, 0)
				limitImpulse = -this.m_motorMass * C
			} else if (this.m_limitState === B2LimitState.e_atUpperLimit) {
				let C: number = angle - this.m_upperAngle
				angularError = C

				// Prevent large angular corrections and allow some slop.
				C = B2Clamp(C - B2_angularSlop, 0, B2_maxAngularCorrection)
				limitImpulse = -this.m_motorMass * C
			}

			aA -= this.m_invIA * limitImpulse
			aB += this.m_invIB * limitImpulse
		}

		// Solve point-to-point constraint.
		{
			qA.SetAngle(aA)
			qB.SetAngle(aB)
			// B2Vec2 rA = B2Mul(qA, m_localAnchorA - m_localCenterA);
			B2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA)
			const rA: B2Vec2 = B2Rot.MulRV(qA, this.m_lalcA, this.m_rA)
			// B2Vec2 rB = B2Mul(qB, m_localAnchorB - m_localCenterB);
			B2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB)
			const rB: B2Vec2 = B2Rot.MulRV(qB, this.m_lalcB, this.m_rB)

			// B2Vec2 C = cB + rB - cA - rA;
			const C_v2 = B2Vec2.SubVV(
				B2Vec2.AddVV(cB, rB, B2Vec2.s_t0),
				B2Vec2.AddVV(cA, rA, B2Vec2.s_t1),
				B2RevoluteJoint.SolvePositionConstraints_s_C_v2
			)
			// positionError = C.Length();
			positionError = C_v2.Length()

			const mA: number = this.m_invMassA,
				mB: number = this.m_invMassB
			const iA: number = this.m_invIA,
				iB: number = this.m_invIB

			const K: B2Mat22 = this.m_K
			K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y
			K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y
			K.ey.x = K.ex.y
			K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x

			// B2Vec2 impulse = -K.Solve(C);
			const impulse: B2Vec2 = K.Solve(
				C_v2.x,
				C_v2.y,
				B2RevoluteJoint.SolvePositionConstraints_s_impulse
			).SelfNeg()

			// cA -= mA * impulse;
			cA.SelfMulSub(mA, impulse)
			aA -= iA * B2Vec2.CrossVV(rA, impulse)

			// cB += mB * impulse;
			cB.SelfMulAdd(mB, impulse)
			aB += iB * B2Vec2.CrossVV(rB, impulse)
		}

		// data.positions[this.m_indexA].c = cA;
		data.positions[this.m_indexA].a = aA
		// data.positions[this.m_indexB].c = cB;
		data.positions[this.m_indexB].a = aB

		return positionError <= B2_linearSlop && angularError <= B2_angularSlop
	}

	public GetAnchorA(out: B2Vec2): B2Vec2 {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out)
	}

	public GetAnchorB(out: B2Vec2): B2Vec2 {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out)
	}

	public GetReactionForce(inv_dt: number, out: B2Vec2): B2Vec2 {
		// B2Vec2 P(this.m_impulse.x, this.m_impulse.y);
		// return inv_dt * P;
		return out.Set(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y)
	}

	public GetReactionTorque(inv_dt: number): number {
		return inv_dt * this.m_impulse.z
	}

	public GetLocalAnchorA(): B2Vec2 {
		return this.m_localAnchorA
	}

	public GetLocalAnchorB(): B2Vec2 {
		return this.m_localAnchorB
	}

	public GetReferenceAngle() {
		return this.m_referenceAngle
	}

	public GetJointAngle(): number {
		// B2Body* bA = this.m_bodyA;
		// B2Body* bB = this.m_bodyB;
		// return bB->this.m_sweep.a - bA->this.m_sweep.a - this.m_referenceAngle;
		return (
			this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a - this.m_referenceAngle
		)
	}

	public GetJointSpeed(): number {
		// B2Body* bA = this.m_bodyA;
		// B2Body* bB = this.m_bodyB;
		// return bB->this.m_angularVelocity - bA->this.m_angularVelocity;
		return this.m_bodyB.m_angularVelocity - this.m_bodyA.m_angularVelocity
	}

	public IsMotorEnabled(): boolean {
		return this.m_enableMotor
	}

	public EnableMotor(flag: boolean): void {
		if (this.m_enableMotor !== flag) {
			this.m_bodyA.SetAwake(true)
			this.m_bodyB.SetAwake(true)
			this.m_enableMotor = flag
		}
	}

	public GetMotorTorque(inv_dt: number): number {
		return inv_dt * this.m_motorImpulse
	}

	public GetMotorSpeed(): number {
		return this.m_motorSpeed
	}

	public SetMaxMotorTorque(torque: number): void {
		this.m_maxMotorTorque = torque
	}

	public GetMaxMotorTorque(): number {
		return this.m_maxMotorTorque
	}

	public IsLimitEnabled(): boolean {
		return this.m_enableLimit
	}

	public EnableLimit(flag: boolean): void {
		if (flag !== this.m_enableLimit) {
			this.m_bodyA.SetAwake(true)
			this.m_bodyB.SetAwake(true)
			this.m_enableLimit = flag
			this.m_impulse.z = 0
		}
	}

	public GetLowerLimit(): number {
		return this.m_lowerAngle
	}

	public GetUpperLimit(): number {
		return this.m_upperAngle
	}

	public SetLimits(lower: number, upper: number): void {
		if (lower !== this.m_lowerAngle || upper !== this.m_upperAngle) {
			this.m_bodyA.SetAwake(true)
			this.m_bodyB.SetAwake(true)
			this.m_impulse.z = 0
			this.m_lowerAngle = lower
			this.m_upperAngle = upper
		}
	}

	public SetMotorSpeed(speed: number): void {
		if (this.m_motorSpeed !== speed) {
			this.m_bodyA.SetAwake(true)
			this.m_bodyB.SetAwake(true)
			this.m_motorSpeed = speed
		}
	}

	public Dump(log: (format: string, ...args: any[]) => void) {
		const indexA = this.m_bodyA.m_islandIndex
		const indexB = this.m_bodyB.m_islandIndex

		log('  const jd: B2RevoluteJointDef = new B2RevoluteJointDef();\n')
		log('  jd.bodyA = bodies[%d];\n', indexA)
		log('  jd.bodyB = bodies[%d];\n', indexB)
		log(
			'  jd.collideConnected = %s;\n',
			this.m_collideConnected ? 'true' : 'false'
		)
		log(
			'  jd.localAnchorA.Set(%.15f, %.15f);\n',
			this.m_localAnchorA.x,
			this.m_localAnchorA.y
		)
		log(
			'  jd.localAnchorB.Set(%.15f, %.15f);\n',
			this.m_localAnchorB.x,
			this.m_localAnchorB.y
		)
		log('  jd.referenceAngle = %.15f;\n', this.m_referenceAngle)
		log('  jd.enableLimit = %s;\n', this.m_enableLimit ? 'true' : 'false')
		log('  jd.lowerAngle = %.15f;\n', this.m_lowerAngle)
		log('  jd.upperAngle = %.15f;\n', this.m_upperAngle)
		log('  jd.enableMotor = %s;\n', this.m_enableMotor ? 'true' : 'false')
		log('  jd.motorSpeed = %.15f;\n', this.m_motorSpeed)
		log('  jd.maxMotorTorque = %.15f;\n', this.m_maxMotorTorque)
		log('  joints[%d] = this.m_world.CreateJoint(jd);\n', this.m_index)
	}
}
