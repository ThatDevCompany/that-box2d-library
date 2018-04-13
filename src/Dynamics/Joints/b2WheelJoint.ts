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

import { B2_pi, B2_linearSlop } from '../../Common/b2Settings';
import { B2Abs, B2Clamp, B2Vec2, B2Rot } from '../../Common/b2Math';
import { B2Joint, B2JointDef, B2JointType } from './b2Joint';
import { B2SolverData } from '../b2TimeStep';
import { B2Body } from '../b2Body';

///  Wheel joint definition. This requires defining a line of
///  motion using an axis and an anchor point. The definition uses local
///  anchor points and a local axis so that the initial configuration
///  can violate the constraint slightly. The joint translation is zero
///  when the local anchor points coincide in world space. Using local
///  anchors and a local axis helps when saving and loading a game.
export class B2WheelJointDef extends B2JointDef {
  public localAnchorA: B2Vec2 = new B2Vec2(0, 0);

  public localAnchorB: B2Vec2 = new B2Vec2(0, 0);

  public localAxisA: B2Vec2 = new B2Vec2(1, 0);

  public enableMotor = false;

  public maxMotorTorque: number = 0;

  public motorSpeed: number = 0;

  public frequencyHz: number = 2;

  public dampingRatio: number = 0.7;

  constructor() {
    super(B2JointType.e_wheelJoint);
  }

  public Initialize(bA: B2Body, bB: B2Body, anchor: B2Vec2, axis: B2Vec2): void {
    this.bodyA = bA;
    this.bodyB = bB;
    this.bodyA.GetLocalPoint(anchor, this.localAnchorA);
    this.bodyB.GetLocalPoint(anchor, this.localAnchorB);
    this.bodyA.GetLocalVector(axis, this.localAxisA);
  }
}

export class B2WheelJoint extends B2Joint {
  public m_frequencyHz: number = 0;
  public m_dampingRatio: number = 0;

  // Solver shared
  public m_localAnchorA: B2Vec2 = new B2Vec2();
  public m_localAnchorB: B2Vec2 = new B2Vec2();
  public m_localXAxisA: B2Vec2 = new B2Vec2();
  public m_localYAxisA: B2Vec2 = new B2Vec2();

  public m_impulse: number = 0;
  public m_motorImpulse: number = 0;
  public m_springImpulse: number = 0;

  public m_maxMotorTorque: number = 0;
  public m_motorSpeed: number = 0;
  public m_enableMotor = false;

  // Solver temp
  public m_indexA: number = 0;
  public m_indexB: number = 0;
  public m_localCenterA: B2Vec2 = new B2Vec2();
  public m_localCenterB: B2Vec2 = new B2Vec2();
  public m_invMassA: number = 0;
  public m_invMassB: number = 0;
  public m_invIA: number = 0;
  public m_invIB: number = 0;

  public m_ax: B2Vec2 = new B2Vec2();
  public m_ay: B2Vec2 = new B2Vec2();
  public m_sAx: number = 0;
  public m_sBx: number = 0;
  public m_sAy: number = 0;
  public m_sBy: number = 0;

  public m_mass: number = 0;
  public m_motorMass: number = 0;
  public m_springMass: number = 0;

  public m_bias: number = 0;
  public m_gamma: number = 0;

  public m_qA: B2Rot = new B2Rot();
  public m_qB: B2Rot = new B2Rot();
  public m_lalcA: B2Vec2 = new B2Vec2();
  public m_lalcB: B2Vec2 = new B2Vec2();
  public m_rA: B2Vec2 = new B2Vec2();
  public m_rB: B2Vec2 = new B2Vec2();

  constructor(def: B2WheelJointDef) {
    super(def);

    this.m_frequencyHz = def.frequencyHz;
    this.m_dampingRatio = def.dampingRatio;

    this.m_localAnchorA.Copy(def.localAnchorA);
    this.m_localAnchorB.Copy(def.localAnchorB);
    this.m_localXAxisA.Copy(def.localAxisA);
    B2Vec2.CrossOneV(this.m_localXAxisA, this.m_localYAxisA);

    this.m_maxMotorTorque = def.maxMotorTorque;
    this.m_motorSpeed = def.motorSpeed;
    this.m_enableMotor = def.enableMotor;

    this.m_ax.SetZero();
    this.m_ay.SetZero();
  }

  public GetMotorSpeed(): number {
    return this.m_motorSpeed;
  }

  public GetMaxMotorTorque(): number {
    return this.m_maxMotorTorque;
  }

  public SetSpringFrequencyHz(hz: number): void {
    this.m_frequencyHz = hz;
  }

  public GetSpringFrequencyHz(): number {
    return this.m_frequencyHz;
  }

  public SetSpringDampingRatio(ratio: number): void {
    this.m_dampingRatio = ratio;
  }

  public GetSpringDampingRatio(): number {
    return this.m_dampingRatio;
  }

  private static InitVelocityConstraints_s_d = new B2Vec2();
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

    const mA: number = this.m_invMassA, mB: number = this.m_invMassB;
    const iA: number = this.m_invIA, iB: number = this.m_invIB;

    const cA: B2Vec2 = data.positions[this.m_indexA].c;
    const aA: number = data.positions[this.m_indexA].a;
    const vA: B2Vec2 = data.velocities[this.m_indexA].v;
    let wA: number = data.velocities[this.m_indexA].w;

    const cB: B2Vec2 = data.positions[this.m_indexB].c;
    const aB: number = data.positions[this.m_indexB].a;
    const vB: B2Vec2 = data.velocities[this.m_indexB].v;
    let wB: number = data.velocities[this.m_indexB].w;

    const qA: B2Rot = this.m_qA.SetAngle(aA), qB: B2Rot = this.m_qB.SetAngle(aB);

    // Compute the effective masses.
    // B2Vec2 rA = B2Mul(qA, m_localAnchorA - m_localCenterA);
    B2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
    const rA: B2Vec2 = B2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
    // B2Vec2 rB = B2Mul(qB, m_localAnchorB - m_localCenterB);
    B2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
    const rB: B2Vec2 = B2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
    // B2Vec2 d = cB + rB - cA - rA;
    const d: B2Vec2 = B2Vec2.SubVV(
      B2Vec2.AddVV(cB, rB, B2Vec2.s_t0),
      B2Vec2.AddVV(cA, rA, B2Vec2.s_t1),
      B2WheelJoint.InitVelocityConstraints_s_d);

    // Point to line constraint
    {
      // m_ay = B2Mul(qA, m_localYAxisA);
      B2Rot.MulRV(qA, this.m_localYAxisA, this.m_ay);
      // m_sAy = B2Cross(d + rA, m_ay);
      this.m_sAy = B2Vec2.CrossVV(B2Vec2.AddVV(d, rA, B2Vec2.s_t0), this.m_ay);
      // m_sBy = B2Cross(rB, m_ay);
      this.m_sBy = B2Vec2.CrossVV(rB, this.m_ay);

      this.m_mass = mA + mB + iA * this.m_sAy * this.m_sAy + iB * this.m_sBy * this.m_sBy;

      if (this.m_mass > 0) {
        this.m_mass = 1 / this.m_mass;
      }
    }

    // Spring constraint
    this.m_springMass = 0;
    this.m_bias = 0;
    this.m_gamma = 0;
    if (this.m_frequencyHz > 0) {
      // m_ax = B2Mul(qA, m_localXAxisA);
      B2Rot.MulRV(qA, this.m_localXAxisA, this.m_ax);
      // m_sAx = B2Cross(d + rA, m_ax);
      this.m_sAx = B2Vec2.CrossVV(B2Vec2.AddVV(d, rA, B2Vec2.s_t0), this.m_ax);
      // m_sBx = B2Cross(rB, m_ax);
      this.m_sBx = B2Vec2.CrossVV(rB, this.m_ax);

      const invMass: number = mA + mB + iA * this.m_sAx * this.m_sAx + iB * this.m_sBx * this.m_sBx;

      if (invMass > 0) {
        this.m_springMass = 1 / invMass;

        const C: number = B2Vec2.DotVV(d, this.m_ax);

        // Frequency
        const omega: number = 2 * B2_pi * this.m_frequencyHz;

        // Damping coefficient
        const dc: number = 2 * this.m_springMass * this.m_dampingRatio * omega;

        // Spring stiffness
        const k: number = this.m_springMass * omega * omega;

        // magic formulas
        const h: number = data.step.dt;
        this.m_gamma = h * (dc + h * k);
        if (this.m_gamma > 0) {
          this.m_gamma = 1 / this.m_gamma;
        }

        this.m_bias = C * h * k * this.m_gamma;

        this.m_springMass = invMass + this.m_gamma;
        if (this.m_springMass > 0) {
          this.m_springMass = 1 / this.m_springMass;
        }
      }
    } else {
      this.m_springImpulse = 0;
    }

    // Rotational motor
    if (this.m_enableMotor) {
      this.m_motorMass = iA + iB;
      if (this.m_motorMass > 0) {
        this.m_motorMass = 1 / this.m_motorMass;
      }
    } else {
      this.m_motorMass = 0;
      this.m_motorImpulse = 0;
    }

    if (data.step.warmStarting) {
      // Account for variable time step.
      this.m_impulse *= data.step.dtRatio;
      this.m_springImpulse *= data.step.dtRatio;
      this.m_motorImpulse *= data.step.dtRatio;

      // B2Vec2 P = m_impulse * m_ay + m_springImpulse * m_ax;
      const P: B2Vec2 = B2Vec2.AddVV(
        B2Vec2.MulSV(this.m_impulse, this.m_ay, B2Vec2.s_t0),
        B2Vec2.MulSV(this.m_springImpulse, this.m_ax, B2Vec2.s_t1),
        B2WheelJoint.InitVelocityConstraints_s_P);
      // float32 LA = m_impulse * m_sAy + m_springImpulse * m_sAx + m_motorImpulse;
      const LA: number = this.m_impulse * this.m_sAy + this.m_springImpulse * this.m_sAx + this.m_motorImpulse;
      // float32 LB = m_impulse * m_sBy + m_springImpulse * m_sBx + m_motorImpulse;
      const LB: number = this.m_impulse * this.m_sBy + this.m_springImpulse * this.m_sBx + this.m_motorImpulse;

      // vA -= m_invMassA * P;
      vA.SelfMulSub(this.m_invMassA, P);
      wA -= this.m_invIA * LA;

      // vB += m_invMassB * P;
      vB.SelfMulAdd(this.m_invMassB, P);
      wB += this.m_invIB * LB;
    } else {
      this.m_impulse = 0;
      this.m_springImpulse = 0;
      this.m_motorImpulse = 0;
    }

    // data.velocities[this.m_indexA].v = vA;
    data.velocities[this.m_indexA].w = wA;
    // data.velocities[this.m_indexB].v = vB;
    data.velocities[this.m_indexB].w = wB;
  }

  private static SolveVelocityConstraints_s_P = new B2Vec2();
  public SolveVelocityConstraints(data: B2SolverData): void {
    const mA: number = this.m_invMassA, mB: number = this.m_invMassB;
    const iA: number = this.m_invIA, iB: number = this.m_invIB;

    const vA: B2Vec2 = data.velocities[this.m_indexA].v;
    let wA: number = data.velocities[this.m_indexA].w;
    const vB: B2Vec2 = data.velocities[this.m_indexB].v;
    let wB: number = data.velocities[this.m_indexB].w;

    // Solve spring constraint
    {
      const Cdot: number = B2Vec2.DotVV(this.m_ax, B2Vec2.SubVV(vB, vA, B2Vec2.s_t0)) + this.m_sBx * wB - this.m_sAx * wA;
      const impulse: number = -this.m_springMass * (Cdot + this.m_bias + this.m_gamma * this.m_springImpulse);
      this.m_springImpulse += impulse;

      // B2Vec2 P = impulse * m_ax;
      const P: B2Vec2 = B2Vec2.MulSV(impulse, this.m_ax, B2WheelJoint.SolveVelocityConstraints_s_P);
      const LA: number = impulse * this.m_sAx;
      const LB: number = impulse * this.m_sBx;

      // vA -= mA * P;
      vA.SelfMulSub(mA, P);
      wA -= iA * LA;

      // vB += mB * P;
      vB.SelfMulAdd(mB, P);
      wB += iB * LB;
    }

    // Solve rotational motor constraint
    {
      const Cdot: number = wB - wA - this.m_motorSpeed;
      let impulse: number = -this.m_motorMass * Cdot;

      const oldImpulse: number = this.m_motorImpulse;
      const maxImpulse: number = data.step.dt * this.m_maxMotorTorque;
      this.m_motorImpulse = B2Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
      impulse = this.m_motorImpulse - oldImpulse;

      wA -= iA * impulse;
      wB += iB * impulse;
    }

    // Solve point to line constraint
    {
      const Cdot: number = B2Vec2.DotVV(this.m_ay, B2Vec2.SubVV(vB, vA, B2Vec2.s_t0)) + this.m_sBy * wB - this.m_sAy * wA;
      const impulse: number = -this.m_mass * Cdot;
      this.m_impulse += impulse;

      // B2Vec2 P = impulse * m_ay;
      const P: B2Vec2 = B2Vec2.MulSV(impulse, this.m_ay, B2WheelJoint.SolveVelocityConstraints_s_P);
      const LA: number = impulse * this.m_sAy;
      const LB: number = impulse * this.m_sBy;

      // vA -= mA * P;
      vA.SelfMulSub(mA, P);
      wA -= iA * LA;

      // vB += mB * P;
      vB.SelfMulAdd(mB, P);
      wB += iB * LB;
    }

    // data.velocities[this.m_indexA].v = vA;
    data.velocities[this.m_indexA].w = wA;
    // data.velocities[this.m_indexB].v = vB;
    data.velocities[this.m_indexB].w = wB;
  }

  private static SolvePositionConstraints_s_d = new B2Vec2();
  private static SolvePositionConstraints_s_P = new B2Vec2();
  public SolvePositionConstraints(data: B2SolverData): boolean {
    const cA: B2Vec2 = data.positions[this.m_indexA].c;
    let aA: number = data.positions[this.m_indexA].a;
    const cB: B2Vec2 = data.positions[this.m_indexB].c;
    let aB: number = data.positions[this.m_indexB].a;

    const qA: B2Rot = this.m_qA.SetAngle(aA), qB: B2Rot = this.m_qB.SetAngle(aB);

    // B2Vec2 rA = B2Mul(qA, m_localAnchorA - m_localCenterA);
    B2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
    const rA: B2Vec2 = B2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
    // B2Vec2 rB = B2Mul(qB, m_localAnchorB - m_localCenterB);
    B2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
    const rB: B2Vec2 = B2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
    // B2Vec2 d = (cB - cA) + rB - rA;
    const d: B2Vec2 = B2Vec2.AddVV(
      B2Vec2.SubVV(cB, cA, B2Vec2.s_t0),
      B2Vec2.SubVV(rB, rA, B2Vec2.s_t1),
      B2WheelJoint.SolvePositionConstraints_s_d);

    // B2Vec2 ay = B2Mul(qA, m_localYAxisA);
    const ay: B2Vec2 = B2Rot.MulRV(qA, this.m_localYAxisA, this.m_ay);

    // float32 sAy = B2Cross(d + rA, ay);
    const sAy = B2Vec2.CrossVV(B2Vec2.AddVV(d, rA, B2Vec2.s_t0), ay);
    // float32 sBy = B2Cross(rB, ay);
    const sBy = B2Vec2.CrossVV(rB, ay);

    // float32 C = B2Dot(d, ay);
    const C: number = B2Vec2.DotVV(d, this.m_ay);

    const k: number = this.m_invMassA + this.m_invMassB + this.m_invIA * this.m_sAy * this.m_sAy + this.m_invIB * this.m_sBy * this.m_sBy;

    let impulse: number;
    if (k !== 0) {
      impulse = - C / k;
    } else {
      impulse = 0;
    }

    // B2Vec2 P = impulse * ay;
    const P: B2Vec2 = B2Vec2.MulSV(impulse, ay, B2WheelJoint.SolvePositionConstraints_s_P);
    const LA: number = impulse * sAy;
    const LB: number = impulse * sBy;

    // cA -= m_invMassA * P;
    cA.SelfMulSub(this.m_invMassA, P);
    aA -= this.m_invIA * LA;
    // cB += m_invMassB * P;
    cB.SelfMulAdd(this.m_invMassB, P);
    aB += this.m_invIB * LB;

    // data.positions[this.m_indexA].c = cA;
    data.positions[this.m_indexA].a = aA;
    // data.positions[this.m_indexB].c = cB;
    data.positions[this.m_indexB].a = aB;

    return B2Abs(C) <= B2_linearSlop;
  }

  public GetDefinition(def: B2WheelJointDef): B2WheelJointDef {
    /// b2Assert(false); // TODO
    return def;
  }

  public GetAnchorA(out: B2Vec2): B2Vec2 {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
  }

  public GetAnchorB(out: B2Vec2): B2Vec2 {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
  }

  public GetReactionForce(inv_dt: number, out: B2Vec2): B2Vec2 {
    // return inv_dt * (m_impulse * m_ay + m_springImpulse * m_ax);
    out.x = inv_dt * (this.m_impulse * this.m_ay.x + this.m_springImpulse * this.m_ax.x);
    out.y = inv_dt * (this.m_impulse * this.m_ay.y + this.m_springImpulse * this.m_ax.y);
    return out;
  }

  public GetReactionTorque(inv_dt: number): number {
    return inv_dt * this.m_motorImpulse;
  }

  public GetLocalAnchorA(): B2Vec2 { return this.m_localAnchorA; }

  public GetLocalAnchorB(): B2Vec2 { return this.m_localAnchorB; }

  public GetLocalAxisA(): B2Vec2 { return this.m_localXAxisA; }

  public GetJointTranslation(): number {
    return this.GetPrismaticJointTranslation();
  }

  public GetJointSpeed(): number {
    return this.GetRevoluteJointSpeed();
  }

  public GetPrismaticJointTranslation(): number {
    const bA: B2Body = this.m_bodyA;
    const bB: B2Body = this.m_bodyB;

    const pA: B2Vec2 = bA.GetWorldPoint(this.m_localAnchorA, new B2Vec2());
    const pB: B2Vec2 = bB.GetWorldPoint(this.m_localAnchorB, new B2Vec2());
    const d: B2Vec2 = B2Vec2.SubVV(pB, pA, new B2Vec2());
    const axis: B2Vec2 = bA.GetWorldVector(this.m_localXAxisA, new B2Vec2());

    const translation: number = B2Vec2.DotVV(d, axis);
    return translation;
  }

  public GetPrismaticJointSpeed(): number {
    const bA: B2Body = this.m_bodyA;
    const bB: B2Body = this.m_bodyB;

    // B2Vec2 rA = B2Mul(bA->m_xf.q, m_localAnchorA - bA->m_sweep.localCenter);
    B2Vec2.SubVV(this.m_localAnchorA, bA.m_sweep.localCenter, this.m_lalcA);
    const rA = B2Rot.MulRV(bA.m_xf.q, this.m_lalcA, this.m_rA);
    // B2Vec2 rB = B2Mul(bB->m_xf.q, m_localAnchorB - bB->m_sweep.localCenter);
    B2Vec2.SubVV(this.m_localAnchorB, bB.m_sweep.localCenter, this.m_lalcB);
    const rB = B2Rot.MulRV(bB.m_xf.q, this.m_lalcB, this.m_rB);
    // B2Vec2 pA = bA->m_sweep.c + rA;
    const pA = B2Vec2.AddVV(bA.m_sweep.c, rA, B2Vec2.s_t0); // pA uses s_t0
    // B2Vec2 pB = bB->m_sweep.c + rB;
    const pB = B2Vec2.AddVV(bB.m_sweep.c, rB, B2Vec2.s_t1); // pB uses s_t1
    // B2Vec2 d = pB - pA;
    const d = B2Vec2.SubVV(pB, pA, B2Vec2.s_t2); // d uses s_t2
    // B2Vec2 axis = B2Mul(bA.m_xf.q, m_localXAxisA);
    const axis = bA.GetWorldVector(this.m_localXAxisA, new B2Vec2());

    const vA = bA.m_linearVelocity;
    const vB = bB.m_linearVelocity;
    const wA = bA.m_angularVelocity;
    const wB = bB.m_angularVelocity;

    // float32 speed = B2Dot(d, B2Cross(wA, axis)) + B2Dot(axis, vB + B2Cross(wB, rB) - vA - B2Cross(wA, rA));
    const speed =
      B2Vec2.DotVV(d, B2Vec2.CrossSV(wA, axis, B2Vec2.s_t0)) +
      B2Vec2.DotVV(
        axis,
        B2Vec2.SubVV(
          B2Vec2.AddVCrossSV(vB, wB, rB, B2Vec2.s_t0),
          B2Vec2.AddVCrossSV(vA, wA, rA, B2Vec2.s_t1),
          B2Vec2.s_t0));
    return speed;
  }

  public GetRevoluteJointAngle(): number {
    // B2Body* bA = this.m_bodyA;
    // B2Body* bB = this.m_bodyB;
    // return bB->this.m_sweep.a - bA->this.m_sweep.a;
    return this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a;
  }

  public GetRevoluteJointSpeed(): number {
    const wA: number = this.m_bodyA.m_angularVelocity;
    const wB: number = this.m_bodyB.m_angularVelocity;
    return wB - wA;
  }

  public IsMotorEnabled(): boolean {
    return this.m_enableMotor;
  }

  public EnableMotor(flag: boolean): void {
    this.m_bodyA.SetAwake(true);
    this.m_bodyB.SetAwake(true);
    this.m_enableMotor = flag;
  }

  public SetMotorSpeed(speed: number): void {
    this.m_bodyA.SetAwake(true);
    this.m_bodyB.SetAwake(true);
    this.m_motorSpeed = speed;
  }

  public SetMaxMotorTorque(force: number): void {
    this.m_bodyA.SetAwake(true);
    this.m_bodyB.SetAwake(true);
    this.m_maxMotorTorque = force;
  }

  public GetMotorTorque(inv_dt: number): number {
    return inv_dt * this.m_motorImpulse;
  }

  public Dump(log: (format: string, ...args: any[]) => void): void {
    const indexA = this.m_bodyA.m_islandIndex;
    const indexB = this.m_bodyB.m_islandIndex;

    log('  const jd: B2WheelJointDef = new B2WheelJointDef();\n');
    log('  jd.bodyA = bodies[%d];\n', indexA);
    log('  jd.bodyB = bodies[%d];\n', indexB);
    log('  jd.collideConnected = %s;\n', (this.m_collideConnected) ? ('true') : ('false'));
    log('  jd.localAnchorA.Set(%.15f, %.15f);\n', this.m_localAnchorA.x, this.m_localAnchorA.y);
    log('  jd.localAnchorB.Set(%.15f, %.15f);\n', this.m_localAnchorB.x, this.m_localAnchorB.y);
    log('  jd.localAxisA.Set(%.15f, %.15f);\n', this.m_localXAxisA.x, this.m_localXAxisA.y);
    log('  jd.enableMotor = %s;\n', (this.m_enableMotor) ? ('true') : ('false'));
    log('  jd.motorSpeed = %.15f;\n', this.m_motorSpeed);
    log('  jd.maxMotorTorque = %.15f;\n', this.m_maxMotorTorque);
    log('  jd.frequencyHz = %.15f;\n', this.m_frequencyHz);
    log('  jd.dampingRatio = %.15f;\n', this.m_dampingRatio);
    log('  joints[%d] = this.m_world.CreateJoint(jd);\n', this.m_index);
  }
}
