/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

import { B2_pi } from '../../Common/b2Settings';
import { B2Vec2, B2Mat22, B2Rot, B2Transform } from '../../Common/b2Math';
import { B2Joint, B2JointDef, B2JointType } from './b2Joint';
import { B2SolverData } from '../b2TimeStep';

///  Mouse joint definition. This requires a world target point,
///  tuning parameters, and the time step.
export class B2MouseJointDef extends B2JointDef {
  public target: B2Vec2 = new B2Vec2();

  public maxForce: number = 0;

  public frequencyHz: number = 5;

  public dampingRatio: number = 0.7;

  constructor() {
    super(B2JointType.e_mouseJoint);
  }
}

export class B2MouseJoint extends B2Joint {
  public m_localAnchorB: B2Vec2 = null;
  public m_targetA: B2Vec2 = null;
  public m_frequencyHz: number = 0;
  public m_dampingRatio: number = 0;
  public m_beta: number = 0;

  // Solver shared
  public m_impulse: B2Vec2 = null;
  public m_maxForce: number = 0;
  public m_gamma: number = 0;

  // Solver temp
  public m_indexA: number = 0;
  public m_indexB: number = 0;
  public m_rB: B2Vec2 = null;
  public m_localCenterB: B2Vec2 = null;
  public m_invMassB: number = 0;
  public m_invIB: number = 0;
  public m_mass: B2Mat22 = null;
  public m_C: B2Vec2 = null;
  public m_qB: B2Rot = null;
  public m_lalcB: B2Vec2 = null;
  public m_K: B2Mat22 = null;

  constructor(def: B2MouseJointDef) {
    super(def);

    this.m_localAnchorB = new B2Vec2();
    this.m_targetA = new B2Vec2();

    this.m_impulse = new B2Vec2();

    this.m_rB = new B2Vec2();
    this.m_localCenterB = new B2Vec2();
    this.m_mass = new B2Mat22();
    this.m_C = new B2Vec2();
    this.m_qB = new B2Rot();
    this.m_lalcB = new B2Vec2();
    this.m_K = new B2Mat22();

    /// b2Assert(def.target.IsValid());
    /// b2Assert(B2IsValid(def.maxForce) && def.maxForce >= 0);
    /// b2Assert(B2IsValid(def.frequencyHz) && def.frequencyHz >= 0);
    /// b2Assert(B2IsValid(def.dampingRatio) && def.dampingRatio >= 0);

    this.m_targetA.Copy(def.target);
    B2Transform.MulTXV(this.m_bodyB.GetTransform(), this.m_targetA, this.m_localAnchorB);

    this.m_maxForce = def.maxForce;
    this.m_impulse.SetZero();

    this.m_frequencyHz = def.frequencyHz;
    this.m_dampingRatio = def.dampingRatio;

    this.m_beta = 0;
    this.m_gamma = 0;
  }

  public SetTarget(target: B2Vec2): void {
    if (!this.m_bodyB.IsAwake()) {
      this.m_bodyB.SetAwake(true);
    }
    this.m_targetA.Copy(target);
  }

  public GetTarget() {
    return this.m_targetA;
  }

  public SetMaxForce(maxForce: number): void {
    this.m_maxForce = maxForce;
  }

  public GetMaxForce() {
    return this.m_maxForce;
  }

  public SetFrequency(hz: number): void {
    this.m_frequencyHz = hz;
  }

  public GetFrequency() {
    return this.m_frequencyHz;
  }

  public SetDampingRatio(ratio: number) {
    this.m_dampingRatio = ratio;
  }

  public GetDampingRatio() {
    return this.m_dampingRatio;
  }

  public InitVelocityConstraints(data: B2SolverData): void {
    this.m_indexB = this.m_bodyB.m_islandIndex;
    this.m_localCenterB.Copy(this.m_bodyB.m_sweep.localCenter);
    this.m_invMassB = this.m_bodyB.m_invMass;
    this.m_invIB = this.m_bodyB.m_invI;

    const cB: B2Vec2 = data.positions[this.m_indexB].c;
    const aB: number = data.positions[this.m_indexB].a;
    const vB: B2Vec2 = data.velocities[this.m_indexB].v;
    let wB: number = data.velocities[this.m_indexB].w;

    const qB = this.m_qB.SetAngle(aB);

    const mass: number = this.m_bodyB.GetMass();

    // Frequency
    const omega: number = 2 * B2_pi * this.m_frequencyHz;

    // Damping coefficient
    const d: number = 2 * mass * this.m_dampingRatio * omega;

    // Spring stiffness
    const k: number = mass * (omega * omega);

    // magic formulas
    // gamma has units of inverse mass.
    // beta has units of inverse time.
    const h: number = data.step.dt;
    /// b2Assert(d + h * k > B2_epsilon);
    this.m_gamma = h * (d + h * k);
    if (this.m_gamma !== 0) {
      this.m_gamma = 1 / this.m_gamma;
    }
    this.m_beta = h * k * this.m_gamma;

    // Compute the effective mass matrix.
    B2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
    B2Rot.MulRV(qB, this.m_lalcB, this.m_rB);

    // K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
    //      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
    //        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
    const K = this.m_K;
    K.ex.x = this.m_invMassB + this.m_invIB * this.m_rB.y * this.m_rB.y + this.m_gamma;
    K.ex.y = -this.m_invIB * this.m_rB.x * this.m_rB.y;
    K.ey.x = K.ex.y;
    K.ey.y = this.m_invMassB + this.m_invIB * this.m_rB.x * this.m_rB.x + this.m_gamma;

    K.GetInverse(this.m_mass);

    // m_C = cB + m_rB - m_targetA;
    this.m_C.x = cB.x + this.m_rB.x - this.m_targetA.x;
    this.m_C.y = cB.y + this.m_rB.y - this.m_targetA.y;
    // m_C *= m_beta;
    this.m_C.SelfMul(this.m_beta);

    // Cheat with some damping
    wB *= 0.98;

    if (data.step.warmStarting) {
      this.m_impulse.SelfMul(data.step.dtRatio);
      // vB += m_invMassB * m_impulse;
      vB.x += this.m_invMassB * this.m_impulse.x;
      vB.y += this.m_invMassB * this.m_impulse.y;
      wB += this.m_invIB * B2Vec2.CrossVV(this.m_rB, this.m_impulse);
    } else {
      this.m_impulse.SetZero();
    }

    // data.velocities[this.m_indexB].v = vB;
    data.velocities[this.m_indexB].w = wB;
  }

  private static SolveVelocityConstraints_s_Cdot = new B2Vec2();
  private static SolveVelocityConstraints_s_impulse = new B2Vec2();
  private static SolveVelocityConstraints_s_oldImpulse = new B2Vec2();
  public SolveVelocityConstraints(data: B2SolverData): void {
    const vB: B2Vec2 = data.velocities[this.m_indexB].v;
    let wB: number = data.velocities[this.m_indexB].w;

    // Cdot = v + cross(w, r)
    // B2Vec2 Cdot = vB + B2Cross(wB, m_rB);
    const Cdot: B2Vec2 = B2Vec2.AddVCrossSV(vB, wB, this.m_rB, B2MouseJoint.SolveVelocityConstraints_s_Cdot);
    //  B2Vec2 impulse = B2Mul(m_mass, -(Cdot + m_C + m_gamma * m_impulse));
    const impulse: B2Vec2 = B2Mat22.MulMV(
      this.m_mass,
      B2Vec2.AddVV(
        Cdot,
        B2Vec2.AddVV(this.m_C,
          B2Vec2.MulSV(this.m_gamma, this.m_impulse, B2Vec2.s_t0),
          B2Vec2.s_t0),
        B2Vec2.s_t0).SelfNeg(),
      B2MouseJoint.SolveVelocityConstraints_s_impulse);

    // B2Vec2 oldImpulse = m_impulse;
    const oldImpulse = B2MouseJoint.SolveVelocityConstraints_s_oldImpulse.Copy(this.m_impulse);
    // m_impulse += impulse;
    this.m_impulse.SelfAdd(impulse);
    const maxImpulse: number = data.step.dt * this.m_maxForce;
    if (this.m_impulse.LengthSquared() > maxImpulse * maxImpulse) {
      this.m_impulse.SelfMul(maxImpulse / this.m_impulse.Length());
    }
    // impulse = m_impulse - oldImpulse;
    B2Vec2.SubVV(this.m_impulse, oldImpulse, impulse);

    // vB += m_invMassB * impulse;
    vB.SelfMulAdd(this.m_invMassB, impulse);
    wB += this.m_invIB * B2Vec2.CrossVV(this.m_rB, impulse);

    // data.velocities[this.m_indexB].v = vB;
    data.velocities[this.m_indexB].w = wB;
  }

  public SolvePositionConstraints(data: B2SolverData): boolean {
    return true;
  }

  public GetAnchorA(out: B2Vec2): B2Vec2 {
    return out.Copy(this.m_targetA);
  }

  public GetAnchorB(out: B2Vec2): B2Vec2 {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
  }

  public GetReactionForce(inv_dt: number, out: B2Vec2): B2Vec2 {
    return B2Vec2.MulSV(inv_dt, this.m_impulse, out);
  }

  public GetReactionTorque(inv_dt: number): number {
    return 0;
  }

  public Dump(log: (format: string, ...args: any[]) => void) {
    log('Mouse joint dumping is not supported.\n');
  }

  public ShiftOrigin(newOrigin: B2Vec2) {
    this.m_targetA.SelfSub(newOrigin);
  }
}
