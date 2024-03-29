"use strict";
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
Object.defineProperty(exports, "__esModule", { value: true });
exports.B2PulleyJoint = exports.B2PulleyJointDef = exports.B2_minPulleyLength = void 0;
const b2Settings_1 = require("../../Common/b2Settings");
const b2Math_1 = require("../../Common/b2Math");
const b2Joint_1 = require("./b2Joint");
exports.B2_minPulleyLength = 2;
///  Pulley joint definition. This requires two ground anchors,
///  two dynamic body anchor points, and a pulley ratio.
class B2PulleyJointDef extends b2Joint_1.B2JointDef {
    constructor() {
        super(4 /* e_pulleyJoint */);
        this.groundAnchorA = new b2Math_1.B2Vec2(-1, 1);
        this.groundAnchorB = new b2Math_1.B2Vec2(1, 1);
        this.localAnchorA = new b2Math_1.B2Vec2(-1, 0);
        this.localAnchorB = new b2Math_1.B2Vec2(1, 0);
        this.lengthA = 0;
        this.lengthB = 0;
        this.ratio = 1;
        this.collideConnected = true;
    }
    Initialize(bA, bB, groundA, groundB, anchorA, anchorB, r) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.groundAnchorA.Copy(groundA);
        this.groundAnchorB.Copy(groundB);
        this.bodyA.GetLocalPoint(anchorA, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchorB, this.localAnchorB);
        this.lengthA = b2Math_1.B2Vec2.DistanceVV(anchorA, groundA);
        this.lengthB = b2Math_1.B2Vec2.DistanceVV(anchorB, groundB);
        this.ratio = r;
        /// b2Assert(this.ratio > B2_epsilon);
    }
}
exports.B2PulleyJointDef = B2PulleyJointDef;
class B2PulleyJoint extends b2Joint_1.B2Joint {
    constructor(def) {
        super(def);
        this.m_groundAnchorA = new b2Math_1.B2Vec2();
        this.m_groundAnchorB = new b2Math_1.B2Vec2();
        this.m_lengthA = 0;
        this.m_lengthB = 0;
        // Solver shared
        this.m_localAnchorA = new b2Math_1.B2Vec2();
        this.m_localAnchorB = new b2Math_1.B2Vec2();
        this.m_constant = 0;
        this.m_ratio = 0;
        this.m_impulse = 0;
        // Solver temp
        this.m_indexA = 0;
        this.m_indexB = 0;
        this.m_uA = new b2Math_1.B2Vec2();
        this.m_uB = new b2Math_1.B2Vec2();
        this.m_rA = new b2Math_1.B2Vec2();
        this.m_rB = new b2Math_1.B2Vec2();
        this.m_localCenterA = new b2Math_1.B2Vec2();
        this.m_localCenterB = new b2Math_1.B2Vec2();
        this.m_invMassA = 0;
        this.m_invMassB = 0;
        this.m_invIA = 0;
        this.m_invIB = 0;
        this.m_mass = 0;
        this.m_qA = new b2Math_1.B2Rot();
        this.m_qB = new b2Math_1.B2Rot();
        this.m_lalcA = new b2Math_1.B2Vec2();
        this.m_lalcB = new b2Math_1.B2Vec2();
        this.m_groundAnchorA.Copy(def.groundAnchorA);
        this.m_groundAnchorB.Copy(def.groundAnchorB);
        this.m_localAnchorA.Copy(def.localAnchorA);
        this.m_localAnchorB.Copy(def.localAnchorB);
        this.m_lengthA = def.lengthA;
        this.m_lengthB = def.lengthB;
        /// b2Assert(def.ratio !== 0);
        this.m_ratio = def.ratio;
        this.m_constant = def.lengthA + this.m_ratio * def.lengthB;
        this.m_impulse = 0;
    }
    InitVelocityConstraints(data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterA.Copy(this.m_bodyA.m_sweep.localCenter);
        this.m_localCenterB.Copy(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassA = this.m_bodyA.m_invMass;
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIA = this.m_bodyA.m_invI;
        this.m_invIB = this.m_bodyB.m_invI;
        const cA = data.positions[this.m_indexA].c;
        const aA = data.positions[this.m_indexA].a;
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const cB = data.positions[this.m_indexB].c;
        const aB = data.positions[this.m_indexB].a;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        // B2Rot qA(aA), qB(aB);
        const qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // m_rA = B2Mul(qA, m_localAnchorA - m_localCenterA);
        b2Math_1.B2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        b2Math_1.B2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // m_rB = B2Mul(qB, m_localAnchorB - m_localCenterB);
        b2Math_1.B2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        b2Math_1.B2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // Get the pulley axes.
        // m_uA = cA + m_rA - m_groundAnchorA;
        this.m_uA
            .Copy(cA)
            .SelfAdd(this.m_rA)
            .SelfSub(this.m_groundAnchorA);
        // m_uB = cB + m_rB - m_groundAnchorB;
        this.m_uB
            .Copy(cB)
            .SelfAdd(this.m_rB)
            .SelfSub(this.m_groundAnchorB);
        const lengthA = this.m_uA.Length();
        const lengthB = this.m_uB.Length();
        if (lengthA > 10 * b2Settings_1.B2_linearSlop) {
            this.m_uA.SelfMul(1 / lengthA);
        }
        else {
            this.m_uA.SetZero();
        }
        if (lengthB > 10 * b2Settings_1.B2_linearSlop) {
            this.m_uB.SelfMul(1 / lengthB);
        }
        else {
            this.m_uB.SetZero();
        }
        // Compute effective mass.
        const ruA = b2Math_1.B2Vec2.CrossVV(this.m_rA, this.m_uA);
        const ruB = b2Math_1.B2Vec2.CrossVV(this.m_rB, this.m_uB);
        const mA = this.m_invMassA + this.m_invIA * ruA * ruA;
        const mB = this.m_invMassB + this.m_invIB * ruB * ruB;
        this.m_mass = mA + this.m_ratio * this.m_ratio * mB;
        if (this.m_mass > 0) {
            this.m_mass = 1 / this.m_mass;
        }
        if (data.step.warmStarting) {
            // Scale impulses to support variable time steps.
            this.m_impulse *= data.step.dtRatio;
            // Warm starting.
            // B2Vec2 PA = -(m_impulse) * m_uA;
            const PA = b2Math_1.B2Vec2.MulSV(-this.m_impulse, this.m_uA, B2PulleyJoint.InitVelocityConstraints_s_PA);
            // B2Vec2 PB = (-m_ratio * m_impulse) * m_uB;
            const PB = b2Math_1.B2Vec2.MulSV(-this.m_ratio * this.m_impulse, this.m_uB, B2PulleyJoint.InitVelocityConstraints_s_PB);
            // vA += m_invMassA * PA;
            vA.SelfMulAdd(this.m_invMassA, PA);
            wA += this.m_invIA * b2Math_1.B2Vec2.CrossVV(this.m_rA, PA);
            // vB += m_invMassB * PB;
            vB.SelfMulAdd(this.m_invMassB, PB);
            wB += this.m_invIB * b2Math_1.B2Vec2.CrossVV(this.m_rB, PB);
        }
        else {
            this.m_impulse = 0;
        }
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    }
    SolveVelocityConstraints(data) {
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        // B2Vec2 vpA = vA + B2Cross(wA, m_rA);
        const vpA = b2Math_1.B2Vec2.AddVCrossSV(vA, wA, this.m_rA, B2PulleyJoint.SolveVelocityConstraints_s_vpA);
        // B2Vec2 vpB = vB + B2Cross(wB, m_rB);
        const vpB = b2Math_1.B2Vec2.AddVCrossSV(vB, wB, this.m_rB, B2PulleyJoint.SolveVelocityConstraints_s_vpB);
        const Cdot = -b2Math_1.B2Vec2.DotVV(this.m_uA, vpA) -
            this.m_ratio * b2Math_1.B2Vec2.DotVV(this.m_uB, vpB);
        const impulse = -this.m_mass * Cdot;
        this.m_impulse += impulse;
        // B2Vec2 PA = -impulse * m_uA;
        const PA = b2Math_1.B2Vec2.MulSV(-impulse, this.m_uA, B2PulleyJoint.SolveVelocityConstraints_s_PA);
        // B2Vec2 PB = -m_ratio * impulse * m_uB;
        const PB = b2Math_1.B2Vec2.MulSV(-this.m_ratio * impulse, this.m_uB, B2PulleyJoint.SolveVelocityConstraints_s_PB);
        // vA += m_invMassA * PA;
        vA.SelfMulAdd(this.m_invMassA, PA);
        wA += this.m_invIA * b2Math_1.B2Vec2.CrossVV(this.m_rA, PA);
        // vB += m_invMassB * PB;
        vB.SelfMulAdd(this.m_invMassB, PB);
        wB += this.m_invIB * b2Math_1.B2Vec2.CrossVV(this.m_rB, PB);
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    }
    SolvePositionConstraints(data) {
        const cA = data.positions[this.m_indexA].c;
        let aA = data.positions[this.m_indexA].a;
        const cB = data.positions[this.m_indexB].c;
        let aB = data.positions[this.m_indexB].a;
        // B2Rot qA(aA), qB(aB);
        const qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // B2Vec2 rA = B2Mul(qA, m_localAnchorA - m_localCenterA);
        b2Math_1.B2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        const rA = b2Math_1.B2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // B2Vec2 rB = B2Mul(qB, m_localAnchorB - m_localCenterB);
        b2Math_1.B2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        const rB = b2Math_1.B2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // Get the pulley axes.
        // B2Vec2 uA = cA + rA - m_groundAnchorA;
        const uA = this.m_uA
            .Copy(cA)
            .SelfAdd(rA)
            .SelfSub(this.m_groundAnchorA);
        // B2Vec2 uB = cB + rB - m_groundAnchorB;
        const uB = this.m_uB
            .Copy(cB)
            .SelfAdd(rB)
            .SelfSub(this.m_groundAnchorB);
        const lengthA = uA.Length();
        const lengthB = uB.Length();
        if (lengthA > 10 * b2Settings_1.B2_linearSlop) {
            uA.SelfMul(1 / lengthA);
        }
        else {
            uA.SetZero();
        }
        if (lengthB > 10 * b2Settings_1.B2_linearSlop) {
            uB.SelfMul(1 / lengthB);
        }
        else {
            uB.SetZero();
        }
        // Compute effective mass.
        const ruA = b2Math_1.B2Vec2.CrossVV(rA, uA);
        const ruB = b2Math_1.B2Vec2.CrossVV(rB, uB);
        const mA = this.m_invMassA + this.m_invIA * ruA * ruA;
        const mB = this.m_invMassB + this.m_invIB * ruB * ruB;
        let mass = mA + this.m_ratio * this.m_ratio * mB;
        if (mass > 0) {
            mass = 1 / mass;
        }
        const C = this.m_constant - lengthA - this.m_ratio * lengthB;
        const linearError = b2Math_1.B2Abs(C);
        const impulse = -mass * C;
        // B2Vec2 PA = -impulse * uA;
        const PA = b2Math_1.B2Vec2.MulSV(-impulse, uA, B2PulleyJoint.SolvePositionConstraints_s_PA);
        // B2Vec2 PB = -m_ratio * impulse * uB;
        const PB = b2Math_1.B2Vec2.MulSV(-this.m_ratio * impulse, uB, B2PulleyJoint.SolvePositionConstraints_s_PB);
        // cA += m_invMassA * PA;
        cA.SelfMulAdd(this.m_invMassA, PA);
        aA += this.m_invIA * b2Math_1.B2Vec2.CrossVV(rA, PA);
        // cB += m_invMassB * PB;
        cB.SelfMulAdd(this.m_invMassB, PB);
        aB += this.m_invIB * b2Math_1.B2Vec2.CrossVV(rB, PB);
        // data.positions[this.m_indexA].c = cA;
        data.positions[this.m_indexA].a = aA;
        // data.positions[this.m_indexB].c = cB;
        data.positions[this.m_indexB].a = aB;
        return linearError < b2Settings_1.B2_linearSlop;
    }
    GetAnchorA(out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }
    GetAnchorB(out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }
    GetReactionForce(inv_dt, out) {
        // B2Vec2 P = m_impulse * m_uB;
        // return inv_dt * P;
        return out.Set(inv_dt * this.m_impulse * this.m_uB.x, inv_dt * this.m_impulse * this.m_uB.y);
    }
    GetReactionTorque(inv_dt) {
        return 0;
    }
    GetGroundAnchorA() {
        return this.m_groundAnchorA;
    }
    GetGroundAnchorB() {
        return this.m_groundAnchorB;
    }
    GetLengthA() {
        return this.m_lengthA;
    }
    GetLengthB() {
        return this.m_lengthB;
    }
    GetRatio() {
        return this.m_ratio;
    }
    GetCurrentLengthA() {
        // B2Vec2 p = m_bodyA->GetWorldPoint(m_localAnchorA);
        // B2Vec2 s = m_groundAnchorA;
        // B2Vec2 d = p - s;
        // return d.Length();
        const p = this.m_bodyA.GetWorldPoint(this.m_localAnchorA, B2PulleyJoint.GetCurrentLengthA_s_p);
        const s = this.m_groundAnchorA;
        return b2Math_1.B2Vec2.DistanceVV(p, s);
    }
    GetCurrentLengthB() {
        // B2Vec2 p = m_bodyB->GetWorldPoint(m_localAnchorB);
        // B2Vec2 s = m_groundAnchorB;
        // B2Vec2 d = p - s;
        // return d.Length();
        const p = this.m_bodyB.GetWorldPoint(this.m_localAnchorB, B2PulleyJoint.GetCurrentLengthB_s_p);
        const s = this.m_groundAnchorB;
        return b2Math_1.B2Vec2.DistanceVV(p, s);
    }
    Dump(log) {
        const indexA = this.m_bodyA.m_islandIndex;
        const indexB = this.m_bodyB.m_islandIndex;
        log('  const jd: B2PulleyJointDef = new B2PulleyJointDef();\n');
        log('  jd.bodyA = bodies[%d];\n', indexA);
        log('  jd.bodyB = bodies[%d];\n', indexB);
        log('  jd.collideConnected = %s;\n', this.m_collideConnected ? 'true' : 'false');
        log('  jd.groundAnchorA.Set(%.15f, %.15f);\n', this.m_groundAnchorA.x, this.m_groundAnchorA.y);
        log('  jd.groundAnchorB.Set(%.15f, %.15f);\n', this.m_groundAnchorB.x, this.m_groundAnchorB.y);
        log('  jd.localAnchorA.Set(%.15f, %.15f);\n', this.m_localAnchorA.x, this.m_localAnchorA.y);
        log('  jd.localAnchorB.Set(%.15f, %.15f);\n', this.m_localAnchorB.x, this.m_localAnchorB.y);
        log('  jd.lengthA = %.15f;\n', this.m_lengthA);
        log('  jd.lengthB = %.15f;\n', this.m_lengthB);
        log('  jd.ratio = %.15f;\n', this.m_ratio);
        log('  joints[%d] = this.m_world.CreateJoint(jd);\n', this.m_index);
    }
    ShiftOrigin(newOrigin) {
        this.m_groundAnchorA.SelfSub(newOrigin);
        this.m_groundAnchorB.SelfSub(newOrigin);
    }
}
exports.B2PulleyJoint = B2PulleyJoint;
B2PulleyJoint.InitVelocityConstraints_s_PA = new b2Math_1.B2Vec2();
B2PulleyJoint.InitVelocityConstraints_s_PB = new b2Math_1.B2Vec2();
B2PulleyJoint.SolveVelocityConstraints_s_vpA = new b2Math_1.B2Vec2();
B2PulleyJoint.SolveVelocityConstraints_s_vpB = new b2Math_1.B2Vec2();
B2PulleyJoint.SolveVelocityConstraints_s_PA = new b2Math_1.B2Vec2();
B2PulleyJoint.SolveVelocityConstraints_s_PB = new b2Math_1.B2Vec2();
B2PulleyJoint.SolvePositionConstraints_s_PA = new b2Math_1.B2Vec2();
B2PulleyJoint.SolvePositionConstraints_s_PB = new b2Math_1.B2Vec2();
B2PulleyJoint.GetCurrentLengthA_s_p = new b2Math_1.B2Vec2();
B2PulleyJoint.GetCurrentLengthB_s_p = new b2Math_1.B2Vec2();
