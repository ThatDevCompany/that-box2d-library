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
exports.B2RopeJoint = exports.B2RopeJointDef = void 0;
const b2Settings_1 = require("../../Common/b2Settings");
const b2Math_1 = require("../../Common/b2Math");
const b2Joint_1 = require("./b2Joint");
///  Rope joint definition. This requires two body anchor points and
///  a maximum lengths.
///  Note: by default the connected objects will not collide.
///  see collideConnected in B2JointDef.
class B2RopeJointDef extends b2Joint_1.B2JointDef {
    constructor() {
        super(10 /* e_ropeJoint */);
        this.localAnchorA = new b2Math_1.B2Vec2(-1, 0);
        this.localAnchorB = new b2Math_1.B2Vec2(1, 0);
        this.maxLength = 0;
    }
}
exports.B2RopeJointDef = B2RopeJointDef;
class B2RopeJoint extends b2Joint_1.B2Joint {
    constructor(def) {
        super(def);
        // Solver shared
        this.m_localAnchorA = new b2Math_1.B2Vec2();
        this.m_localAnchorB = new b2Math_1.B2Vec2();
        this.m_maxLength = 0;
        this.m_length = 0;
        this.m_impulse = 0;
        // Solver temp
        this.m_indexA = 0;
        this.m_indexB = 0;
        this.m_u = new b2Math_1.B2Vec2();
        this.m_rA = new b2Math_1.B2Vec2();
        this.m_rB = new b2Math_1.B2Vec2();
        this.m_localCenterA = new b2Math_1.B2Vec2();
        this.m_localCenterB = new b2Math_1.B2Vec2();
        this.m_invMassA = 0;
        this.m_invMassB = 0;
        this.m_invIA = 0;
        this.m_invIB = 0;
        this.m_mass = 0;
        this.m_state = 0 /* e_inactiveLimit */;
        this.m_qA = new b2Math_1.B2Rot();
        this.m_qB = new b2Math_1.B2Rot();
        this.m_lalcA = new b2Math_1.B2Vec2();
        this.m_lalcB = new b2Math_1.B2Vec2();
        this.m_localAnchorA.Copy(def.localAnchorA);
        this.m_localAnchorB.Copy(def.localAnchorB);
        this.m_maxLength = def.maxLength;
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
        const qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // this.m_rA = B2Mul(qA, this.m_localAnchorA - this.m_localCenterA);
        b2Math_1.B2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        b2Math_1.B2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // this.m_rB = B2Mul(qB, this.m_localAnchorB - this.m_localCenterB);
        b2Math_1.B2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        b2Math_1.B2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // this.m_u = cB + this.m_rB - cA - this.m_rA;
        this.m_u
            .Copy(cB)
            .SelfAdd(this.m_rB)
            .SelfSub(cA)
            .SelfSub(this.m_rA);
        this.m_length = this.m_u.Length();
        const C = this.m_length - this.m_maxLength;
        if (C > 0) {
            this.m_state = 2 /* e_atUpperLimit */;
        }
        else {
            this.m_state = 0 /* e_inactiveLimit */;
        }
        if (this.m_length > b2Settings_1.B2_linearSlop) {
            this.m_u.SelfMul(1 / this.m_length);
        }
        else {
            this.m_u.SetZero();
            this.m_mass = 0;
            this.m_impulse = 0;
            return;
        }
        // Compute effective mass.
        const crA = b2Math_1.B2Vec2.CrossVV(this.m_rA, this.m_u);
        const crB = b2Math_1.B2Vec2.CrossVV(this.m_rB, this.m_u);
        const invMass = this.m_invMassA +
            this.m_invIA * crA * crA +
            this.m_invMassB +
            this.m_invIB * crB * crB;
        this.m_mass = invMass !== 0 ? 1 / invMass : 0;
        if (data.step.warmStarting) {
            // Scale the impulse to support a variable time step.
            this.m_impulse *= data.step.dtRatio;
            // B2Vec2 P = m_impulse * m_u;
            const P = b2Math_1.B2Vec2.MulSV(this.m_impulse, this.m_u, B2RopeJoint.InitVelocityConstraints_s_P);
            // vA -= m_invMassA * P;
            vA.SelfMulSub(this.m_invMassA, P);
            wA -= this.m_invIA * b2Math_1.B2Vec2.CrossVV(this.m_rA, P);
            // vB += m_invMassB * P;
            vB.SelfMulAdd(this.m_invMassB, P);
            wB += this.m_invIB * b2Math_1.B2Vec2.CrossVV(this.m_rB, P);
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
        // Cdot = dot(u, v + cross(w, r))
        // B2Vec2 vpA = vA + B2Cross(wA, m_rA);
        const vpA = b2Math_1.B2Vec2.AddVCrossSV(vA, wA, this.m_rA, B2RopeJoint.SolveVelocityConstraints_s_vpA);
        // B2Vec2 vpB = vB + B2Cross(wB, m_rB);
        const vpB = b2Math_1.B2Vec2.AddVCrossSV(vB, wB, this.m_rB, B2RopeJoint.SolveVelocityConstraints_s_vpB);
        // float32 C = m_length - m_maxLength;
        const C = this.m_length - this.m_maxLength;
        // float32 Cdot = B2Dot(m_u, vpB - vpA);
        let Cdot = b2Math_1.B2Vec2.DotVV(this.m_u, b2Math_1.B2Vec2.SubVV(vpB, vpA, b2Math_1.B2Vec2.s_t0));
        // Predictive constraint.
        if (C < 0) {
            Cdot += data.step.inv_dt * C;
        }
        let impulse = -this.m_mass * Cdot;
        const oldImpulse = this.m_impulse;
        this.m_impulse = b2Math_1.B2Min(0, this.m_impulse + impulse);
        impulse = this.m_impulse - oldImpulse;
        // B2Vec2 P = impulse * m_u;
        const P = b2Math_1.B2Vec2.MulSV(impulse, this.m_u, B2RopeJoint.SolveVelocityConstraints_s_P);
        // vA -= m_invMassA * P;
        vA.SelfMulSub(this.m_invMassA, P);
        wA -= this.m_invIA * b2Math_1.B2Vec2.CrossVV(this.m_rA, P);
        // vB += m_invMassB * P;
        vB.SelfMulAdd(this.m_invMassB, P);
        wB += this.m_invIB * b2Math_1.B2Vec2.CrossVV(this.m_rB, P);
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
        const qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // B2Vec2 rA = B2Mul(qA, this.m_localAnchorA - this.m_localCenterA);
        b2Math_1.B2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        const rA = b2Math_1.B2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // B2Vec2 rB = B2Mul(qB, this.m_localAnchorB - this.m_localCenterB);
        b2Math_1.B2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        const rB = b2Math_1.B2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // B2Vec2 u = cB + rB - cA - rA;
        const u = this.m_u
            .Copy(cB)
            .SelfAdd(rB)
            .SelfSub(cA)
            .SelfSub(rA);
        const length = u.Normalize();
        let C = length - this.m_maxLength;
        C = b2Math_1.B2Clamp(C, 0, b2Settings_1.B2_maxLinearCorrection);
        const impulse = -this.m_mass * C;
        // B2Vec2 P = impulse * u;
        const P = b2Math_1.B2Vec2.MulSV(impulse, u, B2RopeJoint.SolvePositionConstraints_s_P);
        // cA -= m_invMassA * P;
        cA.SelfMulSub(this.m_invMassA, P);
        aA -= this.m_invIA * b2Math_1.B2Vec2.CrossVV(rA, P);
        // cB += m_invMassB * P;
        cB.SelfMulAdd(this.m_invMassB, P);
        aB += this.m_invIB * b2Math_1.B2Vec2.CrossVV(rB, P);
        // data.positions[this.m_indexA].c = cA;
        data.positions[this.m_indexA].a = aA;
        // data.positions[this.m_indexB].c = cB;
        data.positions[this.m_indexB].a = aB;
        return length - this.m_maxLength < b2Settings_1.B2_linearSlop;
    }
    GetAnchorA(out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }
    GetAnchorB(out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }
    GetReactionForce(inv_dt, out) {
        const F = b2Math_1.B2Vec2.MulSV(inv_dt * this.m_impulse, this.m_u, out);
        return F;
        // return out.Set(inv_dt * this.m_linearImpulse.x, inv_dt * this.m_linearImpulse.y);
    }
    GetReactionTorque(inv_dt) {
        return 0;
    }
    GetLocalAnchorA() {
        return this.m_localAnchorA;
    }
    GetLocalAnchorB() {
        return this.m_localAnchorB;
    }
    SetMaxLength(length) {
        this.m_maxLength = length;
    }
    GetMaxLength() {
        return this.m_maxLength;
    }
    GetLimitState() {
        return this.m_state;
    }
    Dump(log) {
        const indexA = this.m_bodyA.m_islandIndex;
        const indexB = this.m_bodyB.m_islandIndex;
        log('  const jd: B2RopeJointDef = new B2RopeJointDef();\n');
        log('  jd.bodyA = bodies[%d];\n', indexA);
        log('  jd.bodyB = bodies[%d];\n', indexB);
        log('  jd.collideConnected = %s;\n', this.m_collideConnected ? 'true' : 'false');
        log('  jd.localAnchorA.Set(%.15f, %.15f);\n', this.m_localAnchorA.x, this.m_localAnchorA.y);
        log('  jd.localAnchorB.Set(%.15f, %.15f);\n', this.m_localAnchorB.x, this.m_localAnchorB.y);
        log('  jd.maxLength = %.15f;\n', this.m_maxLength);
        log('  joints[%d] = this.m_world.CreateJoint(jd);\n', this.m_index);
    }
}
exports.B2RopeJoint = B2RopeJoint;
B2RopeJoint.InitVelocityConstraints_s_P = new b2Math_1.B2Vec2();
B2RopeJoint.SolveVelocityConstraints_s_vpA = new b2Math_1.B2Vec2();
B2RopeJoint.SolveVelocityConstraints_s_vpB = new b2Math_1.B2Vec2();
B2RopeJoint.SolveVelocityConstraints_s_P = new b2Math_1.B2Vec2();
B2RopeJoint.SolvePositionConstraints_s_P = new b2Math_1.B2Vec2();
