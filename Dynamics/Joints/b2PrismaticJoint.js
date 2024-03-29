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
exports.B2PrismaticJoint = exports.B2PrismaticJointDef = void 0;
const b2Settings_1 = require("../../Common/b2Settings");
const b2Math_1 = require("../../Common/b2Math");
const b2Joint_1 = require("./b2Joint");
///  Prismatic joint definition. This requires defining a line of
///  motion using an axis and an anchor point. The definition uses local
///  anchor points and a local axis so that the initial configuration
///  can violate the constraint slightly. The joint translation is zero
///  when the local anchor points coincide in world space. Using local
///  anchors and a local axis helps when saving and loading a game.
class B2PrismaticJointDef extends b2Joint_1.B2JointDef {
    constructor() {
        super(2 /* e_prismaticJoint */);
        this.localAnchorA = null;
        this.localAnchorB = null;
        this.localAxisA = null;
        this.referenceAngle = 0;
        this.enableLimit = false;
        this.lowerTranslation = 0;
        this.upperTranslation = 0;
        this.enableMotor = false;
        this.maxMotorForce = 0;
        this.motorSpeed = 0;
        this.localAnchorA = new b2Math_1.B2Vec2();
        this.localAnchorB = new b2Math_1.B2Vec2();
        this.localAxisA = new b2Math_1.B2Vec2(1, 0);
    }
    Initialize(bA, bB, anchor, axis) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.GetLocalPoint(anchor, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchor, this.localAnchorB);
        this.bodyA.GetLocalVector(axis, this.localAxisA);
        this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
    }
}
exports.B2PrismaticJointDef = B2PrismaticJointDef;
class B2PrismaticJoint extends b2Joint_1.B2Joint {
    constructor(def) {
        super(def);
        // Solver shared
        this.m_localAnchorA = new b2Math_1.B2Vec2();
        this.m_localAnchorB = new b2Math_1.B2Vec2();
        this.m_localXAxisA = new b2Math_1.B2Vec2();
        this.m_localYAxisA = new b2Math_1.B2Vec2();
        this.m_referenceAngle = 0;
        this.m_impulse = new b2Math_1.B2Vec3(0, 0, 0);
        this.m_motorImpulse = 0;
        this.m_lowerTranslation = 0;
        this.m_upperTranslation = 0;
        this.m_maxMotorForce = 0;
        this.m_motorSpeed = 0;
        this.m_enableLimit = false;
        this.m_enableMotor = false;
        this.m_limitState = 0 /* e_inactiveLimit */;
        // Solver temp
        this.m_indexA = 0;
        this.m_indexB = 0;
        this.m_localCenterA = new b2Math_1.B2Vec2();
        this.m_localCenterB = new b2Math_1.B2Vec2();
        this.m_invMassA = 0;
        this.m_invMassB = 0;
        this.m_invIA = 0;
        this.m_invIB = 0;
        this.m_axis = new b2Math_1.B2Vec2(0, 0);
        this.m_perp = new b2Math_1.B2Vec2(0, 0);
        this.m_s1 = 0;
        this.m_s2 = 0;
        this.m_a1 = 0;
        this.m_a2 = 0;
        this.m_K = new b2Math_1.B2Mat33();
        this.m_K3 = new b2Math_1.B2Mat33();
        this.m_K2 = new b2Math_1.B2Mat22();
        this.m_motorMass = 0;
        this.m_qA = new b2Math_1.B2Rot();
        this.m_qB = new b2Math_1.B2Rot();
        this.m_lalcA = new b2Math_1.B2Vec2();
        this.m_lalcB = new b2Math_1.B2Vec2();
        this.m_rA = new b2Math_1.B2Vec2();
        this.m_rB = new b2Math_1.B2Vec2();
        this.m_localAnchorA.Copy(def.localAnchorA);
        this.m_localAnchorB.Copy(def.localAnchorB);
        this.m_localXAxisA.Copy(def.localAxisA).SelfNormalize();
        b2Math_1.B2Vec2.CrossOneV(this.m_localXAxisA, this.m_localYAxisA);
        this.m_referenceAngle = def.referenceAngle;
        this.m_lowerTranslation = def.lowerTranslation;
        this.m_upperTranslation = def.upperTranslation;
        this.m_maxMotorForce = def.maxMotorForce;
        this.m_motorSpeed = def.motorSpeed;
        this.m_enableLimit = def.enableLimit;
        this.m_enableMotor = def.enableMotor;
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
        // Compute the effective masses.
        // B2Vec2 rA = B2Mul(qA, m_localAnchorA - m_localCenterA);
        b2Math_1.B2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        const rA = b2Math_1.B2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // B2Vec2 rB = B2Mul(qB, m_localAnchorB - m_localCenterB);
        b2Math_1.B2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        const rB = b2Math_1.B2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // B2Vec2 d = (cB - cA) + rB - rA;
        const d = b2Math_1.B2Vec2.AddVV(b2Math_1.B2Vec2.SubVV(cB, cA, b2Math_1.B2Vec2.s_t0), b2Math_1.B2Vec2.SubVV(rB, rA, b2Math_1.B2Vec2.s_t1), B2PrismaticJoint.InitVelocityConstraints_s_d);
        const mA = this.m_invMassA, mB = this.m_invMassB;
        const iA = this.m_invIA, iB = this.m_invIB;
        // Compute motor Jacobian and effective mass.
        {
            // m_axis = B2Mul(qA, m_localXAxisA);
            b2Math_1.B2Rot.MulRV(qA, this.m_localXAxisA, this.m_axis);
            // m_a1 = B2Cross(d + rA, m_axis);
            this.m_a1 = b2Math_1.B2Vec2.CrossVV(b2Math_1.B2Vec2.AddVV(d, rA, b2Math_1.B2Vec2.s_t0), this.m_axis);
            // m_a2 = B2Cross(rB, m_axis);
            this.m_a2 = b2Math_1.B2Vec2.CrossVV(rB, this.m_axis);
            this.m_motorMass =
                mA + mB + iA * this.m_a1 * this.m_a1 + iB * this.m_a2 * this.m_a2;
            if (this.m_motorMass > 0) {
                this.m_motorMass = 1 / this.m_motorMass;
            }
        }
        // Prismatic constraint.
        {
            // m_perp = B2Mul(qA, m_localYAxisA);
            b2Math_1.B2Rot.MulRV(qA, this.m_localYAxisA, this.m_perp);
            // m_s1 = B2Cross(d + rA, m_perp);
            this.m_s1 = b2Math_1.B2Vec2.CrossVV(b2Math_1.B2Vec2.AddVV(d, rA, b2Math_1.B2Vec2.s_t0), this.m_perp);
            // m_s2 = B2Cross(rB, m_perp);
            this.m_s2 = b2Math_1.B2Vec2.CrossVV(rB, this.m_perp);
            // float32 k11 = mA + mB + iA * m_s1 * m_s1 + iB * m_s2 * m_s2;
            this.m_K.ex.x =
                mA + mB + iA * this.m_s1 * this.m_s1 + iB * this.m_s2 * this.m_s2;
            // float32 k12 = iA * m_s1 + iB * m_s2;
            this.m_K.ex.y = iA * this.m_s1 + iB * this.m_s2;
            // float32 k13 = iA * m_s1 * m_a1 + iB * m_s2 * m_a2;
            this.m_K.ex.z = iA * this.m_s1 * this.m_a1 + iB * this.m_s2 * this.m_a2;
            this.m_K.ey.x = this.m_K.ex.y;
            // float32 k22 = iA + iB;
            this.m_K.ey.y = iA + iB;
            if (this.m_K.ey.y === 0) {
                // For bodies with fixed rotation.
                this.m_K.ey.y = 1;
            }
            // float32 k23 = iA * m_a1 + iB * m_a2;
            this.m_K.ey.z = iA * this.m_a1 + iB * this.m_a2;
            this.m_K.ez.x = this.m_K.ex.z;
            this.m_K.ez.y = this.m_K.ey.z;
            // float32 k33 = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;
            this.m_K.ez.z =
                mA + mB + iA * this.m_a1 * this.m_a1 + iB * this.m_a2 * this.m_a2;
            // m_K.ex.Set(k11, k12, k13);
            // m_K.ey.Set(k12, k22, k23);
            // m_K.ez.Set(k13, k23, k33);
        }
        // Compute motor and limit terms.
        if (this.m_enableLimit) {
            // float32 jointTranslation = B2Dot(m_axis, d);
            const jointTranslation = b2Math_1.B2Vec2.DotVV(this.m_axis, d);
            if (b2Math_1.B2Abs(this.m_upperTranslation - this.m_lowerTranslation) <
                2 * b2Settings_1.B2_linearSlop) {
                this.m_limitState = 3 /* e_equalLimits */;
            }
            else if (jointTranslation <= this.m_lowerTranslation) {
                if (this.m_limitState !== 1 /* e_atLowerLimit */) {
                    this.m_limitState = 1 /* e_atLowerLimit */;
                    this.m_impulse.z = 0;
                }
            }
            else if (jointTranslation >= this.m_upperTranslation) {
                if (this.m_limitState !== 2 /* e_atUpperLimit */) {
                    this.m_limitState = 2 /* e_atUpperLimit */;
                    this.m_impulse.z = 0;
                }
            }
            else {
                this.m_limitState = 0 /* e_inactiveLimit */;
                this.m_impulse.z = 0;
            }
        }
        else {
            this.m_limitState = 0 /* e_inactiveLimit */;
            this.m_impulse.z = 0;
        }
        if (!this.m_enableMotor) {
            this.m_motorImpulse = 0;
        }
        if (data.step.warmStarting) {
            // Account for variable time step.
            // m_impulse *= data.step.dtRatio;
            this.m_impulse.SelfMul(data.step.dtRatio);
            this.m_motorImpulse *= data.step.dtRatio;
            // B2Vec2 P = m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis;
            const P = b2Math_1.B2Vec2.AddVV(b2Math_1.B2Vec2.MulSV(this.m_impulse.x, this.m_perp, b2Math_1.B2Vec2.s_t0), b2Math_1.B2Vec2.MulSV(this.m_motorImpulse + this.m_impulse.z, this.m_axis, b2Math_1.B2Vec2.s_t1), B2PrismaticJoint.InitVelocityConstraints_s_P);
            // float32 LA = m_impulse.x * m_s1 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a1;
            const LA = this.m_impulse.x * this.m_s1 +
                this.m_impulse.y +
                (this.m_motorImpulse + this.m_impulse.z) * this.m_a1;
            // float32 LB = m_impulse.x * m_s2 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a2;
            const LB = this.m_impulse.x * this.m_s2 +
                this.m_impulse.y +
                (this.m_motorImpulse + this.m_impulse.z) * this.m_a2;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * LA;
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * LB;
        }
        else {
            this.m_impulse.SetZero();
            this.m_motorImpulse = 0;
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
        const mA = this.m_invMassA, mB = this.m_invMassB;
        const iA = this.m_invIA, iB = this.m_invIB;
        // Solve linear motor constraint.
        if (this.m_enableMotor &&
            this.m_limitState !== 3 /* e_equalLimits */) {
            // float32 Cdot = B2Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
            const Cdot = b2Math_1.B2Vec2.DotVV(this.m_axis, b2Math_1.B2Vec2.SubVV(vB, vA, b2Math_1.B2Vec2.s_t0)) +
                this.m_a2 * wB -
                this.m_a1 * wA;
            let impulse = this.m_motorMass * (this.m_motorSpeed - Cdot);
            const oldImpulse = this.m_motorImpulse;
            const maxImpulse = data.step.dt * this.m_maxMotorForce;
            this.m_motorImpulse = b2Math_1.B2Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_motorImpulse - oldImpulse;
            // B2Vec2 P = impulse * m_axis;
            const P = b2Math_1.B2Vec2.MulSV(impulse, this.m_axis, B2PrismaticJoint.SolveVelocityConstraints_s_P);
            const LA = impulse * this.m_a1;
            const LB = impulse * this.m_a2;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * LA;
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * LB;
        }
        // B2Vec2 Cdot1;
        // Cdot1.x = B2Dot(m_perp, vB - vA) + m_s2 * wB - m_s1 * wA;
        const Cdot1_x = b2Math_1.B2Vec2.DotVV(this.m_perp, b2Math_1.B2Vec2.SubVV(vB, vA, b2Math_1.B2Vec2.s_t0)) +
            this.m_s2 * wB -
            this.m_s1 * wA;
        // Cdot1.y = wB - wA;
        const Cdot1_y = wB - wA;
        if (this.m_enableLimit &&
            this.m_limitState !== 0 /* e_inactiveLimit */) {
            // Solve prismatic and limit constraint in block form.
            // float32 Cdot2;
            // Cdot2 = B2Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
            const Cdot2 = b2Math_1.B2Vec2.DotVV(this.m_axis, b2Math_1.B2Vec2.SubVV(vB, vA, b2Math_1.B2Vec2.s_t0)) +
                this.m_a2 * wB -
                this.m_a1 * wA;
            // B2Vec3 Cdot(Cdot1.x, Cdot1.y, Cdot2);
            // B2Vec3 f1 = m_impulse;
            const f1 = B2PrismaticJoint.SolveVelocityConstraints_s_f1.Copy(this.m_impulse);
            // B2Vec3 df =  m_K.Solve33(-Cdot);
            const df3 = this.m_K.Solve33(-Cdot1_x, -Cdot1_y, -Cdot2, B2PrismaticJoint.SolveVelocityConstraints_s_df3);
            // m_impulse += df;
            this.m_impulse.SelfAdd(df3);
            if (this.m_limitState === 1 /* e_atLowerLimit */) {
                this.m_impulse.z = b2Math_1.B2Max(this.m_impulse.z, 0);
            }
            else if (this.m_limitState === 2 /* e_atUpperLimit */) {
                this.m_impulse.z = b2Math_1.B2Min(this.m_impulse.z, 0);
            }
            // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
            // B2Vec2 b = -Cdot1 - (m_impulse.z - f1.z) * B2Vec2(m_K.ez.x, m_K.ez.y);
            const b_x = -Cdot1_x - (this.m_impulse.z - f1.z) * this.m_K.ez.x;
            const b_y = -Cdot1_y - (this.m_impulse.z - f1.z) * this.m_K.ez.y;
            // B2Vec2 f2r = m_K.Solve22(b) + B2Vec2(f1.x, f1.y);
            const f2r = this.m_K.Solve22(b_x, b_y, B2PrismaticJoint.SolveVelocityConstraints_s_f2r);
            f2r.x += f1.x;
            f2r.y += f1.y;
            // m_impulse.x = f2r.x;
            this.m_impulse.x = f2r.x;
            // m_impulse.y = f2r.y;
            this.m_impulse.y = f2r.y;
            // df = m_impulse - f1;
            df3.x = this.m_impulse.x - f1.x;
            df3.y = this.m_impulse.y - f1.y;
            df3.z = this.m_impulse.z - f1.z;
            // B2Vec2 P = df.x * m_perp + df.z * m_axis;
            const P = b2Math_1.B2Vec2.AddVV(b2Math_1.B2Vec2.MulSV(df3.x, this.m_perp, b2Math_1.B2Vec2.s_t0), b2Math_1.B2Vec2.MulSV(df3.z, this.m_axis, b2Math_1.B2Vec2.s_t1), B2PrismaticJoint.SolveVelocityConstraints_s_P);
            // float32 LA = df.x * m_s1 + df.y + df.z * m_a1;
            const LA = df3.x * this.m_s1 + df3.y + df3.z * this.m_a1;
            // float32 LB = df.x * m_s2 + df.y + df.z * m_a2;
            const LB = df3.x * this.m_s2 + df3.y + df3.z * this.m_a2;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * LA;
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * LB;
        }
        else {
            // Limit is inactive, just solve the prismatic constraint in block form.
            // B2Vec2 df = m_K.Solve22(-Cdot1);
            const df2 = this.m_K.Solve22(-Cdot1_x, -Cdot1_y, B2PrismaticJoint.SolveVelocityConstraints_s_df2);
            this.m_impulse.x += df2.x;
            this.m_impulse.y += df2.y;
            // B2Vec2 P = df.x * m_perp;
            const P = b2Math_1.B2Vec2.MulSV(df2.x, this.m_perp, B2PrismaticJoint.SolveVelocityConstraints_s_P);
            // float32 LA = df.x * m_s1 + df.y;
            const LA = df2.x * this.m_s1 + df2.y;
            // float32 LB = df.x * m_s2 + df.y;
            const LB = df2.x * this.m_s2 + df2.y;
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
    SolvePositionConstraints(data) {
        const cA = data.positions[this.m_indexA].c;
        let aA = data.positions[this.m_indexA].a;
        const cB = data.positions[this.m_indexB].c;
        let aB = data.positions[this.m_indexB].a;
        const qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        const mA = this.m_invMassA, mB = this.m_invMassB;
        const iA = this.m_invIA, iB = this.m_invIB;
        // B2Vec2 rA = B2Mul(qA, m_localAnchorA - m_localCenterA);
        const rA = b2Math_1.B2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // B2Vec2 rB = B2Mul(qB, m_localAnchorB - m_localCenterB);
        const rB = b2Math_1.B2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // B2Vec2 d = cB + rB - cA - rA;
        const d = b2Math_1.B2Vec2.SubVV(b2Math_1.B2Vec2.AddVV(cB, rB, b2Math_1.B2Vec2.s_t0), b2Math_1.B2Vec2.AddVV(cA, rA, b2Math_1.B2Vec2.s_t1), B2PrismaticJoint.SolvePositionConstraints_s_d);
        // B2Vec2 axis = B2Mul(qA, m_localXAxisA);
        const axis = b2Math_1.B2Rot.MulRV(qA, this.m_localXAxisA, this.m_axis);
        // float32 a1 = B2Cross(d + rA, axis);
        const a1 = b2Math_1.B2Vec2.CrossVV(b2Math_1.B2Vec2.AddVV(d, rA, b2Math_1.B2Vec2.s_t0), axis);
        // float32 a2 = B2Cross(rB, axis);
        const a2 = b2Math_1.B2Vec2.CrossVV(rB, axis);
        // B2Vec2 perp = B2Mul(qA, m_localYAxisA);
        const perp = b2Math_1.B2Rot.MulRV(qA, this.m_localYAxisA, this.m_perp);
        // float32 s1 = B2Cross(d + rA, perp);
        const s1 = b2Math_1.B2Vec2.CrossVV(b2Math_1.B2Vec2.AddVV(d, rA, b2Math_1.B2Vec2.s_t0), perp);
        // float32 s2 = B2Cross(rB, perp);
        const s2 = b2Math_1.B2Vec2.CrossVV(rB, perp);
        // B2Vec3 impulse;
        let impulse = B2PrismaticJoint.SolvePositionConstraints_s_impulse;
        // B2Vec2 C1;
        // C1.x = B2Dot(perp, d);
        const C1_x = b2Math_1.B2Vec2.DotVV(perp, d);
        // C1.y = aB - aA - m_referenceAngle;
        const C1_y = aB - aA - this.m_referenceAngle;
        let linearError = b2Math_1.B2Abs(C1_x);
        let angularError = b2Math_1.B2Abs(C1_y);
        let active = false;
        let C2 = 0;
        if (this.m_enableLimit) {
            // float32 translation = B2Dot(axis, d);
            const translation = b2Math_1.B2Vec2.DotVV(axis, d);
            if (b2Math_1.B2Abs(this.m_upperTranslation - this.m_lowerTranslation) <
                2 * b2Settings_1.B2_linearSlop) {
                // Prevent large angular corrections
                C2 = b2Math_1.B2Clamp(translation, -b2Settings_1.B2_maxLinearCorrection, b2Settings_1.B2_maxLinearCorrection);
                linearError = b2Math_1.B2Max(linearError, b2Math_1.B2Abs(translation));
                active = true;
            }
            else if (translation <= this.m_lowerTranslation) {
                // Prevent large linear corrections and allow some slop.
                C2 = b2Math_1.B2Clamp(translation - this.m_lowerTranslation + b2Settings_1.B2_linearSlop, -b2Settings_1.B2_maxLinearCorrection, 0);
                linearError = b2Math_1.B2Max(linearError, this.m_lowerTranslation - translation);
                active = true;
            }
            else if (translation >= this.m_upperTranslation) {
                // Prevent large linear corrections and allow some slop.
                C2 = b2Math_1.B2Clamp(translation - this.m_upperTranslation - b2Settings_1.B2_linearSlop, 0, b2Settings_1.B2_maxLinearCorrection);
                linearError = b2Math_1.B2Max(linearError, translation - this.m_upperTranslation);
                active = true;
            }
        }
        if (active) {
            // float32 k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            const k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            // float32 k12 = iA * s1 + iB * s2;
            const k12 = iA * s1 + iB * s2;
            // float32 k13 = iA * s1 * a1 + iB * s2 * a2;
            const k13 = iA * s1 * a1 + iB * s2 * a2;
            // float32 k22 = iA + iB;
            let k22 = iA + iB;
            if (k22 === 0) {
                // For fixed rotation
                k22 = 1;
            }
            // float32 k23 = iA * a1 + iB * a2;
            const k23 = iA * a1 + iB * a2;
            // float32 k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;
            const k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;
            // B2Mat33 K;
            const K = this.m_K3;
            // K.ex.Set(k11, k12, k13);
            K.ex.SetXYZ(k11, k12, k13);
            // K.ey.Set(k12, k22, k23);
            K.ey.SetXYZ(k12, k22, k23);
            // K.ez.Set(k13, k23, k33);
            K.ez.SetXYZ(k13, k23, k33);
            // B2Vec3 C;
            // C.x = C1.x;
            // C.y = C1.y;
            // C.z = C2;
            // impulse = K.Solve33(-C);
            impulse = K.Solve33(-C1_x, -C1_y, -C2, impulse);
        }
        else {
            // float32 k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            const k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            // float32 k12 = iA * s1 + iB * s2;
            const k12 = iA * s1 + iB * s2;
            // float32 k22 = iA + iB;
            let k22 = iA + iB;
            if (k22 === 0) {
                k22 = 1;
            }
            // B2Mat22 K;
            const K2 = this.m_K2;
            // K.ex.Set(k11, k12);
            K2.ex.Set(k11, k12);
            // K.ey.Set(k12, k22);
            K2.ey.Set(k12, k22);
            // B2Vec2 impulse1 = K.Solve(-C1);
            const impulse1 = K2.Solve(-C1_x, -C1_y, B2PrismaticJoint.SolvePositionConstraints_s_impulse1);
            impulse.x = impulse1.x;
            impulse.y = impulse1.y;
            impulse.z = 0;
        }
        // B2Vec2 P = impulse.x * perp + impulse.z * axis;
        const P = b2Math_1.B2Vec2.AddVV(b2Math_1.B2Vec2.MulSV(impulse.x, perp, b2Math_1.B2Vec2.s_t0), b2Math_1.B2Vec2.MulSV(impulse.z, axis, b2Math_1.B2Vec2.s_t1), B2PrismaticJoint.SolvePositionConstraints_s_P);
        // float32 LA = impulse.x * s1 + impulse.y + impulse.z * a1;
        const LA = impulse.x * s1 + impulse.y + impulse.z * a1;
        // float32 LB = impulse.x * s2 + impulse.y + impulse.z * a2;
        const LB = impulse.x * s2 + impulse.y + impulse.z * a2;
        // cA -= mA * P;
        cA.SelfMulSub(mA, P);
        aA -= iA * LA;
        // cB += mB * P;
        cB.SelfMulAdd(mB, P);
        aB += iB * LB;
        // data.positions[this.m_indexA].c = cA;
        data.positions[this.m_indexA].a = aA;
        // data.positions[this.m_indexB].c = cB;
        data.positions[this.m_indexB].a = aB;
        return linearError <= b2Settings_1.B2_linearSlop && angularError <= b2Settings_1.B2_angularSlop;
    }
    GetAnchorA(out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }
    GetAnchorB(out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }
    GetReactionForce(inv_dt, out) {
        // return inv_dt * (m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis);
        return out.Set(inv_dt *
            (this.m_impulse.x * this.m_perp.x +
                (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.x), inv_dt *
            (this.m_impulse.x * this.m_perp.y +
                (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.y));
    }
    GetReactionTorque(inv_dt) {
        return inv_dt * this.m_impulse.y;
    }
    GetLocalAnchorA() {
        return this.m_localAnchorA;
    }
    GetLocalAnchorB() {
        return this.m_localAnchorB;
    }
    GetLocalAxisA() {
        return this.m_localXAxisA;
    }
    GetReferenceAngle() {
        return this.m_referenceAngle;
    }
    GetJointTranslation() {
        // B2Vec2 pA = m_bodyA.GetWorldPoint(m_localAnchorA);
        const pA = this.m_bodyA.GetWorldPoint(this.m_localAnchorA, B2PrismaticJoint.GetJointTranslation_s_pA);
        // B2Vec2 pB = m_bodyB.GetWorldPoint(m_localAnchorB);
        const pB = this.m_bodyB.GetWorldPoint(this.m_localAnchorB, B2PrismaticJoint.GetJointTranslation_s_pB);
        // B2Vec2 d = pB - pA;
        const d = b2Math_1.B2Vec2.SubVV(pB, pA, B2PrismaticJoint.GetJointTranslation_s_d);
        // B2Vec2 axis = m_bodyA.GetWorldVector(m_localXAxisA);
        const axis = this.m_bodyA.GetWorldVector(this.m_localXAxisA, B2PrismaticJoint.GetJointTranslation_s_axis);
        // float32 translation = B2Dot(d, axis);
        const translation = b2Math_1.B2Vec2.DotVV(d, axis);
        return translation;
    }
    GetJointSpeed() {
        const bA = this.m_bodyA;
        const bB = this.m_bodyB;
        // B2Vec2 rA = B2Mul(bA->m_xf.q, m_localAnchorA - bA->m_sweep.localCenter);
        b2Math_1.B2Vec2.SubVV(this.m_localAnchorA, bA.m_sweep.localCenter, this.m_lalcA);
        const rA = b2Math_1.B2Rot.MulRV(bA.m_xf.q, this.m_lalcA, this.m_rA);
        // B2Vec2 rB = B2Mul(bB->m_xf.q, m_localAnchorB - bB->m_sweep.localCenter);
        b2Math_1.B2Vec2.SubVV(this.m_localAnchorB, bB.m_sweep.localCenter, this.m_lalcB);
        const rB = b2Math_1.B2Rot.MulRV(bB.m_xf.q, this.m_lalcB, this.m_rB);
        // B2Vec2 pA = bA->m_sweep.c + rA;
        const pA = b2Math_1.B2Vec2.AddVV(bA.m_sweep.c, rA, b2Math_1.B2Vec2.s_t0); // pA uses s_t0
        // B2Vec2 pB = bB->m_sweep.c + rB;
        const pB = b2Math_1.B2Vec2.AddVV(bB.m_sweep.c, rB, b2Math_1.B2Vec2.s_t1); // pB uses s_t1
        // B2Vec2 d = pB - pA;
        const d = b2Math_1.B2Vec2.SubVV(pB, pA, b2Math_1.B2Vec2.s_t2); // d uses s_t2
        // B2Vec2 axis = B2Mul(bA.m_xf.q, m_localXAxisA);
        const axis = bA.GetWorldVector(this.m_localXAxisA, this.m_axis);
        const vA = bA.m_linearVelocity;
        const vB = bB.m_linearVelocity;
        const wA = bA.m_angularVelocity;
        const wB = bB.m_angularVelocity;
        // float32 speed = B2Dot(d, B2Cross(wA, axis)) + B2Dot(axis, vB + B2Cross(wB, rB) - vA - B2Cross(wA, rA));
        const speed = b2Math_1.B2Vec2.DotVV(d, b2Math_1.B2Vec2.CrossSV(wA, axis, b2Math_1.B2Vec2.s_t0)) +
            b2Math_1.B2Vec2.DotVV(axis, b2Math_1.B2Vec2.SubVV(b2Math_1.B2Vec2.AddVCrossSV(vB, wB, rB, b2Math_1.B2Vec2.s_t0), b2Math_1.B2Vec2.AddVCrossSV(vA, wA, rA, b2Math_1.B2Vec2.s_t1), b2Math_1.B2Vec2.s_t0));
        return speed;
    }
    IsLimitEnabled() {
        return this.m_enableLimit;
    }
    EnableLimit(flag) {
        if (flag !== this.m_enableLimit) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_enableLimit = flag;
            this.m_impulse.z = 0;
        }
    }
    GetLowerLimit() {
        return this.m_lowerTranslation;
    }
    GetUpperLimit() {
        return this.m_upperTranslation;
    }
    SetLimits(lower, upper) {
        if (lower !== this.m_lowerTranslation ||
            upper !== this.m_upperTranslation) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_lowerTranslation = lower;
            this.m_upperTranslation = upper;
            this.m_impulse.z = 0;
        }
    }
    IsMotorEnabled() {
        return this.m_enableMotor;
    }
    EnableMotor(flag) {
        this.m_bodyA.SetAwake(true);
        this.m_bodyB.SetAwake(true);
        this.m_enableMotor = flag;
    }
    SetMotorSpeed(speed) {
        this.m_bodyA.SetAwake(true);
        this.m_bodyB.SetAwake(true);
        this.m_motorSpeed = speed;
    }
    GetMotorSpeed() {
        return this.m_motorSpeed;
    }
    SetMaxMotorForce(force) {
        this.m_bodyA.SetAwake(true);
        this.m_bodyB.SetAwake(true);
        this.m_maxMotorForce = force;
    }
    GetMaxMotorForce() {
        return this.m_maxMotorForce;
    }
    GetMotorForce(inv_dt) {
        return inv_dt * this.m_motorImpulse;
    }
    Dump(log) {
        const indexA = this.m_bodyA.m_islandIndex;
        const indexB = this.m_bodyB.m_islandIndex;
        log('  const jd: B2PrismaticJointDef = new B2PrismaticJointDef();\n');
        log('  jd.bodyA = bodies[%d];\n', indexA);
        log('  jd.bodyB = bodies[%d];\n', indexB);
        log('  jd.collideConnected = %s;\n', this.m_collideConnected ? 'true' : 'false');
        log('  jd.localAnchorA.Set(%.15f, %.15f);\n', this.m_localAnchorA.x, this.m_localAnchorA.y);
        log('  jd.localAnchorB.Set(%.15f, %.15f);\n', this.m_localAnchorB.x, this.m_localAnchorB.y);
        log('  jd.localAxisA.Set(%.15f, %.15f);\n', this.m_localXAxisA.x, this.m_localXAxisA.y);
        log('  jd.referenceAngle = %.15f;\n', this.m_referenceAngle);
        log('  jd.enableLimit = %s;\n', this.m_enableLimit ? 'true' : 'false');
        log('  jd.lowerTranslation = %.15f;\n', this.m_lowerTranslation);
        log('  jd.upperTranslation = %.15f;\n', this.m_upperTranslation);
        log('  jd.enableMotor = %s;\n', this.m_enableMotor ? 'true' : 'false');
        log('  jd.motorSpeed = %.15f;\n', this.m_motorSpeed);
        log('  jd.maxMotorForce = %.15f;\n', this.m_maxMotorForce);
        log('  joints[%d] = this.m_world.CreateJoint(jd);\n', this.m_index);
    }
}
exports.B2PrismaticJoint = B2PrismaticJoint;
B2PrismaticJoint.InitVelocityConstraints_s_d = new b2Math_1.B2Vec2();
B2PrismaticJoint.InitVelocityConstraints_s_P = new b2Math_1.B2Vec2();
B2PrismaticJoint.SolveVelocityConstraints_s_P = new b2Math_1.B2Vec2();
B2PrismaticJoint.SolveVelocityConstraints_s_f2r = new b2Math_1.B2Vec2();
B2PrismaticJoint.SolveVelocityConstraints_s_f1 = new b2Math_1.B2Vec3();
B2PrismaticJoint.SolveVelocityConstraints_s_df3 = new b2Math_1.B2Vec3();
B2PrismaticJoint.SolveVelocityConstraints_s_df2 = new b2Math_1.B2Vec2();
B2PrismaticJoint.SolvePositionConstraints_s_d = new b2Math_1.B2Vec2();
B2PrismaticJoint.SolvePositionConstraints_s_impulse = new b2Math_1.B2Vec3();
B2PrismaticJoint.SolvePositionConstraints_s_impulse1 = new b2Math_1.B2Vec2();
B2PrismaticJoint.SolvePositionConstraints_s_P = new b2Math_1.B2Vec2();
B2PrismaticJoint.GetJointTranslation_s_pA = new b2Math_1.B2Vec2();
B2PrismaticJoint.GetJointTranslation_s_pB = new b2Math_1.B2Vec2();
B2PrismaticJoint.GetJointTranslation_s_d = new b2Math_1.B2Vec2();
B2PrismaticJoint.GetJointTranslation_s_axis = new b2Math_1.B2Vec2();
