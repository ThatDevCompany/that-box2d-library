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
exports.B2WeldJoint = exports.B2WeldJointDef = void 0;
const b2Settings_1 = require("../../Common/b2Settings");
const b2Math_1 = require("../../Common/b2Math");
const b2Joint_1 = require("./b2Joint");
///  Weld joint definition. You need to specify local anchor points
///  where they are attached and the relative body angle. The position
///  of the anchor points is important for computing the reaction torque.
class B2WeldJointDef extends b2Joint_1.B2JointDef {
    constructor() {
        super(8 /* e_weldJoint */);
        this.localAnchorA = new b2Math_1.B2Vec2();
        this.localAnchorB = new b2Math_1.B2Vec2();
        this.referenceAngle = 0;
        this.frequencyHz = 0;
        this.dampingRatio = 0;
    }
    Initialize(bA, bB, anchor) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.GetLocalPoint(anchor, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchor, this.localAnchorB);
        this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
    }
}
exports.B2WeldJointDef = B2WeldJointDef;
class B2WeldJoint extends b2Joint_1.B2Joint {
    constructor(def) {
        super(def);
        this.m_frequencyHz = 0;
        this.m_dampingRatio = 0;
        this.m_bias = 0;
        // Solver shared
        this.m_localAnchorA = new b2Math_1.B2Vec2();
        this.m_localAnchorB = new b2Math_1.B2Vec2();
        this.m_referenceAngle = 0;
        this.m_gamma = 0;
        this.m_impulse = new b2Math_1.B2Vec3(0, 0, 0);
        // Solver temp
        this.m_indexA = 0;
        this.m_indexB = 0;
        this.m_rA = new b2Math_1.B2Vec2();
        this.m_rB = new b2Math_1.B2Vec2();
        this.m_localCenterA = new b2Math_1.B2Vec2();
        this.m_localCenterB = new b2Math_1.B2Vec2();
        this.m_invMassA = 0;
        this.m_invMassB = 0;
        this.m_invIA = 0;
        this.m_invIB = 0;
        this.m_mass = new b2Math_1.B2Mat33();
        this.m_qA = new b2Math_1.B2Rot();
        this.m_qB = new b2Math_1.B2Rot();
        this.m_lalcA = new b2Math_1.B2Vec2();
        this.m_lalcB = new b2Math_1.B2Vec2();
        this.m_K = new b2Math_1.B2Mat33();
        this.m_frequencyHz = def.frequencyHz;
        this.m_dampingRatio = def.dampingRatio;
        this.m_localAnchorA.Copy(def.localAnchorA);
        this.m_localAnchorB.Copy(def.localAnchorB);
        this.m_referenceAngle = def.referenceAngle;
        this.m_impulse.SetZero();
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
        const aA = data.positions[this.m_indexA].a;
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const aB = data.positions[this.m_indexB].a;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        const qA = this.m_qA.SetAngle(aA), qB = this.m_qB.SetAngle(aB);
        // m_rA = B2Mul(qA, m_localAnchorA - m_localCenterA);
        b2Math_1.B2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        b2Math_1.B2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // m_rB = B2Mul(qB, m_localAnchorB - m_localCenterB);
        b2Math_1.B2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        b2Math_1.B2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // J = [-I -r1_skew I r2_skew]
        //     [ 0       -1 0       1]
        // r_skew = [-ry; rx]
        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
        //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
        //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]
        const mA = this.m_invMassA, mB = this.m_invMassB;
        const iA = this.m_invIA, iB = this.m_invIB;
        const K = this.m_K;
        K.ex.x =
            mA + mB + this.m_rA.y * this.m_rA.y * iA + this.m_rB.y * this.m_rB.y * iB;
        K.ey.x = -this.m_rA.y * this.m_rA.x * iA - this.m_rB.y * this.m_rB.x * iB;
        K.ez.x = -this.m_rA.y * iA - this.m_rB.y * iB;
        K.ex.y = K.ey.x;
        K.ey.y =
            mA + mB + this.m_rA.x * this.m_rA.x * iA + this.m_rB.x * this.m_rB.x * iB;
        K.ez.y = this.m_rA.x * iA + this.m_rB.x * iB;
        K.ex.z = K.ez.x;
        K.ey.z = K.ez.y;
        K.ez.z = iA + iB;
        if (this.m_frequencyHz > 0) {
            K.GetInverse22(this.m_mass);
            let invM = iA + iB;
            const m = invM > 0 ? 1 / invM : 0;
            const C = aB - aA - this.m_referenceAngle;
            // Frequency
            const omega = 2 * b2Settings_1.B2_pi * this.m_frequencyHz;
            // Damping coefficient
            const d = 2 * m * this.m_dampingRatio * omega;
            // Spring stiffness
            const k = m * omega * omega;
            // magic formulas
            const h = data.step.dt;
            this.m_gamma = h * (d + h * k);
            this.m_gamma = this.m_gamma !== 0 ? 1 / this.m_gamma : 0;
            this.m_bias = C * h * k * this.m_gamma;
            invM += this.m_gamma;
            this.m_mass.ez.z = invM !== 0 ? 1 / invM : 0;
        }
        else {
            K.GetSymInverse33(this.m_mass);
            this.m_gamma = 0;
            this.m_bias = 0;
        }
        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            this.m_impulse.SelfMul(data.step.dtRatio);
            // B2Vec2 P(m_impulse.x, m_impulse.y);
            const P = B2WeldJoint.InitVelocityConstraints_s_P.Set(this.m_impulse.x, this.m_impulse.y);
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * (b2Math_1.B2Vec2.CrossVV(this.m_rA, P) + this.m_impulse.z);
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * (b2Math_1.B2Vec2.CrossVV(this.m_rB, P) + this.m_impulse.z);
        }
        else {
            this.m_impulse.SetZero();
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
        if (this.m_frequencyHz > 0) {
            const Cdot2 = wB - wA;
            const impulse2 = -this.m_mass.ez.z *
                (Cdot2 + this.m_bias + this.m_gamma * this.m_impulse.z);
            this.m_impulse.z += impulse2;
            wA -= iA * impulse2;
            wB += iB * impulse2;
            // B2Vec2 Cdot1 = vB + B2Vec2.CrossSV(wB, this.m_rB) - vA - B2Vec2.CrossSV(wA, this.m_rA);
            const Cdot1 = b2Math_1.B2Vec2.SubVV(b2Math_1.B2Vec2.AddVCrossSV(vB, wB, this.m_rB, b2Math_1.B2Vec2.s_t0), b2Math_1.B2Vec2.AddVCrossSV(vA, wA, this.m_rA, b2Math_1.B2Vec2.s_t1), B2WeldJoint.SolveVelocityConstraints_s_Cdot1);
            // B2Vec2 impulse1 = -B2Mul22(m_mass, Cdot1);
            const impulse1 = b2Math_1.B2Mat33.MulM33XY(this.m_mass, Cdot1.x, Cdot1.y, B2WeldJoint.SolveVelocityConstraints_s_impulse1).SelfNeg();
            this.m_impulse.x += impulse1.x;
            this.m_impulse.y += impulse1.y;
            // B2Vec2 P = impulse1;
            const P = impulse1;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            // wA -= iA * B2Cross(m_rA, P);
            wA -= iA * b2Math_1.B2Vec2.CrossVV(this.m_rA, P);
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            // wB += iB * B2Cross(m_rB, P);
            wB += iB * b2Math_1.B2Vec2.CrossVV(this.m_rB, P);
        }
        else {
            // B2Vec2 Cdot1 = vB + B2Cross(wB, this.m_rB) - vA - B2Cross(wA, this.m_rA);
            const Cdot1 = b2Math_1.B2Vec2.SubVV(b2Math_1.B2Vec2.AddVCrossSV(vB, wB, this.m_rB, b2Math_1.B2Vec2.s_t0), b2Math_1.B2Vec2.AddVCrossSV(vA, wA, this.m_rA, b2Math_1.B2Vec2.s_t1), B2WeldJoint.SolveVelocityConstraints_s_Cdot1);
            const Cdot2 = wB - wA;
            // B2Vec3 const Cdot(Cdot1.x, Cdot1.y, Cdot2);
            // B2Vec3 impulse = -B2Mul(m_mass, Cdot);
            const impulse = b2Math_1.B2Mat33.MulM33XYZ(this.m_mass, Cdot1.x, Cdot1.y, Cdot2, B2WeldJoint.SolveVelocityConstraints_s_impulse).SelfNeg();
            this.m_impulse.SelfAdd(impulse);
            // B2Vec2 P(impulse.x, impulse.y);
            const P = B2WeldJoint.SolveVelocityConstraints_s_P.Set(impulse.x, impulse.y);
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * (b2Math_1.B2Vec2.CrossVV(this.m_rA, P) + impulse.z);
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * (b2Math_1.B2Vec2.CrossVV(this.m_rB, P) + impulse.z);
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
        b2Math_1.B2Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        const rA = b2Math_1.B2Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // B2Vec2 rB = B2Mul(qB, m_localAnchorB - m_localCenterB);
        b2Math_1.B2Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        const rB = b2Math_1.B2Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        let positionError, angularError;
        const K = this.m_K;
        K.ex.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
        K.ey.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
        K.ez.x = -rA.y * iA - rB.y * iB;
        K.ex.y = K.ey.x;
        K.ey.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
        K.ez.y = rA.x * iA + rB.x * iB;
        K.ex.z = K.ez.x;
        K.ey.z = K.ez.y;
        K.ez.z = iA + iB;
        if (this.m_frequencyHz > 0) {
            // B2Vec2 C1 =  cB + rB - cA - rA;
            const C1 = b2Math_1.B2Vec2.SubVV(b2Math_1.B2Vec2.AddVV(cB, rB, b2Math_1.B2Vec2.s_t0), b2Math_1.B2Vec2.AddVV(cA, rA, b2Math_1.B2Vec2.s_t1), B2WeldJoint.SolvePositionConstraints_s_C1);
            positionError = C1.Length();
            angularError = 0;
            // B2Vec2 P = -K.Solve22(C1);
            const P = K.Solve22(C1.x, C1.y, B2WeldJoint.SolvePositionConstraints_s_P).SelfNeg();
            // cA -= mA * P;
            cA.SelfMulSub(mA, P);
            aA -= iA * b2Math_1.B2Vec2.CrossVV(rA, P);
            // cB += mB * P;
            cB.SelfMulAdd(mB, P);
            aB += iB * b2Math_1.B2Vec2.CrossVV(rB, P);
        }
        else {
            // B2Vec2 C1 =  cB + rB - cA - rA;
            const C1 = b2Math_1.B2Vec2.SubVV(b2Math_1.B2Vec2.AddVV(cB, rB, b2Math_1.B2Vec2.s_t0), b2Math_1.B2Vec2.AddVV(cA, rA, b2Math_1.B2Vec2.s_t1), B2WeldJoint.SolvePositionConstraints_s_C1);
            const C2 = aB - aA - this.m_referenceAngle;
            positionError = C1.Length();
            angularError = b2Math_1.B2Abs(C2);
            // B2Vec3 C(C1.x, C1.y, C2);
            // B2Vec3 impulse = -K.Solve33(C);
            const impulse = K.Solve33(C1.x, C1.y, C2, B2WeldJoint.SolvePositionConstraints_s_impulse).SelfNeg();
            // B2Vec2 P(impulse.x, impulse.y);
            const P = B2WeldJoint.SolvePositionConstraints_s_P.Set(impulse.x, impulse.y);
            // cA -= mA * P;
            cA.SelfMulSub(mA, P);
            aA -= iA * (b2Math_1.B2Vec2.CrossVV(this.m_rA, P) + impulse.z);
            // cB += mB * P;
            cB.SelfMulAdd(mB, P);
            aB += iB * (b2Math_1.B2Vec2.CrossVV(this.m_rB, P) + impulse.z);
        }
        // data.positions[this.m_indexA].c = cA;
        data.positions[this.m_indexA].a = aA;
        // data.positions[this.m_indexB].c = cB;
        data.positions[this.m_indexB].a = aB;
        return positionError <= b2Settings_1.B2_linearSlop && angularError <= b2Settings_1.B2_angularSlop;
    }
    GetAnchorA(out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }
    GetAnchorB(out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }
    GetReactionForce(inv_dt, out) {
        // B2Vec2 P(this.m_impulse.x, this.m_impulse.y);
        // return inv_dt * P;
        return out.Set(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
    }
    GetReactionTorque(inv_dt) {
        return inv_dt * this.m_impulse.z;
    }
    GetLocalAnchorA() {
        return this.m_localAnchorA;
    }
    GetLocalAnchorB() {
        return this.m_localAnchorB;
    }
    GetReferenceAngle() {
        return this.m_referenceAngle;
    }
    SetFrequency(hz) {
        this.m_frequencyHz = hz;
    }
    GetFrequency() {
        return this.m_frequencyHz;
    }
    SetDampingRatio(ratio) {
        this.m_dampingRatio = ratio;
    }
    GetDampingRatio() {
        return this.m_dampingRatio;
    }
    Dump(log) {
        const indexA = this.m_bodyA.m_islandIndex;
        const indexB = this.m_bodyB.m_islandIndex;
        log('  const jd: B2WeldJointDef = new B2WeldJointDef();\n');
        log('  jd.bodyA = bodies[%d];\n', indexA);
        log('  jd.bodyB = bodies[%d];\n', indexB);
        log('  jd.collideConnected = %s;\n', this.m_collideConnected ? 'true' : 'false');
        log('  jd.localAnchorA.Set(%.15f, %.15f);\n', this.m_localAnchorA.x, this.m_localAnchorA.y);
        log('  jd.localAnchorB.Set(%.15f, %.15f);\n', this.m_localAnchorB.x, this.m_localAnchorB.y);
        log('  jd.referenceAngle = %.15f;\n', this.m_referenceAngle);
        log('  jd.frequencyHz = %.15f;\n', this.m_frequencyHz);
        log('  jd.dampingRatio = %.15f;\n', this.m_dampingRatio);
        log('  joints[%d] = this.m_world.CreateJoint(jd);\n', this.m_index);
    }
}
exports.B2WeldJoint = B2WeldJoint;
B2WeldJoint.InitVelocityConstraints_s_P = new b2Math_1.B2Vec2();
B2WeldJoint.SolveVelocityConstraints_s_Cdot1 = new b2Math_1.B2Vec2();
B2WeldJoint.SolveVelocityConstraints_s_impulse1 = new b2Math_1.B2Vec2();
B2WeldJoint.SolveVelocityConstraints_s_impulse = new b2Math_1.B2Vec3();
B2WeldJoint.SolveVelocityConstraints_s_P = new b2Math_1.B2Vec2();
B2WeldJoint.SolvePositionConstraints_s_C1 = new b2Math_1.B2Vec2();
B2WeldJoint.SolvePositionConstraints_s_P = new b2Math_1.B2Vec2();
B2WeldJoint.SolvePositionConstraints_s_impulse = new b2Math_1.B2Vec3();
