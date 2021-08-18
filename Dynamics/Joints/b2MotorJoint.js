"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.B2MotorJoint = exports.B2MotorJointDef = void 0;
const b2Math_1 = require("../../Common/b2Math");
const b2Joint_1 = require("./b2Joint");
class B2MotorJointDef extends b2Joint_1.B2JointDef {
    constructor() {
        super(11 /* e_motorJoint */);
        this.linearOffset = new b2Math_1.B2Vec2(0, 0);
        this.angularOffset = 0;
        this.maxForce = 1;
        this.maxTorque = 1;
        this.correctionFactor = 0.3;
    }
    Initialize(bA, bB) {
        this.bodyA = bA;
        this.bodyB = bB;
        // B2Vec2 xB = bodyB->GetPosition();
        // linearOffset = bodyA->GetLocalPoint(xB);
        this.bodyA.GetLocalPoint(this.bodyB.GetPosition(), this.linearOffset);
        const angleA = this.bodyA.GetAngle();
        const angleB = this.bodyB.GetAngle();
        this.angularOffset = angleB - angleA;
    }
}
exports.B2MotorJointDef = B2MotorJointDef;
class B2MotorJoint extends b2Joint_1.B2Joint {
    constructor(def) {
        super(def);
        // Solver shared
        this.m_linearOffset = new b2Math_1.B2Vec2();
        this.m_angularOffset = 0;
        this.m_linearImpulse = new b2Math_1.B2Vec2();
        this.m_angularImpulse = 0;
        this.m_maxForce = 0;
        this.m_maxTorque = 0;
        this.m_correctionFactor = 0.3;
        // Solver temp
        this.m_indexA = 0;
        this.m_indexB = 0;
        this.m_rA = new b2Math_1.B2Vec2();
        this.m_rB = new b2Math_1.B2Vec2();
        this.m_localCenterA = new b2Math_1.B2Vec2();
        this.m_localCenterB = new b2Math_1.B2Vec2();
        this.m_linearError = new b2Math_1.B2Vec2();
        this.m_angularError = 0;
        this.m_invMassA = 0;
        this.m_invMassB = 0;
        this.m_invIA = 0;
        this.m_invIB = 0;
        this.m_linearMass = new b2Math_1.B2Mat22();
        this.m_angularMass = 0;
        this.m_qA = new b2Math_1.B2Rot();
        this.m_qB = new b2Math_1.B2Rot();
        this.m_K = new b2Math_1.B2Mat22();
        this.m_linearOffset.Copy(def.linearOffset);
        this.m_linearImpulse.SetZero();
        this.m_maxForce = def.maxForce;
        this.m_maxTorque = def.maxTorque;
        this.m_correctionFactor = def.correctionFactor;
    }
    GetAnchorA() {
        return this.m_bodyA.GetPosition();
    }
    GetAnchorB() {
        return this.m_bodyB.GetPosition();
    }
    GetReactionForce(inv_dt, out) {
        // return inv_dt * m_linearImpulse;
        return b2Math_1.B2Vec2.MulSV(inv_dt, this.m_linearImpulse, out);
    }
    GetReactionTorque(inv_dt) {
        return inv_dt * this.m_angularImpulse;
    }
    SetLinearOffset(linearOffset) {
        if (!b2Math_1.B2Vec2.IsEqualToV(linearOffset, this.m_linearOffset)) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_linearOffset.Copy(linearOffset);
        }
    }
    GetLinearOffset() {
        return this.m_linearOffset;
    }
    SetAngularOffset(angularOffset) {
        if (angularOffset !== this.m_angularOffset) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_angularOffset = angularOffset;
        }
    }
    GetAngularOffset() {
        return this.m_angularOffset;
    }
    SetMaxForce(force) {
        /// b2Assert(B2IsValid(force) && force >= 0);
        this.m_maxForce = force;
    }
    GetMaxForce() {
        return this.m_maxForce;
    }
    SetMaxTorque(torque) {
        /// b2Assert(B2IsValid(torque) && torque >= 0);
        this.m_maxTorque = torque;
    }
    GetMaxTorque() {
        return this.m_maxTorque;
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
        // Compute the effective mass matrix.
        // this.m_rA = B2Mul(qA, -this.m_localCenterA);
        const rA = b2Math_1.B2Rot.MulRV(qA, b2Math_1.B2Vec2.NegV(this.m_localCenterA, b2Math_1.B2Vec2.s_t0), this.m_rA);
        // this.m_rB = B2Mul(qB, -this.m_localCenterB);
        const rB = b2Math_1.B2Rot.MulRV(qB, b2Math_1.B2Vec2.NegV(this.m_localCenterB, b2Math_1.B2Vec2.s_t0), this.m_rB);
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
        K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
        K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
        K.ey.x = K.ex.y;
        K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;
        // this.m_linearMass = K.GetInverse();
        K.GetInverse(this.m_linearMass);
        this.m_angularMass = iA + iB;
        if (this.m_angularMass > 0) {
            this.m_angularMass = 1 / this.m_angularMass;
        }
        // this.m_linearError = cB + rB - cA - rA - B2Mul(qA, this.m_linearOffset);
        b2Math_1.B2Vec2.SubVV(b2Math_1.B2Vec2.SubVV(b2Math_1.B2Vec2.AddVV(cB, rB, b2Math_1.B2Vec2.s_t0), b2Math_1.B2Vec2.AddVV(cA, rA, b2Math_1.B2Vec2.s_t1), b2Math_1.B2Vec2.s_t2), b2Math_1.B2Rot.MulRV(qA, this.m_linearOffset, b2Math_1.B2Vec2.s_t3), this.m_linearError);
        this.m_angularError = aB - aA - this.m_angularOffset;
        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            // this.m_linearImpulse *= data.step.dtRatio;
            this.m_linearImpulse.SelfMul(data.step.dtRatio);
            this.m_angularImpulse *= data.step.dtRatio;
            // B2Vec2 P(this.m_linearImpulse.x, this.m_linearImpulse.y);
            const P = this.m_linearImpulse;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * (b2Math_1.B2Vec2.CrossVV(rA, P) + this.m_angularImpulse);
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * (b2Math_1.B2Vec2.CrossVV(rB, P) + this.m_angularImpulse);
        }
        else {
            this.m_linearImpulse.SetZero();
            this.m_angularImpulse = 0;
        }
        // data.velocities[this.m_indexA].v = vA; // vA is a reference
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB; // vB is a reference
        data.velocities[this.m_indexB].w = wB;
    }
    SolveVelocityConstraints(data) {
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        const mA = this.m_invMassA, mB = this.m_invMassB;
        const iA = this.m_invIA, iB = this.m_invIB;
        const h = data.step.dt;
        const inv_h = data.step.inv_dt;
        // Solve angular friction
        {
            const Cdot = wB - wA + inv_h * this.m_correctionFactor * this.m_angularError;
            let impulse = -this.m_angularMass * Cdot;
            const oldImpulse = this.m_angularImpulse;
            const maxImpulse = h * this.m_maxTorque;
            this.m_angularImpulse = b2Math_1.B2Clamp(this.m_angularImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_angularImpulse - oldImpulse;
            wA -= iA * impulse;
            wB += iB * impulse;
        }
        // Solve linear friction
        {
            const rA = this.m_rA;
            const rB = this.m_rB;
            // B2Vec2 Cdot = vB + B2Vec2.CrossSV(wB, rB) - vA - B2Vec2.CrossSV(wA, rA) + inv_h * this.m_correctionFactor * this.m_linearError;
            const Cdot_v2 = b2Math_1.B2Vec2.AddVV(b2Math_1.B2Vec2.SubVV(b2Math_1.B2Vec2.AddVV(vB, b2Math_1.B2Vec2.CrossSV(wB, rB, b2Math_1.B2Vec2.s_t0), b2Math_1.B2Vec2.s_t0), b2Math_1.B2Vec2.AddVV(vA, b2Math_1.B2Vec2.CrossSV(wA, rA, b2Math_1.B2Vec2.s_t1), b2Math_1.B2Vec2.s_t1), b2Math_1.B2Vec2.s_t2), b2Math_1.B2Vec2.MulSV(inv_h * this.m_correctionFactor, this.m_linearError, b2Math_1.B2Vec2.s_t3), B2MotorJoint.SolveVelocityConstraints_s_Cdot_v2);
            // B2Vec2 impulse = -B2Mul(this.m_linearMass, Cdot);
            const impulse_v2 = b2Math_1.B2Mat22.MulMV(this.m_linearMass, Cdot_v2, B2MotorJoint.SolveVelocityConstraints_s_impulse_v2).SelfNeg();
            // B2Vec2 oldImpulse = this.m_linearImpulse;
            const oldImpulse_v2 = B2MotorJoint.SolveVelocityConstraints_s_oldImpulse_v2.Copy(this.m_linearImpulse);
            // this.m_linearImpulse += impulse;
            this.m_linearImpulse.SelfAdd(impulse_v2);
            const maxImpulse = h * this.m_maxForce;
            if (this.m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse) {
                this.m_linearImpulse.Normalize();
                // this.m_linearImpulse *= maxImpulse;
                this.m_linearImpulse.SelfMul(maxImpulse);
            }
            // impulse = this.m_linearImpulse - oldImpulse;
            b2Math_1.B2Vec2.SubVV(this.m_linearImpulse, oldImpulse_v2, impulse_v2);
            // vA -= mA * impulse;
            vA.SelfMulSub(mA, impulse_v2);
            // wA -= iA * B2Vec2.CrossVV(rA, impulse);
            wA -= iA * b2Math_1.B2Vec2.CrossVV(rA, impulse_v2);
            // vB += mB * impulse;
            vB.SelfMulAdd(mB, impulse_v2);
            // wB += iB * B2Vec2.CrossVV(rB, impulse);
            wB += iB * b2Math_1.B2Vec2.CrossVV(rB, impulse_v2);
        }
        // data.velocities[this.m_indexA].v = vA; // vA is a reference
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB; // vB is a reference
        data.velocities[this.m_indexB].w = wB;
    }
    SolvePositionConstraints(data) {
        return true;
    }
    Dump(log) {
        const indexA = this.m_bodyA.m_islandIndex;
        const indexB = this.m_bodyB.m_islandIndex;
        log('  const jd: B2MotorJointDef = new B2MotorJointDef();\n');
        log('  jd.bodyA = bodies[%d];\n', indexA);
        log('  jd.bodyB = bodies[%d];\n', indexB);
        log('  jd.collideConnected = %s;\n', this.m_collideConnected ? 'true' : 'false');
        log('  jd.linearOffset.Set(%.15f, %.15f);\n', this.m_linearOffset.x, this.m_linearOffset.y);
        log('  jd.angularOffset = %.15f;\n', this.m_angularOffset);
        log('  jd.maxForce = %.15f;\n', this.m_maxForce);
        log('  jd.maxTorque = %.15f;\n', this.m_maxTorque);
        log('  jd.correctionFactor = %.15f;\n', this.m_correctionFactor);
        log('  joints[%d] = this.m_world.CreateJoint(jd);\n', this.m_index);
    }
}
exports.B2MotorJoint = B2MotorJoint;
B2MotorJoint.SolveVelocityConstraints_s_Cdot_v2 = new b2Math_1.B2Vec2();
B2MotorJoint.SolveVelocityConstraints_s_impulse_v2 = new b2Math_1.B2Vec2();
B2MotorJoint.SolveVelocityConstraints_s_oldImpulse_v2 = new b2Math_1.B2Vec2();
