"use strict";
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
Object.defineProperty(exports, "__esModule", { value: true });
exports.B2ContactSolver = exports.B2PositionSolverManifold = exports.B2ContactSolverDef = exports.B2ContactPositionConstraint = exports.B2ContactVelocityConstraint = exports.B2VelocityConstraintPoint = void 0;
const b2Settings_1 = require("../../Common/b2Settings");
const b2Math_1 = require("../../Common/b2Math");
const b2Collision_1 = require("../../Collision/b2Collision");
const b2TimeStep_1 = require("../b2TimeStep");
class B2VelocityConstraintPoint {
    constructor() {
        this.rA = new b2Math_1.B2Vec2();
        this.rB = new b2Math_1.B2Vec2();
        this.normalImpulse = 0;
        this.tangentImpulse = 0;
        this.normalMass = 0;
        this.tangentMass = 0;
        this.velocityBias = 0;
    }
    static MakeArray(length) {
        return b2Settings_1.B2MakeArray(length, function (i) {
            return new B2VelocityConstraintPoint();
        });
    }
}
exports.B2VelocityConstraintPoint = B2VelocityConstraintPoint;
class B2ContactVelocityConstraint {
    constructor() {
        this.points = B2VelocityConstraintPoint.MakeArray(b2Settings_1.B2_maxManifoldPoints);
        this.normal = new b2Math_1.B2Vec2();
        this.tangent = new b2Math_1.B2Vec2();
        this.normalMass = new b2Math_1.B2Mat22();
        this.K = new b2Math_1.B2Mat22();
        this.indexA = 0;
        this.indexB = 0;
        this.invMassA = 0;
        this.invMassB = 0;
        this.invIA = 0;
        this.invIB = 0;
        this.friction = 0;
        this.restitution = 0;
        this.tangentSpeed = 0;
        this.pointCount = 0;
        this.contactIndex = 0;
    }
    static MakeArray(length) {
        return b2Settings_1.B2MakeArray(length, function (i) {
            return new B2ContactVelocityConstraint();
        });
    }
}
exports.B2ContactVelocityConstraint = B2ContactVelocityConstraint;
class B2ContactPositionConstraint {
    constructor() {
        this.localPoints = b2Math_1.B2Vec2.MakeArray(b2Settings_1.B2_maxManifoldPoints);
        this.localNormal = new b2Math_1.B2Vec2();
        this.localPoint = new b2Math_1.B2Vec2();
        this.indexA = 0;
        this.indexB = 0;
        this.invMassA = 0;
        this.invMassB = 0;
        this.localCenterA = new b2Math_1.B2Vec2();
        this.localCenterB = new b2Math_1.B2Vec2();
        this.invIA = 0;
        this.invIB = 0;
        this.type = -1 /* e_unknown */;
        this.radiusA = 0;
        this.radiusB = 0;
        this.pointCount = 0;
    }
    static MakeArray(length) {
        return b2Settings_1.B2MakeArray(length, function (i) {
            return new B2ContactPositionConstraint();
        });
    }
}
exports.B2ContactPositionConstraint = B2ContactPositionConstraint;
class B2ContactSolverDef {
    constructor() {
        this.step = new b2TimeStep_1.B2TimeStep();
        this.contacts = null;
        this.count = 0;
        this.positions = null;
        this.velocities = null;
        this.allocator = null;
    }
}
exports.B2ContactSolverDef = B2ContactSolverDef;
class B2PositionSolverManifold {
    constructor() {
        this.normal = new b2Math_1.B2Vec2();
        this.point = new b2Math_1.B2Vec2();
        this.separation = 0;
    }
    Initialize(pc, xfA, xfB, index) {
        const pointA = B2PositionSolverManifold.Initialize_s_pointA;
        const pointB = B2PositionSolverManifold.Initialize_s_pointB;
        const planePoint = B2PositionSolverManifold.Initialize_s_planePoint;
        const clipPoint = B2PositionSolverManifold.Initialize_s_clipPoint;
        /// b2Assert(pc.pointCount > 0);
        switch (pc.type) {
            case 0 /* e_circles */:
                {
                    // B2Vec2 pointA = B2Mul(xfA, pc->localPoint);
                    b2Math_1.B2Transform.MulXV(xfA, pc.localPoint, pointA);
                    // B2Vec2 pointB = B2Mul(xfB, pc->localPoints[0]);
                    b2Math_1.B2Transform.MulXV(xfB, pc.localPoints[0], pointB);
                    // normal = pointB - pointA;
                    // normal.Normalize();
                    b2Math_1.B2Vec2.SubVV(pointB, pointA, this.normal).SelfNormalize();
                    // point = 0.5f * (pointA + pointB);
                    b2Math_1.B2Vec2.MidVV(pointA, pointB, this.point);
                    // separation = B2Dot(pointB - pointA, normal) - pc->radius;
                    this.separation =
                        b2Math_1.B2Vec2.DotVV(b2Math_1.B2Vec2.SubVV(pointB, pointA, b2Math_1.B2Vec2.s_t0), this.normal) -
                            pc.radiusA -
                            pc.radiusB;
                }
                break;
            case 1 /* e_faceA */:
                {
                    // normal = B2Mul(xfA.q, pc->localNormal);
                    b2Math_1.B2Rot.MulRV(xfA.q, pc.localNormal, this.normal);
                    // B2Vec2 planePoint = B2Mul(xfA, pc->localPoint);
                    b2Math_1.B2Transform.MulXV(xfA, pc.localPoint, planePoint);
                    // B2Vec2 clipPoint = B2Mul(xfB, pc->localPoints[index]);
                    b2Math_1.B2Transform.MulXV(xfB, pc.localPoints[index], clipPoint);
                    // separation = B2Dot(clipPoint - planePoint, normal) - pc->radius;
                    this.separation =
                        b2Math_1.B2Vec2.DotVV(b2Math_1.B2Vec2.SubVV(clipPoint, planePoint, b2Math_1.B2Vec2.s_t0), this.normal) -
                            pc.radiusA -
                            pc.radiusB;
                    // point = clipPoint;
                    this.point.Copy(clipPoint);
                }
                break;
            case 2 /* e_faceB */:
                {
                    // normal = B2Mul(xfB.q, pc->localNormal);
                    b2Math_1.B2Rot.MulRV(xfB.q, pc.localNormal, this.normal);
                    // B2Vec2 planePoint = B2Mul(xfB, pc->localPoint);
                    b2Math_1.B2Transform.MulXV(xfB, pc.localPoint, planePoint);
                    // B2Vec2 clipPoint = B2Mul(xfA, pc->localPoints[index]);
                    b2Math_1.B2Transform.MulXV(xfA, pc.localPoints[index], clipPoint);
                    // separation = B2Dot(clipPoint - planePoint, normal) - pc->radius;
                    this.separation =
                        b2Math_1.B2Vec2.DotVV(b2Math_1.B2Vec2.SubVV(clipPoint, planePoint, b2Math_1.B2Vec2.s_t0), this.normal) -
                            pc.radiusA -
                            pc.radiusB;
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
exports.B2PositionSolverManifold = B2PositionSolverManifold;
B2PositionSolverManifold.Initialize_s_pointA = new b2Math_1.B2Vec2();
B2PositionSolverManifold.Initialize_s_pointB = new b2Math_1.B2Vec2();
B2PositionSolverManifold.Initialize_s_planePoint = new b2Math_1.B2Vec2();
B2PositionSolverManifold.Initialize_s_clipPoint = new b2Math_1.B2Vec2();
class B2ContactSolver {
    constructor() {
        this.m_step = new b2TimeStep_1.B2TimeStep();
        this.m_positions = null;
        this.m_velocities = null;
        this.m_allocator = null;
        this.m_positionConstraints = B2ContactPositionConstraint.MakeArray(1024); // TODO: B2Settings
        this.m_velocityConstraints = B2ContactVelocityConstraint.MakeArray(1024); // TODO: B2Settings
        this.m_contacts = null;
        this.m_count = 0;
    }
    Initialize(def) {
        this.m_step.Copy(def.step);
        this.m_allocator = def.allocator;
        this.m_count = def.count;
        // TODO:
        if (this.m_positionConstraints.length < this.m_count) {
            const new_length = b2Math_1.B2Max(this.m_positionConstraints.length * 2, this.m_count);
            while (this.m_positionConstraints.length < new_length) {
                this.m_positionConstraints[this.m_positionConstraints.length] = new B2ContactPositionConstraint();
            }
        }
        // TODO:
        if (this.m_velocityConstraints.length < this.m_count) {
            const new_length = b2Math_1.B2Max(this.m_velocityConstraints.length * 2, this.m_count);
            while (this.m_velocityConstraints.length < new_length) {
                this.m_velocityConstraints[this.m_velocityConstraints.length] = new B2ContactVelocityConstraint();
            }
        }
        this.m_positions = def.positions;
        this.m_velocities = def.velocities;
        this.m_contacts = def.contacts;
        // Initialize position independent portions of the constraints.
        for (let i = 0; i < this.m_count; ++i) {
            const contact = this.m_contacts[i];
            const fixtureA = contact.m_fixtureA;
            const fixtureB = contact.m_fixtureB;
            const shapeA = fixtureA.GetShape();
            const shapeB = fixtureB.GetShape();
            const radiusA = shapeA.m_radius;
            const radiusB = shapeB.m_radius;
            const bodyA = fixtureA.GetBody();
            const bodyB = fixtureB.GetBody();
            const manifold = contact.GetManifold();
            const pointCount = manifold.pointCount;
            /// b2Assert(pointCount > 0);
            const vc = this.m_velocityConstraints[i];
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
            const pc = this.m_positionConstraints[i];
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
            for (let j = 0; j < pointCount; ++j) {
                const cp = manifold.points[j];
                const vcp = vc.points[j];
                if (this.m_step.warmStarting) {
                    vcp.normalImpulse = this.m_step.dtRatio * cp.normalImpulse;
                    vcp.tangentImpulse = this.m_step.dtRatio * cp.tangentImpulse;
                }
                else {
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
    InitializeVelocityConstraints() {
        const xfA = B2ContactSolver.InitializeVelocityConstraints_s_xfA;
        const xfB = B2ContactSolver.InitializeVelocityConstraints_s_xfB;
        const worldManifold = B2ContactSolver.InitializeVelocityConstraints_s_worldManifold;
        const k_maxConditionNumber = 1000;
        for (let i = 0; i < this.m_count; ++i) {
            const vc = this.m_velocityConstraints[i];
            const pc = this.m_positionConstraints[i];
            const radiusA = pc.radiusA;
            const radiusB = pc.radiusB;
            const manifold = this.m_contacts[vc.contactIndex].GetManifold();
            const indexA = vc.indexA;
            const indexB = vc.indexB;
            const mA = vc.invMassA;
            const mB = vc.invMassB;
            const iA = vc.invIA;
            const iB = vc.invIB;
            const localCenterA = pc.localCenterA;
            const localCenterB = pc.localCenterB;
            const cA = this.m_positions[indexA].c;
            const aA = this.m_positions[indexA].a;
            const vA = this.m_velocities[indexA].v;
            const wA = this.m_velocities[indexA].w;
            const cB = this.m_positions[indexB].c;
            const aB = this.m_positions[indexB].a;
            const vB = this.m_velocities[indexB].v;
            const wB = this.m_velocities[indexB].w;
            /// b2Assert(manifold.pointCount > 0);
            xfA.q.SetAngle(aA);
            xfB.q.SetAngle(aB);
            b2Math_1.B2Vec2.SubVV(cA, b2Math_1.B2Rot.MulRV(xfA.q, localCenterA, b2Math_1.B2Vec2.s_t0), xfA.p);
            b2Math_1.B2Vec2.SubVV(cB, b2Math_1.B2Rot.MulRV(xfB.q, localCenterB, b2Math_1.B2Vec2.s_t0), xfB.p);
            worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB);
            vc.normal.Copy(worldManifold.normal);
            b2Math_1.B2Vec2.CrossVOne(vc.normal, vc.tangent); // compute from normal
            const pointCount = vc.pointCount;
            for (let j = 0; j < pointCount; ++j) {
                const vcp = vc.points[j];
                // vcp->rA = worldManifold.points[j] - cA;
                b2Math_1.B2Vec2.SubVV(worldManifold.points[j], cA, vcp.rA);
                // vcp->rB = worldManifold.points[j] - cB;
                b2Math_1.B2Vec2.SubVV(worldManifold.points[j], cB, vcp.rB);
                const rnA = b2Math_1.B2Vec2.CrossVV(vcp.rA, vc.normal);
                const rnB = b2Math_1.B2Vec2.CrossVV(vcp.rB, vc.normal);
                const kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                vcp.normalMass = kNormal > 0 ? 1 / kNormal : 0;
                // B2Vec2 tangent = B2Cross(vc->normal, 1.0f);
                const tangent = vc.tangent; // precomputed from normal
                const rtA = b2Math_1.B2Vec2.CrossVV(vcp.rA, tangent);
                const rtB = b2Math_1.B2Vec2.CrossVV(vcp.rB, tangent);
                const kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                vcp.tangentMass = kTangent > 0 ? 1 / kTangent : 0;
                // Setup a velocity bias for restitution.
                vcp.velocityBias = 0;
                // float32 vRel = B2Dot(vc->normal, vB + B2Cross(wB, vcp->rB) - vA - B2Cross(wA, vcp->rA));
                const vRel = b2Math_1.B2Vec2.DotVV(vc.normal, b2Math_1.B2Vec2.SubVV(b2Math_1.B2Vec2.AddVCrossSV(vB, wB, vcp.rB, b2Math_1.B2Vec2.s_t0), b2Math_1.B2Vec2.AddVCrossSV(vA, wA, vcp.rA, b2Math_1.B2Vec2.s_t1), b2Math_1.B2Vec2.s_t0));
                if (vRel < -b2Settings_1.B2_velocityThreshold) {
                    vcp.velocityBias += -vc.restitution * vRel;
                }
            }
            // If we have two points, then prepare the block solver.
            if (vc.pointCount === 2) {
                const vcp1 = vc.points[0];
                const vcp2 = vc.points[1];
                const rn1A = b2Math_1.B2Vec2.CrossVV(vcp1.rA, vc.normal);
                const rn1B = b2Math_1.B2Vec2.CrossVV(vcp1.rB, vc.normal);
                const rn2A = b2Math_1.B2Vec2.CrossVV(vcp2.rA, vc.normal);
                const rn2B = b2Math_1.B2Vec2.CrossVV(vcp2.rB, vc.normal);
                const k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
                const k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
                const k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;
                // Ensure a reasonable condition number.
                // float32 k_maxConditionNumber = 1000.0f;
                if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
                    // K is safe to invert.
                    vc.K.ex.Set(k11, k12);
                    vc.K.ey.Set(k12, k22);
                    vc.K.GetInverse(vc.normalMass);
                }
                else {
                    // The constraints are redundant, just use one.
                    // TODO_ERIN use deepest?
                    vc.pointCount = 1;
                }
            }
        }
    }
    WarmStart() {
        const P = B2ContactSolver.WarmStart_s_P;
        // Warm start.
        for (let i = 0; i < this.m_count; ++i) {
            const vc = this.m_velocityConstraints[i];
            const indexA = vc.indexA;
            const indexB = vc.indexB;
            const mA = vc.invMassA;
            const iA = vc.invIA;
            const mB = vc.invMassB;
            const iB = vc.invIB;
            const pointCount = vc.pointCount;
            const vA = this.m_velocities[indexA].v;
            let wA = this.m_velocities[indexA].w;
            const vB = this.m_velocities[indexB].v;
            let wB = this.m_velocities[indexB].w;
            const normal = vc.normal;
            // B2Vec2 tangent = B2Cross(normal, 1.0f);
            const tangent = vc.tangent; // precomputed from normal
            for (let j = 0; j < pointCount; ++j) {
                const vcp = vc.points[j];
                // B2Vec2 P = vcp->normalImpulse * normal + vcp->tangentImpulse * tangent;
                b2Math_1.B2Vec2.AddVV(b2Math_1.B2Vec2.MulSV(vcp.normalImpulse, normal, b2Math_1.B2Vec2.s_t0), b2Math_1.B2Vec2.MulSV(vcp.tangentImpulse, tangent, b2Math_1.B2Vec2.s_t1), P);
                // wA -= iA * B2Cross(vcp->rA, P);
                wA -= iA * b2Math_1.B2Vec2.CrossVV(vcp.rA, P);
                // vA -= mA * P;
                vA.SelfMulSub(mA, P);
                // wB += iB * B2Cross(vcp->rB, P);
                wB += iB * b2Math_1.B2Vec2.CrossVV(vcp.rB, P);
                // vB += mB * P;
                vB.SelfMulAdd(mB, P);
            }
            // this.m_velocities[indexA].v = vA;
            this.m_velocities[indexA].w = wA;
            // this.m_velocities[indexB].v = vB;
            this.m_velocities[indexB].w = wB;
        }
    }
    SolveVelocityConstraints() {
        const dv = B2ContactSolver.SolveVelocityConstraints_s_dv;
        const dv1 = B2ContactSolver.SolveVelocityConstraints_s_dv1;
        const dv2 = B2ContactSolver.SolveVelocityConstraints_s_dv2;
        const P = B2ContactSolver.SolveVelocityConstraints_s_P;
        const a = B2ContactSolver.SolveVelocityConstraints_s_a;
        const b = B2ContactSolver.SolveVelocityConstraints_s_b;
        const x = B2ContactSolver.SolveVelocityConstraints_s_x;
        const d = B2ContactSolver.SolveVelocityConstraints_s_d;
        const P1 = B2ContactSolver.SolveVelocityConstraints_s_P1;
        const P2 = B2ContactSolver.SolveVelocityConstraints_s_P2;
        const P1P2 = B2ContactSolver.SolveVelocityConstraints_s_P1P2;
        for (let i = 0; i < this.m_count; ++i) {
            const vc = this.m_velocityConstraints[i];
            const indexA = vc.indexA;
            const indexB = vc.indexB;
            const mA = vc.invMassA;
            const iA = vc.invIA;
            const mB = vc.invMassB;
            const iB = vc.invIB;
            const pointCount = vc.pointCount;
            const vA = this.m_velocities[indexA].v;
            let wA = this.m_velocities[indexA].w;
            const vB = this.m_velocities[indexB].v;
            let wB = this.m_velocities[indexB].w;
            // B2Vec2 normal = vc->normal;
            const normal = vc.normal;
            // B2Vec2 tangent = B2Cross(normal, 1.0f);
            const tangent = vc.tangent; // precomputed from normal
            const friction = vc.friction;
            /// b2Assert(pointCount === 1 || pointCount === 2);
            // Solve tangent constraints first because non-penetration is more important
            // than friction.
            for (let j = 0; j < pointCount; ++j) {
                const vcp = vc.points[j];
                // Relative velocity at contact
                // B2Vec2 dv = vB + B2Cross(wB, vcp->rB) - vA - B2Cross(wA, vcp->rA);
                b2Math_1.B2Vec2.SubVV(b2Math_1.B2Vec2.AddVCrossSV(vB, wB, vcp.rB, b2Math_1.B2Vec2.s_t0), b2Math_1.B2Vec2.AddVCrossSV(vA, wA, vcp.rA, b2Math_1.B2Vec2.s_t1), dv);
                // Compute tangent force
                // float32 vt = B2Dot(dv, tangent) - vc->tangentSpeed;
                const vt = b2Math_1.B2Vec2.DotVV(dv, tangent) - vc.tangentSpeed;
                let lambda = vcp.tangentMass * -vt;
                // B2Clamp the accumulated force
                const maxFriction = friction * vcp.normalImpulse;
                const newImpulse = b2Math_1.B2Clamp(vcp.tangentImpulse + lambda, -maxFriction, maxFriction);
                lambda = newImpulse - vcp.tangentImpulse;
                vcp.tangentImpulse = newImpulse;
                // Apply contact impulse
                // B2Vec2 P = lambda * tangent;
                b2Math_1.B2Vec2.MulSV(lambda, tangent, P);
                // vA -= mA * P;
                vA.SelfMulSub(mA, P);
                // wA -= iA * B2Cross(vcp->rA, P);
                wA -= iA * b2Math_1.B2Vec2.CrossVV(vcp.rA, P);
                // vB += mB * P;
                vB.SelfMulAdd(mB, P);
                // wB += iB * B2Cross(vcp->rB, P);
                wB += iB * b2Math_1.B2Vec2.CrossVV(vcp.rB, P);
            }
            // Solve normal constraints
            if (vc.pointCount === 1) {
                const vcp = vc.points[0];
                // Relative velocity at contact
                // B2Vec2 dv = vB + B2Cross(wB, vcp->rB) - vA - B2Cross(wA, vcp->rA);
                b2Math_1.B2Vec2.SubVV(b2Math_1.B2Vec2.AddVCrossSV(vB, wB, vcp.rB, b2Math_1.B2Vec2.s_t0), b2Math_1.B2Vec2.AddVCrossSV(vA, wA, vcp.rA, b2Math_1.B2Vec2.s_t1), dv);
                // Compute normal impulse
                // float32 vn = B2Dot(dv, normal);
                const vn = b2Math_1.B2Vec2.DotVV(dv, normal);
                let lambda = -vcp.normalMass * (vn - vcp.velocityBias);
                // B2Clamp the accumulated impulse
                // float32 newImpulse = B2Max(vcp->normalImpulse + lambda, 0.0f);
                const newImpulse = b2Math_1.B2Max(vcp.normalImpulse + lambda, 0);
                lambda = newImpulse - vcp.normalImpulse;
                vcp.normalImpulse = newImpulse;
                // Apply contact impulse
                // B2Vec2 P = lambda * normal;
                b2Math_1.B2Vec2.MulSV(lambda, normal, P);
                // vA -= mA * P;
                vA.SelfMulSub(mA, P);
                // wA -= iA * B2Cross(vcp->rA, P);
                wA -= iA * b2Math_1.B2Vec2.CrossVV(vcp.rA, P);
                // vB += mB * P;
                vB.SelfMulAdd(mB, P);
                // wB += iB * B2Cross(vcp->rB, P);
                wB += iB * b2Math_1.B2Vec2.CrossVV(vcp.rB, P);
            }
            else {
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
                const cp1 = vc.points[0];
                const cp2 = vc.points[1];
                // B2Vec2 a(cp1->normalImpulse, cp2->normalImpulse);
                a.Set(cp1.normalImpulse, cp2.normalImpulse);
                /// b2Assert(a.x >= 0 && a.y >= 0);
                // Relative velocity at contact
                // B2Vec2 dv1 = vB + B2Cross(wB, cp1->rB) - vA - B2Cross(wA, cp1->rA);
                b2Math_1.B2Vec2.SubVV(b2Math_1.B2Vec2.AddVCrossSV(vB, wB, cp1.rB, b2Math_1.B2Vec2.s_t0), b2Math_1.B2Vec2.AddVCrossSV(vA, wA, cp1.rA, b2Math_1.B2Vec2.s_t1), dv1);
                // B2Vec2 dv2 = vB + B2Cross(wB, cp2->rB) - vA - B2Cross(wA, cp2->rA);
                b2Math_1.B2Vec2.SubVV(b2Math_1.B2Vec2.AddVCrossSV(vB, wB, cp2.rB, b2Math_1.B2Vec2.s_t0), b2Math_1.B2Vec2.AddVCrossSV(vA, wA, cp2.rA, b2Math_1.B2Vec2.s_t1), dv2);
                // Compute normal velocity
                // float32 vn1 = B2Dot(dv1, normal);
                let vn1 = b2Math_1.B2Vec2.DotVV(dv1, normal);
                // float32 vn2 = B2Dot(dv2, normal);
                let vn2 = b2Math_1.B2Vec2.DotVV(dv2, normal);
                // B2Vec2 b;
                b.x = vn1 - cp1.velocityBias;
                b.y = vn2 - cp2.velocityBias;
                // Compute b'
                // b -= B2Mul(vc->K, a);
                b.SelfSub(b2Math_1.B2Mat22.MulMV(vc.K, a, b2Math_1.B2Vec2.s_t0));
                /*
                #if B2_DEBUG_SOLVER === 1
                const k_errorTol: number = 0.001;
                #endif
                */
                for (;;) {
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
                    b2Math_1.B2Mat22.MulMV(vc.normalMass, b, x).SelfNeg();
                    if (x.x >= 0 && x.y >= 0) {
                        // Get the incremental impulse
                        // B2Vec2 d = x - a;
                        b2Math_1.B2Vec2.SubVV(x, a, d);
                        // Apply incremental impulse
                        // B2Vec2 P1 = d.x * normal;
                        b2Math_1.B2Vec2.MulSV(d.x, normal, P1);
                        // B2Vec2 P2 = d.y * normal;
                        b2Math_1.B2Vec2.MulSV(d.y, normal, P2);
                        b2Math_1.B2Vec2.AddVV(P1, P2, P1P2);
                        // vA -= mA * (P1 + P2);
                        vA.SelfMulSub(mA, P1P2);
                        // wA -= iA * (B2Cross(cp1->rA, P1) + B2Cross(cp2->rA, P2));
                        wA -= iA * (b2Math_1.B2Vec2.CrossVV(cp1.rA, P1) + b2Math_1.B2Vec2.CrossVV(cp2.rA, P2));
                        // vB += mB * (P1 + P2);
                        vB.SelfMulAdd(mB, P1P2);
                        // wB += iB * (B2Cross(cp1->rB, P1) + B2Cross(cp2->rB, P2));
                        wB += iB * (b2Math_1.B2Vec2.CrossVV(cp1.rB, P1) + b2Math_1.B2Vec2.CrossVV(cp2.rB, P2));
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
                    x.x = -cp1.normalMass * b.x;
                    x.y = 0;
                    vn1 = 0;
                    vn2 = vc.K.ex.y * x.x + b.y;
                    if (x.x >= 0 && vn2 >= 0) {
                        // Get the incremental impulse
                        // B2Vec2 d = x - a;
                        b2Math_1.B2Vec2.SubVV(x, a, d);
                        // Apply incremental impulse
                        // B2Vec2 P1 = d.x * normal;
                        b2Math_1.B2Vec2.MulSV(d.x, normal, P1);
                        // B2Vec2 P2 = d.y * normal;
                        b2Math_1.B2Vec2.MulSV(d.y, normal, P2);
                        b2Math_1.B2Vec2.AddVV(P1, P2, P1P2);
                        // vA -= mA * (P1 + P2);
                        vA.SelfMulSub(mA, P1P2);
                        // wA -= iA * (B2Cross(cp1->rA, P1) + B2Cross(cp2->rA, P2));
                        wA -= iA * (b2Math_1.B2Vec2.CrossVV(cp1.rA, P1) + b2Math_1.B2Vec2.CrossVV(cp2.rA, P2));
                        // vB += mB * (P1 + P2);
                        vB.SelfMulAdd(mB, P1P2);
                        // wB += iB * (B2Cross(cp1->rB, P1) + B2Cross(cp2->rB, P2));
                        wB += iB * (b2Math_1.B2Vec2.CrossVV(cp1.rB, P1) + b2Math_1.B2Vec2.CrossVV(cp2.rB, P2));
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
                    x.y = -cp2.normalMass * b.y;
                    vn1 = vc.K.ey.x * x.y + b.x;
                    vn2 = 0;
                    if (x.y >= 0 && vn1 >= 0) {
                        // Resubstitute for the incremental impulse
                        // B2Vec2 d = x - a;
                        b2Math_1.B2Vec2.SubVV(x, a, d);
                        // Apply incremental impulse
                        // B2Vec2 P1 = d.x * normal;
                        b2Math_1.B2Vec2.MulSV(d.x, normal, P1);
                        // B2Vec2 P2 = d.y * normal;
                        b2Math_1.B2Vec2.MulSV(d.y, normal, P2);
                        b2Math_1.B2Vec2.AddVV(P1, P2, P1P2);
                        // vA -= mA * (P1 + P2);
                        vA.SelfMulSub(mA, P1P2);
                        // wA -= iA * (B2Cross(cp1->rA, P1) + B2Cross(cp2->rA, P2));
                        wA -= iA * (b2Math_1.B2Vec2.CrossVV(cp1.rA, P1) + b2Math_1.B2Vec2.CrossVV(cp2.rA, P2));
                        // vB += mB * (P1 + P2);
                        vB.SelfMulAdd(mB, P1P2);
                        // wB += iB * (B2Cross(cp1->rB, P1) + B2Cross(cp2->rB, P2));
                        wB += iB * (b2Math_1.B2Vec2.CrossVV(cp1.rB, P1) + b2Math_1.B2Vec2.CrossVV(cp2.rB, P2));
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
                        b2Math_1.B2Vec2.SubVV(x, a, d);
                        // Apply incremental impulse
                        // B2Vec2 P1 = d.x * normal;
                        b2Math_1.B2Vec2.MulSV(d.x, normal, P1);
                        // B2Vec2 P2 = d.y * normal;
                        b2Math_1.B2Vec2.MulSV(d.y, normal, P2);
                        b2Math_1.B2Vec2.AddVV(P1, P2, P1P2);
                        // vA -= mA * (P1 + P2);
                        vA.SelfMulSub(mA, P1P2);
                        // wA -= iA * (B2Cross(cp1->rA, P1) + B2Cross(cp2->rA, P2));
                        wA -= iA * (b2Math_1.B2Vec2.CrossVV(cp1.rA, P1) + b2Math_1.B2Vec2.CrossVV(cp2.rA, P2));
                        // vB += mB * (P1 + P2);
                        vB.SelfMulAdd(mB, P1P2);
                        // wB += iB * (B2Cross(cp1->rB, P1) + B2Cross(cp2->rB, P2));
                        wB += iB * (b2Math_1.B2Vec2.CrossVV(cp1.rB, P1) + b2Math_1.B2Vec2.CrossVV(cp2.rB, P2));
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
    StoreImpulses() {
        for (let i = 0; i < this.m_count; ++i) {
            let vc = this.m_velocityConstraints[i];
            let manifold = this.m_contacts[vc.contactIndex].GetManifold();
            for (let j = 0; j < vc.pointCount; ++j) {
                manifold.points[j].normalImpulse = vc.points[j].normalImpulse;
                manifold.points[j].tangentImpulse = vc.points[j].tangentImpulse;
            }
        }
    }
    SolvePositionConstraints() {
        const xfA = B2ContactSolver.SolvePositionConstraints_s_xfA;
        const xfB = B2ContactSolver.SolvePositionConstraints_s_xfB;
        const psm = B2ContactSolver.SolvePositionConstraints_s_psm;
        const rA = B2ContactSolver.SolvePositionConstraints_s_rA;
        const rB = B2ContactSolver.SolvePositionConstraints_s_rB;
        const P = B2ContactSolver.SolvePositionConstraints_s_P;
        let minSeparation = 0;
        for (let i = 0; i < this.m_count; ++i) {
            const pc = this.m_positionConstraints[i];
            const indexA = pc.indexA;
            const indexB = pc.indexB;
            const localCenterA = pc.localCenterA;
            const mA = pc.invMassA;
            const iA = pc.invIA;
            const localCenterB = pc.localCenterB;
            const mB = pc.invMassB;
            const iB = pc.invIB;
            const pointCount = pc.pointCount;
            const cA = this.m_positions[indexA].c;
            let aA = this.m_positions[indexA].a;
            const cB = this.m_positions[indexB].c;
            let aB = this.m_positions[indexB].a;
            // Solve normal constraints
            for (let j = 0; j < pointCount; ++j) {
                xfA.q.SetAngle(aA);
                xfB.q.SetAngle(aB);
                b2Math_1.B2Vec2.SubVV(cA, b2Math_1.B2Rot.MulRV(xfA.q, localCenterA, b2Math_1.B2Vec2.s_t0), xfA.p);
                b2Math_1.B2Vec2.SubVV(cB, b2Math_1.B2Rot.MulRV(xfB.q, localCenterB, b2Math_1.B2Vec2.s_t0), xfB.p);
                psm.Initialize(pc, xfA, xfB, j);
                const normal = psm.normal;
                const point = psm.point;
                const separation = psm.separation;
                // B2Vec2 rA = point - cA;
                b2Math_1.B2Vec2.SubVV(point, cA, rA);
                // B2Vec2 rB = point - cB;
                b2Math_1.B2Vec2.SubVV(point, cB, rB);
                // Track max constraint error.
                minSeparation = b2Math_1.B2Min(minSeparation, separation);
                // Prevent large corrections and allow slop.
                const C = b2Math_1.B2Clamp(b2Settings_1.B2_baumgarte * (separation + b2Settings_1.B2_linearSlop), -b2Settings_1.B2_maxLinearCorrection, 0);
                // Compute the effective mass.
                // float32 rnA = B2Cross(rA, normal);
                const rnA = b2Math_1.B2Vec2.CrossVV(rA, normal);
                // float32 rnB = B2Cross(rB, normal);
                const rnB = b2Math_1.B2Vec2.CrossVV(rB, normal);
                // float32 K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                const K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                // Compute normal impulse
                const impulse = K > 0 ? -C / K : 0;
                // B2Vec2 P = impulse * normal;
                b2Math_1.B2Vec2.MulSV(impulse, normal, P);
                // cA -= mA * P;
                cA.SelfMulSub(mA, P);
                // aA -= iA * B2Cross(rA, P);
                aA -= iA * b2Math_1.B2Vec2.CrossVV(rA, P);
                // cB += mB * P;
                cB.SelfMulAdd(mB, P);
                // aB += iB * B2Cross(rB, P);
                aB += iB * b2Math_1.B2Vec2.CrossVV(rB, P);
            }
            // this.m_positions[indexA].c = cA;
            this.m_positions[indexA].a = aA;
            // this.m_positions[indexB].c = cB;
            this.m_positions[indexB].a = aB;
        }
        // We can't expect minSpeparation >= -B2_linearSlop because we don't
        // push the separation above -B2_linearSlop.
        return minSeparation > -3 * b2Settings_1.B2_linearSlop;
    }
    SolveTOIPositionConstraints(toiIndexA, toiIndexB) {
        const xfA = B2ContactSolver.SolveTOIPositionConstraints_s_xfA;
        const xfB = B2ContactSolver.SolveTOIPositionConstraints_s_xfB;
        const psm = B2ContactSolver.SolveTOIPositionConstraints_s_psm;
        const rA = B2ContactSolver.SolveTOIPositionConstraints_s_rA;
        const rB = B2ContactSolver.SolveTOIPositionConstraints_s_rB;
        const P = B2ContactSolver.SolveTOIPositionConstraints_s_P;
        let minSeparation = 0;
        for (let i = 0; i < this.m_count; ++i) {
            const pc = this.m_positionConstraints[i];
            const indexA = pc.indexA;
            const indexB = pc.indexB;
            const localCenterA = pc.localCenterA;
            const localCenterB = pc.localCenterB;
            const pointCount = pc.pointCount;
            let mA = 0;
            let iA = 0;
            if (indexA === toiIndexA || indexA === toiIndexB) {
                mA = pc.invMassA;
                iA = pc.invIA;
            }
            let mB = 0;
            let iB = 0;
            if (indexB === toiIndexA || indexB === toiIndexB) {
                mB = pc.invMassB;
                iB = pc.invIB;
            }
            const cA = this.m_positions[indexA].c;
            let aA = this.m_positions[indexA].a;
            const cB = this.m_positions[indexB].c;
            let aB = this.m_positions[indexB].a;
            // Solve normal constraints
            for (let j = 0; j < pointCount; ++j) {
                xfA.q.SetAngle(aA);
                xfB.q.SetAngle(aB);
                b2Math_1.B2Vec2.SubVV(cA, b2Math_1.B2Rot.MulRV(xfA.q, localCenterA, b2Math_1.B2Vec2.s_t0), xfA.p);
                b2Math_1.B2Vec2.SubVV(cB, b2Math_1.B2Rot.MulRV(xfB.q, localCenterB, b2Math_1.B2Vec2.s_t0), xfB.p);
                psm.Initialize(pc, xfA, xfB, j);
                const normal = psm.normal;
                const point = psm.point;
                const separation = psm.separation;
                // B2Vec2 rA = point - cA;
                b2Math_1.B2Vec2.SubVV(point, cA, rA);
                // B2Vec2 rB = point - cB;
                b2Math_1.B2Vec2.SubVV(point, cB, rB);
                // Track max constraint error.
                minSeparation = b2Math_1.B2Min(minSeparation, separation);
                // Prevent large corrections and allow slop.
                const C = b2Math_1.B2Clamp(b2Settings_1.B2_toiBaumgarte * (separation + b2Settings_1.B2_linearSlop), -b2Settings_1.B2_maxLinearCorrection, 0);
                // Compute the effective mass.
                // float32 rnA = B2Cross(rA, normal);
                const rnA = b2Math_1.B2Vec2.CrossVV(rA, normal);
                // float32 rnB = B2Cross(rB, normal);
                const rnB = b2Math_1.B2Vec2.CrossVV(rB, normal);
                // float32 K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                const K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                // Compute normal impulse
                const impulse = K > 0 ? -C / K : 0;
                // B2Vec2 P = impulse * normal;
                b2Math_1.B2Vec2.MulSV(impulse, normal, P);
                // cA -= mA * P;
                cA.SelfMulSub(mA, P);
                // aA -= iA * B2Cross(rA, P);
                aA -= iA * b2Math_1.B2Vec2.CrossVV(rA, P);
                // cB += mB * P;
                cB.SelfMulAdd(mB, P);
                // aB += iB * B2Cross(rB, P);
                aB += iB * b2Math_1.B2Vec2.CrossVV(rB, P);
            }
            // this.m_positions[indexA].c = cA;
            this.m_positions[indexA].a = aA;
            // this.m_positions[indexB].c = cB;
            this.m_positions[indexB].a = aB;
        }
        // We can't expect minSpeparation >= -B2_linearSlop because we don't
        // push the separation above -B2_linearSlop.
        return minSeparation >= -1.5 * b2Settings_1.B2_linearSlop;
    }
}
exports.B2ContactSolver = B2ContactSolver;
B2ContactSolver.InitializeVelocityConstraints_s_xfA = new b2Math_1.B2Transform();
B2ContactSolver.InitializeVelocityConstraints_s_xfB = new b2Math_1.B2Transform();
B2ContactSolver.InitializeVelocityConstraints_s_worldManifold = new b2Collision_1.B2WorldManifold();
B2ContactSolver.WarmStart_s_P = new b2Math_1.B2Vec2();
B2ContactSolver.SolveVelocityConstraints_s_dv = new b2Math_1.B2Vec2();
B2ContactSolver.SolveVelocityConstraints_s_dv1 = new b2Math_1.B2Vec2();
B2ContactSolver.SolveVelocityConstraints_s_dv2 = new b2Math_1.B2Vec2();
B2ContactSolver.SolveVelocityConstraints_s_P = new b2Math_1.B2Vec2();
B2ContactSolver.SolveVelocityConstraints_s_a = new b2Math_1.B2Vec2();
B2ContactSolver.SolveVelocityConstraints_s_b = new b2Math_1.B2Vec2();
B2ContactSolver.SolveVelocityConstraints_s_x = new b2Math_1.B2Vec2();
B2ContactSolver.SolveVelocityConstraints_s_d = new b2Math_1.B2Vec2();
B2ContactSolver.SolveVelocityConstraints_s_P1 = new b2Math_1.B2Vec2();
B2ContactSolver.SolveVelocityConstraints_s_P2 = new b2Math_1.B2Vec2();
B2ContactSolver.SolveVelocityConstraints_s_P1P2 = new b2Math_1.B2Vec2();
B2ContactSolver.SolvePositionConstraints_s_xfA = new b2Math_1.B2Transform();
B2ContactSolver.SolvePositionConstraints_s_xfB = new b2Math_1.B2Transform();
B2ContactSolver.SolvePositionConstraints_s_psm = new B2PositionSolverManifold();
B2ContactSolver.SolvePositionConstraints_s_rA = new b2Math_1.B2Vec2();
B2ContactSolver.SolvePositionConstraints_s_rB = new b2Math_1.B2Vec2();
B2ContactSolver.SolvePositionConstraints_s_P = new b2Math_1.B2Vec2();
B2ContactSolver.SolveTOIPositionConstraints_s_xfA = new b2Math_1.B2Transform();
B2ContactSolver.SolveTOIPositionConstraints_s_xfB = new b2Math_1.B2Transform();
B2ContactSolver.SolveTOIPositionConstraints_s_psm = new B2PositionSolverManifold();
B2ContactSolver.SolveTOIPositionConstraints_s_rA = new b2Math_1.B2Vec2();
B2ContactSolver.SolveTOIPositionConstraints_s_rB = new b2Math_1.B2Vec2();
B2ContactSolver.SolveTOIPositionConstraints_s_P = new b2Math_1.B2Vec2();
