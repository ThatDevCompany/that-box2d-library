"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.B2AreaJoint = exports.B2AreaJointDef = void 0;
const b2Settings_1 = require("../../Common/b2Settings");
const b2Math_1 = require("../../Common/b2Math");
const b2Joint_1 = require("./b2Joint");
const b2DistanceJoint_1 = require("./b2DistanceJoint");
class B2AreaJointDef extends b2Joint_1.B2JointDef {
    constructor() {
        super(12 /* e_areaJoint */);
        this.world = null;
        this.bodies = [];
        this.frequencyHz = 0;
        this.dampingRatio = 0;
    }
    AddBody(body) {
        this.bodies.push(body);
        if (this.bodies.length === 1) {
            this.bodyA = body;
        }
        else if (this.bodies.length === 2) {
            this.bodyB = body;
        }
    }
}
exports.B2AreaJointDef = B2AreaJointDef;
class B2AreaJoint extends b2Joint_1.B2Joint {
    constructor(def) {
        super(def);
        this.m_bodies = null;
        this.m_frequencyHz = 0;
        this.m_dampingRatio = 0;
        // Solver shared
        this.m_impulse = 0;
        // Solver temp
        this.m_targetLengths = null;
        this.m_targetArea = 0;
        this.m_normals = null;
        this.m_joints = null;
        this.m_deltas = null;
        this.m_delta = null;
        /// b2Assert(def.bodies.length >= 3, 'You cannot create an area joint with less than three bodies.');
        this.m_bodies = def.bodies;
        this.m_frequencyHz = def.frequencyHz;
        this.m_dampingRatio = def.dampingRatio;
        this.m_targetLengths = b2Settings_1.B2MakeNumberArray(def.bodies.length);
        this.m_normals = b2Math_1.B2Vec2.MakeArray(def.bodies.length);
        this.m_joints = b2Settings_1.B2MakeNullArray(def.bodies.length);
        this.m_deltas = b2Math_1.B2Vec2.MakeArray(def.bodies.length);
        this.m_delta = new b2Math_1.B2Vec2();
        const djd = new b2DistanceJoint_1.B2DistanceJointDef();
        djd.frequencyHz = def.frequencyHz;
        djd.dampingRatio = def.dampingRatio;
        this.m_targetArea = 0;
        for (let i = 0; i < this.m_bodies.length; ++i) {
            const body = this.m_bodies[i];
            const next = this.m_bodies[(i + 1) % this.m_bodies.length];
            const body_c = body.GetWorldCenter();
            const next_c = next.GetWorldCenter();
            this.m_targetLengths[i] = b2Math_1.B2Vec2.DistanceVV(body_c, next_c);
            this.m_targetArea += b2Math_1.B2Vec2.CrossVV(body_c, next_c);
            djd.Initialize(body, next, body_c, next_c);
            this.m_joints[i] = def.world.CreateJoint(djd);
        }
        this.m_targetArea *= 0.5;
    }
    GetAnchorA(out) {
        return out.SetZero();
    }
    GetAnchorB(out) {
        return out.SetZero();
    }
    GetReactionForce(inv_dt, out) {
        return out.SetZero();
    }
    GetReactionTorque(inv_dt) {
        return 0;
    }
    SetFrequency(hz) {
        this.m_frequencyHz = hz;
        for (let i = 0; i < this.m_joints.length; ++i) {
            this.m_joints[i].SetFrequency(hz);
        }
    }
    GetFrequency() {
        return this.m_frequencyHz;
    }
    SetDampingRatio(ratio) {
        this.m_dampingRatio = ratio;
        for (let i = 0; i < this.m_joints.length; ++i) {
            this.m_joints[i].SetDampingRatio(ratio);
        }
    }
    GetDampingRatio() {
        return this.m_dampingRatio;
    }
    Dump(log) {
        log('Area joint dumping is not supported.\n');
    }
    InitVelocityConstraints(data) {
        for (let i = 0; i < this.m_bodies.length; ++i) {
            const prev = this.m_bodies[(i + this.m_bodies.length - 1) % this.m_bodies.length];
            const next = this.m_bodies[(i + 1) % this.m_bodies.length];
            const prev_c = data.positions[prev.m_islandIndex].c;
            const next_c = data.positions[next.m_islandIndex].c;
            const delta = this.m_deltas[i];
            b2Math_1.B2Vec2.SubVV(next_c, prev_c, delta);
        }
        if (data.step.warmStarting) {
            this.m_impulse *= data.step.dtRatio;
            for (let i = 0; i < this.m_bodies.length; ++i) {
                const body = this.m_bodies[i];
                const body_v = data.velocities[body.m_islandIndex].v;
                const delta = this.m_deltas[i];
                body_v.x += body.m_invMass * delta.y * 0.5 * this.m_impulse;
                body_v.y += body.m_invMass * -delta.x * 0.5 * this.m_impulse;
            }
        }
        else {
            this.m_impulse = 0;
        }
    }
    SolveVelocityConstraints(data) {
        let dotMassSum = 0;
        let crossMassSum = 0;
        for (let i = 0; i < this.m_bodies.length; ++i) {
            const body = this.m_bodies[i];
            const body_v = data.velocities[body.m_islandIndex].v;
            const delta = this.m_deltas[i];
            dotMassSum += delta.LengthSquared() / body.GetMass();
            crossMassSum += b2Math_1.B2Vec2.CrossVV(body_v, delta);
        }
        const lambda = (-2 * crossMassSum) / dotMassSum;
        // lambda = B2Clamp(lambda, -B2_maxLinearCorrection, B2_maxLinearCorrection);
        this.m_impulse += lambda;
        for (let i = 0; i < this.m_bodies.length; ++i) {
            const body = this.m_bodies[i];
            const body_v = data.velocities[body.m_islandIndex].v;
            const delta = this.m_deltas[i];
            body_v.x += body.m_invMass * delta.y * 0.5 * lambda;
            body_v.y += body.m_invMass * -delta.x * 0.5 * lambda;
        }
    }
    SolvePositionConstraints(data) {
        let perimeter = 0;
        let area = 0;
        for (let i = 0; i < this.m_bodies.length; ++i) {
            const body = this.m_bodies[i];
            const next = this.m_bodies[(i + 1) % this.m_bodies.length];
            const body_c = data.positions[body.m_islandIndex].c;
            const next_c = data.positions[next.m_islandIndex].c;
            const delta = b2Math_1.B2Vec2.SubVV(next_c, body_c, this.m_delta);
            let dist = delta.Length();
            if (dist < b2Settings_1.B2_epsilon) {
                dist = 1;
            }
            this.m_normals[i].x = delta.y / dist;
            this.m_normals[i].y = -delta.x / dist;
            perimeter += dist;
            area += b2Math_1.B2Vec2.CrossVV(body_c, next_c);
        }
        area *= 0.5;
        const deltaArea = this.m_targetArea - area;
        const toExtrude = (0.5 * deltaArea) / perimeter;
        let done = true;
        for (let i = 0; i < this.m_bodies.length; ++i) {
            const body = this.m_bodies[i];
            const body_c = data.positions[body.m_islandIndex].c;
            const next_i = (i + 1) % this.m_bodies.length;
            const delta = b2Math_1.B2Vec2.AddVV(this.m_normals[i], this.m_normals[next_i], this.m_delta);
            delta.SelfMul(toExtrude);
            const norm_sq = delta.LengthSquared();
            if (norm_sq > b2Math_1.B2Sq(b2Settings_1.B2_maxLinearCorrection)) {
                delta.SelfMul(b2Settings_1.B2_maxLinearCorrection / b2Math_1.B2Sqrt(norm_sq));
            }
            if (norm_sq > b2Math_1.B2Sq(b2Settings_1.B2_linearSlop)) {
                done = false;
            }
            body_c.x += delta.x;
            body_c.y += delta.y;
        }
        return done;
    }
}
exports.B2AreaJoint = B2AreaJoint;
