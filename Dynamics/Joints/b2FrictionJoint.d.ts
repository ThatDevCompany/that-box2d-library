import { B2Vec2, B2Mat22, B2Rot } from '../../Common/b2Math';
import { B2Joint, B2JointDef } from './b2Joint';
import { B2SolverData } from '../b2TimeStep';
import { B2Body } from '../b2Body';
export declare class B2FrictionJointDef extends B2JointDef {
    localAnchorA: B2Vec2;
    localAnchorB: B2Vec2;
    maxForce: number;
    maxTorque: number;
    constructor();
    Initialize(bA: B2Body, bB: B2Body, anchor: B2Vec2): void;
}
export declare class B2FrictionJoint extends B2Joint {
    m_localAnchorA: B2Vec2;
    m_localAnchorB: B2Vec2;
    m_linearImpulse: B2Vec2;
    m_angularImpulse: number;
    m_maxForce: number;
    m_maxTorque: number;
    m_indexA: number;
    m_indexB: number;
    m_rA: B2Vec2;
    m_rB: B2Vec2;
    m_localCenterA: B2Vec2;
    m_localCenterB: B2Vec2;
    m_invMassA: number;
    m_invMassB: number;
    m_invIA: number;
    m_invIB: number;
    m_linearMass: B2Mat22;
    m_angularMass: number;
    m_qA: B2Rot;
    m_qB: B2Rot;
    m_lalcA: B2Vec2;
    m_lalcB: B2Vec2;
    m_K: B2Mat22;
    constructor(def: B2FrictionJointDef);
    InitVelocityConstraints(data: B2SolverData): void;
    private static SolveVelocityConstraints_s_Cdot_v2;
    private static SolveVelocityConstraints_s_impulseV;
    private static SolveVelocityConstraints_s_oldImpulseV;
    SolveVelocityConstraints(data: B2SolverData): void;
    SolvePositionConstraints(data: B2SolverData): boolean;
    GetAnchorA(out: B2Vec2): B2Vec2;
    GetAnchorB(out: B2Vec2): B2Vec2;
    GetReactionForce(inv_dt: number, out: B2Vec2): B2Vec2;
    GetReactionTorque(inv_dt: number): number;
    GetLocalAnchorA(): B2Vec2;
    GetLocalAnchorB(): B2Vec2;
    SetMaxForce(force: number): void;
    GetMaxForce(): number;
    SetMaxTorque(torque: number): void;
    GetMaxTorque(): number;
    Dump(log: (format: string, ...args: any[]) => void): void;
}
