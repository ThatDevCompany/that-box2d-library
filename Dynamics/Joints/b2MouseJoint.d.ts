import { B2Vec2, B2Mat22, B2Rot } from '../../Common/b2Math';
import { B2Joint, B2JointDef } from './b2Joint';
import { B2SolverData } from '../b2TimeStep';
export declare class B2MouseJointDef extends B2JointDef {
    target: B2Vec2;
    maxForce: number;
    frequencyHz: number;
    dampingRatio: number;
    constructor();
}
export declare class B2MouseJoint extends B2Joint {
    m_localAnchorB: B2Vec2;
    m_targetA: B2Vec2;
    m_frequencyHz: number;
    m_dampingRatio: number;
    m_beta: number;
    m_impulse: B2Vec2;
    m_maxForce: number;
    m_gamma: number;
    m_indexA: number;
    m_indexB: number;
    m_rB: B2Vec2;
    m_localCenterB: B2Vec2;
    m_invMassB: number;
    m_invIB: number;
    m_mass: B2Mat22;
    m_C: B2Vec2;
    m_qB: B2Rot;
    m_lalcB: B2Vec2;
    m_K: B2Mat22;
    constructor(def: B2MouseJointDef);
    SetTarget(target: B2Vec2): void;
    GetTarget(): B2Vec2;
    SetMaxForce(maxForce: number): void;
    GetMaxForce(): number;
    SetFrequency(hz: number): void;
    GetFrequency(): number;
    SetDampingRatio(ratio: number): void;
    GetDampingRatio(): number;
    InitVelocityConstraints(data: B2SolverData): void;
    private static SolveVelocityConstraints_s_Cdot;
    private static SolveVelocityConstraints_s_impulse;
    private static SolveVelocityConstraints_s_oldImpulse;
    SolveVelocityConstraints(data: B2SolverData): void;
    SolvePositionConstraints(data: B2SolverData): boolean;
    GetAnchorA(out: B2Vec2): B2Vec2;
    GetAnchorB(out: B2Vec2): B2Vec2;
    GetReactionForce(inv_dt: number, out: B2Vec2): B2Vec2;
    GetReactionTorque(inv_dt: number): number;
    Dump(log: (format: string, ...args: any[]) => void): void;
    ShiftOrigin(newOrigin: B2Vec2): void;
}