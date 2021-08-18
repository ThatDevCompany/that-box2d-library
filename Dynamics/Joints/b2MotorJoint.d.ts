import { B2Vec2, B2Mat22, B2Rot } from '../../Common/b2Math';
import { B2Body } from '../b2Body';
import { B2Joint, B2JointDef } from './b2Joint';
import { B2SolverData } from '../b2TimeStep';
export declare class B2MotorJointDef extends B2JointDef {
    linearOffset: B2Vec2;
    angularOffset: number;
    maxForce: number;
    maxTorque: number;
    correctionFactor: number;
    constructor();
    Initialize(bA: B2Body, bB: B2Body): void;
}
export declare class B2MotorJoint extends B2Joint {
    m_linearOffset: B2Vec2;
    m_angularOffset: number;
    m_linearImpulse: B2Vec2;
    m_angularImpulse: number;
    m_maxForce: number;
    m_maxTorque: number;
    m_correctionFactor: number;
    m_indexA: number;
    m_indexB: number;
    m_rA: B2Vec2;
    m_rB: B2Vec2;
    m_localCenterA: B2Vec2;
    m_localCenterB: B2Vec2;
    m_linearError: B2Vec2;
    m_angularError: number;
    m_invMassA: number;
    m_invMassB: number;
    m_invIA: number;
    m_invIB: number;
    m_linearMass: B2Mat22;
    m_angularMass: number;
    m_qA: B2Rot;
    m_qB: B2Rot;
    m_K: B2Mat22;
    constructor(def: B2MotorJointDef);
    GetAnchorA(): B2Vec2;
    GetAnchorB(): B2Vec2;
    GetReactionForce(inv_dt: number, out: B2Vec2): B2Vec2;
    GetReactionTorque(inv_dt: number): number;
    SetLinearOffset(linearOffset: B2Vec2): void;
    GetLinearOffset(): B2Vec2;
    SetAngularOffset(angularOffset: number): void;
    GetAngularOffset(): number;
    SetMaxForce(force: number): void;
    GetMaxForce(): number;
    SetMaxTorque(torque: number): void;
    GetMaxTorque(): number;
    InitVelocityConstraints(data: B2SolverData): void;
    private static SolveVelocityConstraints_s_Cdot_v2;
    private static SolveVelocityConstraints_s_impulse_v2;
    private static SolveVelocityConstraints_s_oldImpulse_v2;
    SolveVelocityConstraints(data: B2SolverData): void;
    SolvePositionConstraints(data: B2SolverData): boolean;
    Dump(log: (format: string, ...args: any[]) => void): void;
}