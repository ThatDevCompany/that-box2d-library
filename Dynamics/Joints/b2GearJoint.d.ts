import { B2Vec2, B2Rot } from '../../Common/b2Math';
import { B2Joint, B2JointDef, B2JointType } from './b2Joint';
import { B2SolverData } from '../b2TimeStep';
import { B2Body } from '../b2Body';
export declare class B2GearJointDef extends B2JointDef {
    joint1: B2Joint;
    joint2: B2Joint;
    ratio: number;
    constructor();
}
export declare class B2GearJoint extends B2Joint {
    m_joint1: B2Joint;
    m_joint2: B2Joint;
    m_typeA: B2JointType;
    m_typeB: B2JointType;
    m_bodyC: B2Body;
    m_bodyD: B2Body;
    m_localAnchorA: B2Vec2;
    m_localAnchorB: B2Vec2;
    m_localAnchorC: B2Vec2;
    m_localAnchorD: B2Vec2;
    m_localAxisC: B2Vec2;
    m_localAxisD: B2Vec2;
    m_referenceAngleA: number;
    m_referenceAngleB: number;
    m_constant: number;
    m_ratio: number;
    m_impulse: number;
    m_indexA: number;
    m_indexB: number;
    m_indexC: number;
    m_indexD: number;
    m_lcA: B2Vec2;
    m_lcB: B2Vec2;
    m_lcC: B2Vec2;
    m_lcD: B2Vec2;
    m_mA: number;
    m_mB: number;
    m_mC: number;
    m_mD: number;
    m_iA: number;
    m_iB: number;
    m_iC: number;
    m_iD: number;
    m_JvAC: B2Vec2;
    m_JvBD: B2Vec2;
    m_JwA: number;
    m_JwB: number;
    m_JwC: number;
    m_JwD: number;
    m_mass: number;
    m_qA: B2Rot;
    m_qB: B2Rot;
    m_qC: B2Rot;
    m_qD: B2Rot;
    m_lalcA: B2Vec2;
    m_lalcB: B2Vec2;
    m_lalcC: B2Vec2;
    m_lalcD: B2Vec2;
    constructor(def: B2GearJointDef);
    private static InitVelocityConstraints_s_u;
    private static InitVelocityConstraints_s_rA;
    private static InitVelocityConstraints_s_rB;
    private static InitVelocityConstraints_s_rC;
    private static InitVelocityConstraints_s_rD;
    InitVelocityConstraints(data: B2SolverData): void;
    SolveVelocityConstraints(data: B2SolverData): void;
    private static SolvePositionConstraints_s_u;
    private static SolvePositionConstraints_s_rA;
    private static SolvePositionConstraints_s_rB;
    private static SolvePositionConstraints_s_rC;
    private static SolvePositionConstraints_s_rD;
    SolvePositionConstraints(data: B2SolverData): boolean;
    GetAnchorA(out: B2Vec2): B2Vec2;
    GetAnchorB(out: B2Vec2): B2Vec2;
    GetReactionForce(inv_dt: number, out: B2Vec2): B2Vec2;
    GetReactionTorque(inv_dt: number): number;
    GetJoint1(): B2Joint;
    GetJoint2(): B2Joint;
    GetRatio(): number;
    SetRatio(ratio: number): void;
    Dump(log: (format: string, ...args: any[]) => void): void;
}
