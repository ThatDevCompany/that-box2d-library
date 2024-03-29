import { B2Vec2, B2Mat22, B2Vec3, B2Mat33, B2Rot } from '../../Common/b2Math';
import { B2Body } from '../b2Body';
import { B2Joint, B2JointDef, B2LimitState } from './b2Joint';
import { B2SolverData } from '../b2TimeStep';
export declare class B2PrismaticJointDef extends B2JointDef {
    localAnchorA: B2Vec2;
    localAnchorB: B2Vec2;
    localAxisA: B2Vec2;
    referenceAngle: number;
    enableLimit: boolean;
    lowerTranslation: number;
    upperTranslation: number;
    enableMotor: boolean;
    maxMotorForce: number;
    motorSpeed: number;
    constructor();
    Initialize(bA: B2Body, bB: B2Body, anchor: B2Vec2, axis: B2Vec2): void;
}
export declare class B2PrismaticJoint extends B2Joint {
    m_localAnchorA: B2Vec2;
    m_localAnchorB: B2Vec2;
    m_localXAxisA: B2Vec2;
    m_localYAxisA: B2Vec2;
    m_referenceAngle: number;
    m_impulse: B2Vec3;
    m_motorImpulse: number;
    m_lowerTranslation: number;
    m_upperTranslation: number;
    m_maxMotorForce: number;
    m_motorSpeed: number;
    m_enableLimit: boolean;
    m_enableMotor: boolean;
    m_limitState: B2LimitState;
    m_indexA: number;
    m_indexB: number;
    m_localCenterA: B2Vec2;
    m_localCenterB: B2Vec2;
    m_invMassA: number;
    m_invMassB: number;
    m_invIA: number;
    m_invIB: number;
    m_axis: B2Vec2;
    m_perp: B2Vec2;
    m_s1: number;
    m_s2: number;
    m_a1: number;
    m_a2: number;
    m_K: B2Mat33;
    m_K3: B2Mat33;
    m_K2: B2Mat22;
    m_motorMass: number;
    m_qA: B2Rot;
    m_qB: B2Rot;
    m_lalcA: B2Vec2;
    m_lalcB: B2Vec2;
    m_rA: B2Vec2;
    m_rB: B2Vec2;
    constructor(def: B2PrismaticJointDef);
    private static InitVelocityConstraints_s_d;
    private static InitVelocityConstraints_s_P;
    InitVelocityConstraints(data: B2SolverData): void;
    private static SolveVelocityConstraints_s_P;
    private static SolveVelocityConstraints_s_f2r;
    private static SolveVelocityConstraints_s_f1;
    private static SolveVelocityConstraints_s_df3;
    private static SolveVelocityConstraints_s_df2;
    SolveVelocityConstraints(data: B2SolverData): void;
    private static SolvePositionConstraints_s_d;
    private static SolvePositionConstraints_s_impulse;
    private static SolvePositionConstraints_s_impulse1;
    private static SolvePositionConstraints_s_P;
    SolvePositionConstraints(data: B2SolverData): boolean;
    GetAnchorA(out: B2Vec2): B2Vec2;
    GetAnchorB(out: B2Vec2): B2Vec2;
    GetReactionForce(inv_dt: number, out: B2Vec2): B2Vec2;
    GetReactionTorque(inv_dt: number): number;
    GetLocalAnchorA(): B2Vec2;
    GetLocalAnchorB(): B2Vec2;
    GetLocalAxisA(): B2Vec2;
    GetReferenceAngle(): number;
    private static GetJointTranslation_s_pA;
    private static GetJointTranslation_s_pB;
    private static GetJointTranslation_s_d;
    private static GetJointTranslation_s_axis;
    GetJointTranslation(): number;
    GetJointSpeed(): number;
    IsLimitEnabled(): boolean;
    EnableLimit(flag: boolean): void;
    GetLowerLimit(): number;
    GetUpperLimit(): number;
    SetLimits(lower: number, upper: number): void;
    IsMotorEnabled(): boolean;
    EnableMotor(flag: boolean): void;
    SetMotorSpeed(speed: number): void;
    GetMotorSpeed(): number;
    SetMaxMotorForce(force: number): void;
    GetMaxMotorForce(): number;
    GetMotorForce(inv_dt: number): number;
    Dump(log: (format: string, ...args: any[]) => void): void;
}
