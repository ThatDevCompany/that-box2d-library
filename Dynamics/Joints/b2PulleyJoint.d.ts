import { B2Vec2, B2Rot } from '../../Common/b2Math';
import { B2Body } from '../b2Body';
import { B2Joint, B2JointDef } from './b2Joint';
import { B2SolverData } from '../b2TimeStep';
export declare const B2_minPulleyLength: number;
export declare class B2PulleyJointDef extends B2JointDef {
    groundAnchorA: B2Vec2;
    groundAnchorB: B2Vec2;
    localAnchorA: B2Vec2;
    localAnchorB: B2Vec2;
    lengthA: number;
    lengthB: number;
    ratio: number;
    constructor();
    Initialize(bA: B2Body, bB: B2Body, groundA: B2Vec2, groundB: B2Vec2, anchorA: B2Vec2, anchorB: B2Vec2, r: number): void;
}
export declare class B2PulleyJoint extends B2Joint {
    m_groundAnchorA: B2Vec2;
    m_groundAnchorB: B2Vec2;
    m_lengthA: number;
    m_lengthB: number;
    m_localAnchorA: B2Vec2;
    m_localAnchorB: B2Vec2;
    m_constant: number;
    m_ratio: number;
    m_impulse: number;
    m_indexA: number;
    m_indexB: number;
    m_uA: B2Vec2;
    m_uB: B2Vec2;
    m_rA: B2Vec2;
    m_rB: B2Vec2;
    m_localCenterA: B2Vec2;
    m_localCenterB: B2Vec2;
    m_invMassA: number;
    m_invMassB: number;
    m_invIA: number;
    m_invIB: number;
    m_mass: number;
    m_qA: B2Rot;
    m_qB: B2Rot;
    m_lalcA: B2Vec2;
    m_lalcB: B2Vec2;
    constructor(def: B2PulleyJointDef);
    private static InitVelocityConstraints_s_PA;
    private static InitVelocityConstraints_s_PB;
    InitVelocityConstraints(data: B2SolverData): void;
    private static SolveVelocityConstraints_s_vpA;
    private static SolveVelocityConstraints_s_vpB;
    private static SolveVelocityConstraints_s_PA;
    private static SolveVelocityConstraints_s_PB;
    SolveVelocityConstraints(data: B2SolverData): void;
    private static SolvePositionConstraints_s_PA;
    private static SolvePositionConstraints_s_PB;
    SolvePositionConstraints(data: B2SolverData): boolean;
    GetAnchorA(out: B2Vec2): B2Vec2;
    GetAnchorB(out: B2Vec2): B2Vec2;
    GetReactionForce(inv_dt: number, out: B2Vec2): B2Vec2;
    GetReactionTorque(inv_dt: number): number;
    GetGroundAnchorA(): B2Vec2;
    GetGroundAnchorB(): B2Vec2;
    GetLengthA(): number;
    GetLengthB(): number;
    GetRatio(): number;
    private static GetCurrentLengthA_s_p;
    GetCurrentLengthA(): number;
    private static GetCurrentLengthB_s_p;
    GetCurrentLengthB(): number;
    Dump(log: (format: string, ...args: any[]) => void): void;
    ShiftOrigin(newOrigin: B2Vec2): void;
}