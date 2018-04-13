import { B2Vec2, B2Mat22, B2Transform } from '../../Common/b2Math';
import { B2ManifoldType } from '../../Collision/b2Collision';
import { B2Contact } from './b2Contact';
import { B2TimeStep, B2Position, B2Velocity } from '../b2TimeStep';
export declare class B2VelocityConstraintPoint {
    rA: B2Vec2;
    rB: B2Vec2;
    normalImpulse: number;
    tangentImpulse: number;
    normalMass: number;
    tangentMass: number;
    velocityBias: number;
    static MakeArray(length: number): B2VelocityConstraintPoint[];
}
export declare class B2ContactVelocityConstraint {
    points: B2VelocityConstraintPoint[];
    normal: B2Vec2;
    tangent: B2Vec2;
    normalMass: B2Mat22;
    K: B2Mat22;
    indexA: number;
    indexB: number;
    invMassA: number;
    invMassB: number;
    invIA: number;
    invIB: number;
    friction: number;
    restitution: number;
    tangentSpeed: number;
    pointCount: number;
    contactIndex: number;
    static MakeArray(length: number): B2ContactVelocityConstraint[];
}
export declare class B2ContactPositionConstraint {
    localPoints: B2Vec2[];
    localNormal: B2Vec2;
    localPoint: B2Vec2;
    indexA: number;
    indexB: number;
    invMassA: number;
    invMassB: number;
    localCenterA: B2Vec2;
    localCenterB: B2Vec2;
    invIA: number;
    invIB: number;
    type: B2ManifoldType;
    radiusA: number;
    radiusB: number;
    pointCount: number;
    static MakeArray(length: number): B2ContactPositionConstraint[];
}
export declare class B2ContactSolverDef {
    step: B2TimeStep;
    contacts: B2Contact[];
    count: number;
    positions: B2Position[];
    velocities: B2Velocity[];
    allocator: any;
}
export declare class B2PositionSolverManifold {
    normal: B2Vec2;
    point: B2Vec2;
    separation: number;
    private static Initialize_s_pointA;
    private static Initialize_s_pointB;
    private static Initialize_s_planePoint;
    private static Initialize_s_clipPoint;
    Initialize(pc: B2ContactPositionConstraint, xfA: B2Transform, xfB: B2Transform, index: number): void;
}
export declare class B2ContactSolver {
    m_step: B2TimeStep;
    m_positions: B2Position[];
    m_velocities: B2Velocity[];
    m_allocator: any;
    m_positionConstraints: B2ContactPositionConstraint[];
    m_velocityConstraints: B2ContactVelocityConstraint[];
    m_contacts: B2Contact[];
    m_count: number;
    Initialize(def: B2ContactSolverDef): B2ContactSolver;
    private static InitializeVelocityConstraints_s_xfA;
    private static InitializeVelocityConstraints_s_xfB;
    private static InitializeVelocityConstraints_s_worldManifold;
    InitializeVelocityConstraints(): void;
    private static WarmStart_s_P;
    WarmStart(): void;
    private static SolveVelocityConstraints_s_dv;
    private static SolveVelocityConstraints_s_dv1;
    private static SolveVelocityConstraints_s_dv2;
    private static SolveVelocityConstraints_s_P;
    private static SolveVelocityConstraints_s_a;
    private static SolveVelocityConstraints_s_b;
    private static SolveVelocityConstraints_s_x;
    private static SolveVelocityConstraints_s_d;
    private static SolveVelocityConstraints_s_P1;
    private static SolveVelocityConstraints_s_P2;
    private static SolveVelocityConstraints_s_P1P2;
    SolveVelocityConstraints(): void;
    StoreImpulses(): void;
    private static SolvePositionConstraints_s_xfA;
    private static SolvePositionConstraints_s_xfB;
    private static SolvePositionConstraints_s_psm;
    private static SolvePositionConstraints_s_rA;
    private static SolvePositionConstraints_s_rB;
    private static SolvePositionConstraints_s_P;
    SolvePositionConstraints(): boolean;
    private static SolveTOIPositionConstraints_s_xfA;
    private static SolveTOIPositionConstraints_s_xfB;
    private static SolveTOIPositionConstraints_s_psm;
    private static SolveTOIPositionConstraints_s_rA;
    private static SolveTOIPositionConstraints_s_rB;
    private static SolveTOIPositionConstraints_s_P;
    SolveTOIPositionConstraints(toiIndexA: number, toiIndexB: number): boolean;
}
