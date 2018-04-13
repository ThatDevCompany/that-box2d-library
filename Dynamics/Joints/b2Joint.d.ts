import { B2Vec2 } from '../../Common/b2Math';
import { B2Body } from '../b2Body';
import { B2SolverData } from '../b2TimeStep';
export declare const enum B2JointType {
    e_unknownJoint = 0,
    e_revoluteJoint = 1,
    e_prismaticJoint = 2,
    e_distanceJoint = 3,
    e_pulleyJoint = 4,
    e_mouseJoint = 5,
    e_gearJoint = 6,
    e_wheelJoint = 7,
    e_weldJoint = 8,
    e_frictionJoint = 9,
    e_ropeJoint = 10,
    e_motorJoint = 11,
    e_areaJoint = 12,
}
export declare const enum B2LimitState {
    e_inactiveLimit = 0,
    e_atLowerLimit = 1,
    e_atUpperLimit = 2,
    e_equalLimits = 3,
}
export declare class B2Jacobian {
    linear: B2Vec2;
    angularA: number;
    angularB: number;
    SetZero(): B2Jacobian;
    Set(x: B2Vec2, a1: number, a2: number): B2Jacobian;
}
export declare class B2JointEdge {
    other: B2Body;
    joint: B2Joint;
    prev: B2JointEdge;
    next: B2JointEdge;
}
export declare class B2JointDef {
    type: B2JointType;
    userData: any;
    bodyA: B2Body;
    bodyB: B2Body;
    collideConnected: boolean;
    constructor(type: B2JointType);
}
export declare class B2Joint {
    m_type: B2JointType;
    m_prev: B2Joint;
    m_next: B2Joint;
    m_edgeA: B2JointEdge;
    m_edgeB: B2JointEdge;
    m_bodyA: B2Body;
    m_bodyB: B2Body;
    m_index: number;
    m_islandFlag: boolean;
    m_collideConnected: boolean;
    m_userData: any;
    constructor(def: B2JointDef);
    GetType(): B2JointType;
    GetBodyA(): B2Body;
    GetBodyB(): B2Body;
    GetAnchorA(out: B2Vec2): B2Vec2;
    GetAnchorB(out: B2Vec2): B2Vec2;
    GetReactionForce(inv_dt: number, out: B2Vec2): B2Vec2;
    GetReactionTorque(inv_dt: number): number;
    GetNext(): B2Joint;
    GetUserData(): any;
    SetUserData(data: any): void;
    IsActive(): boolean;
    GetCollideConnected(): boolean;
    Dump(log: (format: string, ...args: any[]) => void): void;
    ShiftOrigin(newOrigin: B2Vec2): void;
    InitVelocityConstraints(data: B2SolverData): void;
    SolveVelocityConstraints(data: B2SolverData): void;
    SolvePositionConstraints(data: B2SolverData): boolean;
}
