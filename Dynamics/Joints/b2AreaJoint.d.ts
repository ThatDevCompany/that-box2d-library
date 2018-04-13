import { B2Vec2 } from '../../Common/b2Math';
import { B2Joint, B2JointDef } from './b2Joint';
import { B2DistanceJoint } from './b2DistanceJoint';
import { B2SolverData } from '../b2TimeStep';
import { B2Body } from '../b2Body';
import { B2World } from '../b2World';
export declare class B2AreaJointDef extends B2JointDef {
    world: B2World;
    bodies: B2Body[];
    frequencyHz: number;
    dampingRatio: number;
    constructor();
    AddBody(body: B2Body): void;
}
export declare class B2AreaJoint extends B2Joint {
    m_bodies: B2Body[];
    m_frequencyHz: number;
    m_dampingRatio: number;
    m_impulse: number;
    m_targetLengths: number[];
    m_targetArea: number;
    m_normals: B2Vec2[];
    m_joints: B2DistanceJoint[];
    m_deltas: B2Vec2[];
    m_delta: B2Vec2;
    constructor(def: B2AreaJointDef);
    GetAnchorA(out: B2Vec2): B2Vec2;
    GetAnchorB(out: B2Vec2): B2Vec2;
    GetReactionForce(inv_dt: number, out: B2Vec2): B2Vec2;
    GetReactionTorque(inv_dt: number): number;
    SetFrequency(hz: number): void;
    GetFrequency(): number;
    SetDampingRatio(ratio: number): void;
    GetDampingRatio(): number;
    Dump(log: (format: string, ...args: any[]) => void): void;
    InitVelocityConstraints(data: B2SolverData): void;
    SolveVelocityConstraints(data: B2SolverData): void;
    SolvePositionConstraints(data: B2SolverData): boolean;
}
