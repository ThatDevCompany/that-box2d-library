import { B2Vec2, B2Transform } from '../../Common/b2Math';
import { B2AABB, B2RayCastInput, B2RayCastOutput } from '../b2Collision';
import { B2DistanceProxy } from '../b2Distance';
import { B2MassData } from './b2Shape';
import { B2Shape } from './b2Shape';
import { B2EdgeShape } from './b2EdgeShape';
export declare class B2ChainShape extends B2Shape {
    m_vertices: B2Vec2[];
    m_count: number;
    m_prevVertex: B2Vec2;
    m_nextVertex: B2Vec2;
    m_hasPrevVertex: boolean;
    m_hasNextVertex: boolean;
    constructor();
    CreateLoop(vertices: B2Vec2[], count?: number): B2ChainShape;
    CreateChain(vertices: B2Vec2[], count?: number): B2ChainShape;
    SetPrevVertex(prevVertex: B2Vec2): B2ChainShape;
    SetNextVertex(nextVertex: B2Vec2): B2ChainShape;
    Clone(): B2ChainShape;
    Copy(other: B2ChainShape): B2ChainShape;
    GetChildCount(): number;
    GetChildEdge(edge: B2EdgeShape, index: number): void;
    TestPoint(xf: B2Transform, p: B2Vec2): boolean;
    private static ComputeDistance_s_edgeShape;
    ComputeDistance(xf: B2Transform, p: B2Vec2, normal: B2Vec2, childIndex: number): number;
    private static RayCast_s_edgeShape;
    RayCast(output: B2RayCastOutput, input: B2RayCastInput, xf: B2Transform, childIndex: number): boolean;
    private static ComputeAABB_s_v1;
    private static ComputeAABB_s_v2;
    ComputeAABB(aabb: B2AABB, xf: B2Transform, childIndex: number): void;
    ComputeMass(massData: B2MassData, density: number): void;
    SetupDistanceProxy(proxy: B2DistanceProxy, index: number): void;
    ComputeSubmergedArea(normal: B2Vec2, offset: number, xf: B2Transform, c: B2Vec2): number;
    Dump(log: (format: string, ...args: any[]) => void): void;
}
