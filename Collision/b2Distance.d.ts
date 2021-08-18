import { B2Vec2, B2Transform } from '../Common/b2Math';
import { B2Shape } from './Shapes/b2Shape';
export declare class B2DistanceProxy {
    m_buffer: B2Vec2[];
    m_vertices: B2Vec2[];
    m_count: number;
    m_radius: number;
    Reset(): B2DistanceProxy;
    SetShape(shape: B2Shape, index: number): void;
    GetSupport(d: B2Vec2): number;
    GetSupportVertex(d: B2Vec2): B2Vec2;
    GetVertexCount(): number;
    GetVertex(index: number): B2Vec2;
}
export declare class B2SimplexCache {
    metric: number;
    count: number;
    indexA: number[];
    indexB: number[];
    Reset(): B2SimplexCache;
}
export declare class B2DistanceInput {
    proxyA: B2DistanceProxy;
    proxyB: B2DistanceProxy;
    transformA: B2Transform;
    transformB: B2Transform;
    useRadii: boolean;
    Reset(): B2DistanceInput;
}
export declare class B2DistanceOutput {
    pointA: B2Vec2;
    pointB: B2Vec2;
    distance: number;
    iterations: number;
    Reset(): B2DistanceOutput;
}
export declare let B2_gjkCalls: number;
export declare let B2_gjkIters: number;
export declare let B2_gjkMaxIters: number;
export declare class B2SimplexVertex {
    wA: B2Vec2;
    wB: B2Vec2;
    w: B2Vec2;
    a: number;
    indexA: number;
    indexB: number;
    Copy(other: B2SimplexVertex): B2SimplexVertex;
}
export declare class B2Simplex {
    m_v1: B2SimplexVertex;
    m_v2: B2SimplexVertex;
    m_v3: B2SimplexVertex;
    m_vertices: B2SimplexVertex[];
    m_count: number;
    constructor();
    ReadCache(cache: B2SimplexCache, proxyA: B2DistanceProxy, transformA: B2Transform, proxyB: B2DistanceProxy, transformB: B2Transform): void;
    WriteCache(cache: B2SimplexCache): void;
    GetSearchDirection(out: B2Vec2): B2Vec2;
    GetClosestPoint(out: B2Vec2): B2Vec2;
    GetWitnessPoints(pA: B2Vec2, pB: B2Vec2): void;
    GetMetric(): number;
    Solve2(): void;
    Solve3(): void;
    private static s_e12;
    private static s_e13;
    private static s_e23;
}
export declare function B2Distance(output: B2DistanceOutput, cache: B2SimplexCache, input: B2DistanceInput): void;
