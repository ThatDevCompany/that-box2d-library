import { B2Vec2, B2Transform } from '../Common/b2Math';
import { B2Shape } from './Shapes/b2Shape';
export declare const enum B2ContactFeatureType {
    e_vertex = 0,
    e_face = 1
}
export declare class B2ContactFeature {
    _key: number;
    _key_invalid: boolean;
    _indexA: number;
    _indexB: number;
    _typeA: number;
    _typeB: number;
    constructor();
    get key(): number;
    set key(value: number);
    get indexA(): number;
    set indexA(value: number);
    get indexB(): number;
    set indexB(value: number);
    get typeA(): number;
    set typeA(value: number);
    get typeB(): number;
    set typeB(value: number);
}
export declare class B2ContactID {
    cf: B2ContactFeature;
    Copy(o: B2ContactID): B2ContactID;
    Clone(): B2ContactID;
    get key(): number;
    set key(value: number);
}
export declare class B2ManifoldPoint {
    localPoint: B2Vec2;
    normalImpulse: number;
    tangentImpulse: number;
    id: B2ContactID;
    static MakeArray(length: number): B2ManifoldPoint[];
    Reset(): void;
    Copy(o: B2ManifoldPoint): B2ManifoldPoint;
}
export declare const enum B2ManifoldType {
    e_unknown = -1,
    e_circles = 0,
    e_faceA = 1,
    e_faceB = 2
}
export declare class B2Manifold {
    points: B2ManifoldPoint[];
    localNormal: B2Vec2;
    localPoint: B2Vec2;
    type: number;
    pointCount: number;
    Reset(): void;
    Copy(o: B2Manifold): B2Manifold;
    Clone(): B2Manifold;
}
export declare class B2WorldManifold {
    normal: B2Vec2;
    points: B2Vec2[];
    separations: number[];
    private static Initialize_s_pointA;
    private static Initialize_s_pointB;
    private static Initialize_s_cA;
    private static Initialize_s_cB;
    private static Initialize_s_planePoint;
    private static Initialize_s_clipPoint;
    Initialize(manifold: B2Manifold, xfA: B2Transform, radiusA: number, xfB: B2Transform, radiusB: number): void;
}
export declare const enum B2PointState {
    B2_nullState = 0,
    B2_addState = 1,
    B2_persistState = 2,
    B2_removeState = 3
}
export declare function B2GetPointStates(state1: B2PointState[], state2: B2PointState[], manifold1: B2Manifold, manifold2: B2Manifold): void;
export declare class B2ClipVertex {
    v: B2Vec2;
    id: B2ContactID;
    static MakeArray(length: number): B2ClipVertex[];
    Copy(other: B2ClipVertex): B2ClipVertex;
}
export declare class B2RayCastInput {
    p1: B2Vec2;
    p2: B2Vec2;
    maxFraction: number;
    Copy(o: B2RayCastInput): B2RayCastInput;
}
export declare class B2RayCastOutput {
    normal: B2Vec2;
    fraction: number;
    Copy(o: B2RayCastOutput): B2RayCastOutput;
}
export declare class B2AABB {
    lowerBound: B2Vec2;
    upperBound: B2Vec2;
    private m_cache_center;
    private m_cache_extent;
    Copy(o: B2AABB): B2AABB;
    IsValid(): boolean;
    GetCenter(): B2Vec2;
    GetExtents(): B2Vec2;
    GetPerimeter(): number;
    Combine1(aabb: B2AABB): B2AABB;
    Combine2(aabb1: B2AABB, aabb2: B2AABB): B2AABB;
    static Combine(aabb1: B2AABB, aabb2: B2AABB, out: B2AABB): B2AABB;
    Contains(aabb: B2AABB): boolean;
    RayCast(output: B2RayCastOutput, input: B2RayCastInput): boolean;
    TestOverlap(other: B2AABB): boolean;
}
export declare function B2TestOverlapAABB(a: B2AABB, b: B2AABB): boolean;
export declare function B2ClipSegmentToLine(vOut: B2ClipVertex[], vIn: B2ClipVertex[], normal: B2Vec2, offset: number, vertexIndexA: number): number;
export declare function B2TestOverlapShape(shapeA: B2Shape, indexA: number, shapeB: B2Shape, indexB: number, xfA: B2Transform, xfB: B2Transform): boolean;
