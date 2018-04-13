import { B2Vec2, B2Transform } from '../Common/b2Math';
import { B2BroadPhase } from '../Collision/b2BroadPhase';
import { B2AABB, B2RayCastInput, B2RayCastOutput } from '../Collision/b2Collision';
import { B2TreeNode } from '../Collision/b2DynamicTree';
import { B2Shape, B2ShapeType, B2MassData } from '../Collision/Shapes/b2Shape';
import { B2Body } from './b2Body';
export declare class B2Filter {
    categoryBits: number;
    maskBits: number;
    groupIndex: number;
    Clone(): B2Filter;
    Copy(other: B2Filter): B2Filter;
}
export declare class B2FixtureDef {
    shape: B2Shape;
    userData: any;
    friction: number;
    restitution: number;
    density: number;
    isSensor: boolean;
    filter: B2Filter;
}
export declare class B2FixtureProxy {
    aabb: B2AABB;
    fixture: B2Fixture;
    childIndex: number;
    proxy: B2TreeNode;
    static MakeArray(length: number): B2FixtureProxy[];
}
export declare class B2Fixture {
    m_density: number;
    m_next: B2Fixture;
    m_body: B2Body;
    m_shape: B2Shape;
    m_friction: number;
    m_restitution: number;
    m_proxies: B2FixtureProxy[];
    m_proxyCount: number;
    m_filter: B2Filter;
    m_isSensor: boolean;
    m_userData: any;
    GetType(): B2ShapeType;
    GetShape(): B2Shape;
    SetSensor(sensor: boolean): void;
    IsSensor(): boolean;
    SetFilterData(filter: B2Filter): void;
    GetFilterData(): B2Filter;
    Refilter(): void;
    GetBody(): B2Body;
    GetNext(): B2Fixture;
    GetUserData(): any;
    SetUserData(data: any): void;
    TestPoint(p: B2Vec2): boolean;
    ComputeDistance(p: B2Vec2, normal: B2Vec2, childIndex: number): number;
    RayCast(output: B2RayCastOutput, input: B2RayCastInput, childIndex: number): boolean;
    GetMassData(massData?: B2MassData): B2MassData;
    SetDensity(density: number): void;
    GetDensity(): number;
    GetFriction(): number;
    SetFriction(friction: number): void;
    GetRestitution(): number;
    SetRestitution(restitution: number): void;
    GetAABB(childIndex: number): B2AABB;
    Dump(log: (format: string, ...args: any[]) => void, bodyIndex: number): void;
    Create(body: B2Body, def: B2FixtureDef): void;
    Destroy(): void;
    CreateProxies(broadPhase: B2BroadPhase, xf: B2Transform): void;
    DestroyProxies(broadPhase: B2BroadPhase): void;
    private static Synchronize_s_aabb1;
    private static Synchronize_s_aabb2;
    private static Synchronize_s_displacement;
    Synchronize(broadPhase: B2BroadPhase, transform1: B2Transform, transform2: B2Transform): void;
}
