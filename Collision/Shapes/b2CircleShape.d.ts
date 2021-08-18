import { B2Vec2, B2Transform } from '../../Common/b2Math';
import { B2AABB, B2RayCastInput, B2RayCastOutput } from '../b2Collision';
import { B2DistanceProxy } from '../b2Distance';
import { B2MassData } from './b2Shape';
import { B2Shape } from './b2Shape';
export declare class B2CircleShape extends B2Shape {
    m_p: B2Vec2;
    constructor(radius?: number);
    Clone(): B2CircleShape;
    Copy(other: B2CircleShape): B2CircleShape;
    GetChildCount(): number;
    private static TestPoint_s_center;
    private static TestPoint_s_d;
    TestPoint(transform: B2Transform, p: B2Vec2): boolean;
    private static ComputeDistance_s_center;
    ComputeDistance(xf: B2Transform, p: B2Vec2, normal: B2Vec2, childIndex: number): number;
    private static RayCast_s_position;
    private static RayCast_s_s;
    private static RayCast_s_r;
    RayCast(output: B2RayCastOutput, input: B2RayCastInput, transform: B2Transform, childIndex: number): boolean;
    private static ComputeAABB_s_p;
    ComputeAABB(aabb: B2AABB, transform: B2Transform, childIndex: number): void;
    ComputeMass(massData: B2MassData, density: number): void;
    SetupDistanceProxy(proxy: B2DistanceProxy, index: number): void;
    ComputeSubmergedArea(normal: B2Vec2, offset: number, xf: B2Transform, c: B2Vec2): number;
    Dump(log: (format: string, ...args: any[]) => void): void;
}
