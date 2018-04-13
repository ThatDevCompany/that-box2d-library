import { B2Vec2, B2Transform } from '../../Common/b2Math';
import { B2AABB, B2RayCastInput, B2RayCastOutput } from '../b2Collision';
import { B2DistanceProxy } from '../b2Distance';
export declare class B2MassData {
    mass: number;
    center: B2Vec2;
    I: number;
}
export declare const enum B2ShapeType {
    e_unknown = -1,
    e_circleShape = 0,
    e_edgeShape = 1,
    e_polygonShape = 2,
    e_chainShape = 3,
    e_shapeTypeCount = 4,
}
export declare abstract class B2Shape {
    m_type: B2ShapeType;
    m_radius: number;
    constructor(type: B2ShapeType, radius: number);
    abstract Clone(): B2Shape;
    Copy(other: B2Shape): B2Shape;
    GetType(): B2ShapeType;
    abstract GetChildCount(): number;
    abstract TestPoint(xf: B2Transform, p: B2Vec2): boolean;
    abstract ComputeDistance(xf: B2Transform, p: B2Vec2, normal: B2Vec2, childIndex: number): number;
    abstract RayCast(output: B2RayCastOutput, input: B2RayCastInput, transform: B2Transform, childIndex: number): boolean;
    abstract ComputeAABB(aabb: B2AABB, xf: B2Transform, childIndex: number): void;
    abstract ComputeMass(massData: B2MassData, density: number): void;
    abstract SetupDistanceProxy(proxy: B2DistanceProxy, index: number): void;
    abstract ComputeSubmergedArea(normal: B2Vec2, offset: number, xf: B2Transform, c: B2Vec2): number;
    abstract Dump(log: (format: string, ...args: any[]) => void): void;
}
