import { B2Vec2, B2Transform } from '../Common/b2Math';
import { B2Color } from '../Common/b2Draw';
import { B2Shape } from '../Collision/Shapes/b2Shape';
import { B2ParticleFlag } from './b2Particle';
import { B2ParticleSystem } from './b2ParticleSystem';
export declare const enum B2ParticleGroupFlag {
    B2_solidParticleGroup = 1,
    B2_rigidParticleGroup = 2,
    B2_particleGroupCanBeEmpty = 4,
    B2_particleGroupWillBeDestroyed = 8,
    B2_particleGroupNeedsUpdateDepth = 16,
    B2_particleGroupInternalMask = 24
}
export declare class B2ParticleGroupDef {
    flags: B2ParticleFlag;
    groupFlags: B2ParticleGroupFlag;
    position: B2Vec2;
    angle: number;
    linearVelocity: B2Vec2;
    angularVelocity: number;
    color: B2Color;
    strength: number;
    shape: B2Shape;
    shapes: B2Shape[];
    shapeCount: number;
    stride: number;
    particleCount: number;
    positionData: B2Vec2[];
    lifetime: number;
    userData: any;
    group: B2ParticleGroup;
}
export declare class B2ParticleGroup {
    m_system: B2ParticleSystem;
    m_firstIndex: number;
    m_lastIndex: number;
    m_groupFlags: B2ParticleGroupFlag;
    m_strength: number;
    m_prev: B2ParticleGroup;
    m_next: B2ParticleGroup;
    m_timestamp: number;
    m_mass: number;
    m_inertia: number;
    m_center: B2Vec2;
    m_linearVelocity: B2Vec2;
    m_angularVelocity: number;
    m_transform: B2Transform;
    m_userData: any;
    GetNext(): B2ParticleGroup;
    GetParticleSystem(): B2ParticleSystem;
    GetParticleCount(): number;
    GetBufferIndex(): number;
    ContainsParticle(index: number): boolean;
    GetAllParticleFlags(): B2ParticleFlag;
    GetGroupFlags(): B2ParticleGroupFlag;
    SetGroupFlags(flags: number): void;
    GetMass(): number;
    GetInertia(): number;
    GetCenter(): B2Vec2;
    GetLinearVelocity(): B2Vec2;
    GetAngularVelocity(): number;
    GetTransform(): B2Transform;
    GetPosition(): B2Vec2;
    GetAngle(): number;
    GetLinearVelocityFromWorldPoint(worldPoint: B2Vec2, out: B2Vec2): B2Vec2;
    static GetLinearVelocityFromWorldPoint_s_t0: B2Vec2;
    GetUserData(): void;
    SetUserData(data: any): void;
    ApplyForce(force: B2Vec2): void;
    ApplyLinearImpulse(impulse: B2Vec2): void;
    DestroyParticles(callDestructionListener: boolean): void;
    UpdateStatistics(): void;
}
