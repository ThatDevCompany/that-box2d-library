import { B2Vec2 } from '../Common/b2Math';
import { B2Color } from '../Common/b2Draw';
import { B2ParticleGroup } from './b2ParticleGroup';
/**
 * The particle type. Can be combined with the | operator.
 */
export declare const enum B2ParticleFlag {
    B2_waterParticle = 0,
    B2_zombieParticle = 2,
    B2_wallParticle = 4,
    B2_springParticle = 8,
    B2_elasticParticle = 16,
    B2_viscousParticle = 32,
    B2_powderParticle = 64,
    B2_tensileParticle = 128,
    B2_colorMixingParticle = 256,
    B2_destructionListenerParticle = 512,
    B2_barrierParticle = 1024,
    B2_staticPressureParticle = 2048,
    B2_reactiveParticle = 4096,
    B2_repulsiveParticle = 8192,
    B2_fixtureContactListenerParticle = 16384,
    B2_particleContactListenerParticle = 32768,
    B2_fixtureContactFilterParticle = 65536,
    B2_particleContactFilterParticle = 131072,
}
export declare class B2ParticleDef {
    flags: B2ParticleFlag;
    position: B2Vec2;
    velocity: B2Vec2;
    color: B2Color;
    lifetime: number;
    userData: any;
    group: B2ParticleGroup;
}
export declare function B2CalculateParticleIterations(gravity: number, radius: number, timeStep: number): number;
export declare class B2ParticleHandle {
    m_index: number;
    GetIndex(): number;
    SetIndex(index: number): void;
}
