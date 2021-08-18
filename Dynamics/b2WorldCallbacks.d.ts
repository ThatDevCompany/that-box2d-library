import { B2Vec2 } from '../Common/b2Math';
import { B2Manifold } from '../Collision/b2Collision';
import { B2Contact } from './Contacts/b2Contact';
import { B2Joint } from './Joints/b2Joint';
import { B2Fixture } from './b2Fixture';
import { B2ParticleGroup } from '../Particle/b2ParticleGroup';
import { B2ParticleSystem, B2ParticleContact, B2ParticleBodyContact } from '../Particle/b2ParticleSystem';
export declare class B2DestructionListener {
    SayGoodbyeJoint(joint: B2Joint): void;
    SayGoodbyeFixture(fixture: B2Fixture): void;
    SayGoodbyeParticleGroup(group: B2ParticleGroup): void;
    SayGoodbyeParticle(system: B2ParticleSystem, index: number): void;
}
export declare class B2ContactFilter {
    ShouldCollide(fixtureA: B2Fixture, fixtureB: B2Fixture): boolean;
    ShouldCollideFixtureParticle(fixture: B2Fixture, system: B2ParticleSystem, index: number): boolean;
    ShouldCollideParticleParticle(system: B2ParticleSystem, indexA: number, indexB: number): boolean;
    static B2_defaultFilter: B2ContactFilter;
}
export declare class B2ContactImpulse {
    normalImpulses: number[];
    tangentImpulses: number[];
    count: number;
}
export declare class B2ContactListener {
    BeginContact(contact: B2Contact): void;
    ContinueContact(contact: B2Contact): void;
    EndContact(contact: B2Contact): void;
    BeginContactFixtureParticle(system: B2ParticleSystem, contact: B2ParticleBodyContact): void;
    EndContactFixtureParticle(system: B2ParticleSystem, contact: B2ParticleBodyContact): void;
    BeginContactParticleParticle(system: B2ParticleSystem, contact: B2ParticleContact): void;
    EndContactParticleParticle(system: B2ParticleSystem, contact: B2ParticleContact): void;
    PreSolve(contact: B2Contact, oldManifold: B2Manifold): void;
    PostSolve(contact: B2Contact, impulse: B2ContactImpulse): void;
    static B2_defaultListener: B2ContactListener;
}
export declare class B2QueryCallback {
    ReportFixture(fixture: B2Fixture): boolean;
    ReportParticle(system: B2ParticleSystem, index: number): boolean;
    ShouldQueryParticleSystem(system: B2ParticleSystem): boolean;
}
export declare type B2QueryCallbackFunction = {
    (fixture: B2Fixture): boolean;
};
export declare class B2RayCastCallback {
    ReportFixture(fixture: B2Fixture, point: B2Vec2, normal: B2Vec2, fraction: number): number;
    ReportParticle(system: B2ParticleSystem, index: number, point: B2Vec2, normal: B2Vec2, fraction: number): number;
    ShouldQueryParticleSystem(system: B2ParticleSystem): boolean;
}
export declare type B2RayCastCallbackFunction = {
    (fixture: B2Fixture, point: B2Vec2, normal: B2Vec2, fraction: number): number;
};
