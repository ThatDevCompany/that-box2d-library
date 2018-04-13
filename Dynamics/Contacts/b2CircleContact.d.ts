import { B2Transform } from '../../Common/b2Math';
import { B2Manifold } from '../../Collision/b2Collision';
import { B2Contact } from './b2Contact';
import { B2Fixture } from '../b2Fixture';
export declare class B2CircleContact extends B2Contact {
    constructor();
    static Create(allocator: any): B2Contact;
    static Destroy(contact: B2Contact, allocator: any): void;
    Reset(fixtureA: B2Fixture, indexA: number, fixtureB: B2Fixture, indexB: number): void;
    Evaluate(manifold: B2Manifold, xfA: B2Transform, xfB: B2Transform): void;
}
