import { B2Contact } from './b2Contact';
import { B2Fixture } from '../b2Fixture';
export declare class B2ContactRegister {
    pool: B2Contact[];
    createFcn: {
        (allocator: any): B2Contact;
    };
    destroyFcn: {
        (contact: B2Contact, allocator: any): void;
    };
    primary: boolean;
}
export declare class B2ContactFactory {
    m_allocator: any;
    m_registers: B2ContactRegister[][];
    constructor(allocator: any);
    private AddType;
    private InitializeRegisters;
    Create(fixtureA: B2Fixture, indexA: number, fixtureB: B2Fixture, indexB: number): B2Contact;
    Destroy(contact: B2Contact): void;
}
