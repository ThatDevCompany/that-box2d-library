import { B2BroadPhase } from '../Collision/b2BroadPhase';
import { B2Contact } from './Contacts/b2Contact';
import { B2ContactFactory } from './Contacts/b2ContactFactory';
import { B2ContactFilter, B2ContactListener } from './b2WorldCallbacks';
export declare class B2ContactManager {
    m_broadPhase: B2BroadPhase;
    m_contactList: B2Contact;
    m_contactCount: number;
    m_contactFilter: B2ContactFilter;
    m_contactListener: B2ContactListener;
    m_allocator: any;
    m_contactFactory: B2ContactFactory;
    constructor();
    AddPair(proxyUserDataA: any, proxyUserDataB: any): void;
    FindNewContacts(): void;
    Destroy(c: B2Contact): void;
    Collide(): void;
}
