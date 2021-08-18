import { B2Joint, B2JointDef } from './b2Joint';
export declare class B2JointFactory {
    static Create(def: B2JointDef, allocator: any): B2Joint;
    static Destroy(joint: B2Joint, allocator: any): void;
}
