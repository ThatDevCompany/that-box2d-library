import { B2Vec2 } from '../Common/b2Math';
export declare class B2Profile {
    step: number;
    collide: number;
    solve: number;
    solveInit: number;
    solveVelocity: number;
    solvePosition: number;
    broadphase: number;
    solveTOI: number;
    Reset(): this;
}
export declare class B2TimeStep {
    dt: number;
    inv_dt: number;
    dtRatio: number;
    velocityIterations: number;
    positionIterations: number;
    particleIterations: number;
    warmStarting: boolean;
    Copy(step: B2TimeStep): B2TimeStep;
}
export declare class B2Position {
    c: B2Vec2;
    a: number;
    static MakeArray(length: number): B2Position[];
}
export declare class B2Velocity {
    v: B2Vec2;
    w: number;
    static MakeArray(length: number): B2Velocity[];
}
export declare class B2SolverData {
    step: B2TimeStep;
    positions: B2Position[];
    velocities: B2Velocity[];
}
