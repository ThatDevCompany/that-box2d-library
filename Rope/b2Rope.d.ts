import { B2Vec2 } from '../Common/b2Math';
import { B2Draw } from '../Common/b2Draw';
export declare class B2RopeDef {
    vertices: B2Vec2[];
    count: number;
    masses: number[];
    gravity: B2Vec2;
    damping: number;
    k2: number;
    k3: number;
}
export declare class B2Rope {
    m_count: number;
    m_ps: B2Vec2[];
    m_p0s: B2Vec2[];
    m_vs: B2Vec2[];
    m_ims: number[];
    m_Ls: number[];
    m_as: number[];
    m_gravity: B2Vec2;
    m_damping: number;
    m_k2: number;
    m_k3: number;
    GetVertexCount(): number;
    GetVertices(): B2Vec2[];
    Initialize(def: B2RopeDef): void;
    Step(h: number, iterations: number): void;
    private static s_d;
    SolveC2(): void;
    SetAngle(angle: number): void;
    private static s_d1;
    private static s_d2;
    private static s_Jd1;
    private static s_Jd2;
    private static s_J1;
    private static s_J2;
    SolveC3(): void;
    Draw(draw: B2Draw): void;
}
