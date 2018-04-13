import { B2Vec2, B2Transform } from './b2Math';
export declare class B2Color {
    static RED: B2Color;
    static GREEN: B2Color;
    static BLUE: B2Color;
    r: number;
    g: number;
    b: number;
    a: number;
    constructor(rr?: number, gg?: number, bb?: number, aa?: number);
    Clone(): B2Color;
    Copy(other: B2Color): B2Color;
    IsEqual(color: B2Color): boolean;
    IsZero(): boolean;
    GetColor(out: B2Color): B2Color;
    SetColor(color: B2Color): void;
    Set(a0: number | B2Color, a1?: number, a2?: number, a3?: number): void;
    SetRGB(rr: number, gg: number, bb: number): B2Color;
    SetRGBA(rr: number, gg: number, bb: number, aa: number): B2Color;
    SelfAdd(color: B2Color): B2Color;
    Add(color: B2Color, out: B2Color): B2Color;
    SelfSub(color: B2Color): B2Color;
    Sub(color: B2Color, out: B2Color): B2Color;
    SelfMul_0_1(s: number): B2Color;
    Mul_0_1(s: number, out: B2Color): B2Color;
    Mix(mixColor: B2Color, strength: number): void;
    static MixColors(colorA: B2Color, colorB: B2Color, strength: number): void;
    MakeStyleString(alpha?: number): string;
    static MakeStyleString(r: number, g: number, b: number, a?: number): string;
}
export declare const enum B2DrawFlags {
    e_none = 0,
    e_shapeBit = 1,
    e_jointBit = 2,
    e_aabbBit = 4,
    e_pairBit = 8,
    e_centerOfMassBit = 16,
    e_particleBit = 32,
    e_controllerBit = 64,
    e_all = 63,
}
export declare class B2Draw {
    m_drawFlags: B2DrawFlags;
    SetFlags(flags: B2DrawFlags): void;
    GetFlags(): B2DrawFlags;
    AppendFlags(flags: B2DrawFlags): void;
    ClearFlags(flags: B2DrawFlags): void;
    PushTransform(xf: B2Transform): void;
    PopTransform(xf: B2Transform): void;
    DrawPolygon(vertices: B2Vec2[], vertexCount: number, color: B2Color): void;
    DrawSolidPolygon(vertices: B2Vec2[], vertexCount: number, color: B2Color): void;
    DrawCircle(center: B2Vec2, radius: number, color: B2Color): void;
    DrawSolidCircle(center: B2Vec2, radius: number, axis: B2Vec2, color: B2Color): void;
    DrawParticles(centers: B2Vec2[], radius: number, colors: B2Color[], count: number): void;
    DrawSegment(p1: B2Vec2, p2: B2Vec2, color: B2Color): void;
    DrawTransform(xf: B2Transform): void;
}
