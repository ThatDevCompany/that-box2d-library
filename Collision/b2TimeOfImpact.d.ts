import { B2Vec2, B2Sweep } from '../Common/b2Math';
import { B2DistanceProxy, B2SimplexCache } from './b2Distance';
export declare let B2_toiTime: number;
export declare let B2_toiMaxTime: number;
export declare let B2_toiCalls: number;
export declare let B2_toiIters: number;
export declare let B2_toiMaxIters: number;
export declare let B2_toiRootIters: number;
export declare let B2_toiMaxRootIters: number;
export declare class B2TOIInput {
    proxyA: B2DistanceProxy;
    proxyB: B2DistanceProxy;
    sweepA: B2Sweep;
    sweepB: B2Sweep;
    tMax: number;
}
export declare const enum B2TOIOutputState {
    e_unknown = 0,
    e_failed = 1,
    e_overlapped = 2,
    e_touching = 3,
    e_separated = 4,
}
export declare class B2TOIOutput {
    state: B2TOIOutputState;
    t: number;
}
export declare const enum B2SeparationFunctionType {
    e_unknown = -1,
    e_points = 0,
    e_faceA = 1,
    e_faceB = 2,
}
export declare class B2SeparationFunction {
    m_proxyA: B2DistanceProxy;
    m_proxyB: B2DistanceProxy;
    m_sweepA: B2Sweep;
    m_sweepB: B2Sweep;
    m_type: B2SeparationFunctionType;
    m_localPoint: B2Vec2;
    m_axis: B2Vec2;
    Initialize(cache: B2SimplexCache, proxyA: B2DistanceProxy, sweepA: B2Sweep, proxyB: B2DistanceProxy, sweepB: B2Sweep, t1: number): number;
    FindMinSeparation(indexA: number[], indexB: number[], t: number): number;
    Evaluate(indexA: number, indexB: number, t: number): number;
}
export declare function B2TimeOfImpact(output: B2TOIOutput, input: B2TOIInput): void;
