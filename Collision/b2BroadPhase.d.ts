import { B2Vec2 } from '../Common/b2Math';
import { B2AABB, B2RayCastInput } from './b2Collision';
import { B2TreeNode, B2DynamicTree } from './b2DynamicTree';
import { B2ContactManager } from '../Dynamics/b2ContactManager';
export declare class B2Pair {
    proxyA: B2TreeNode | null;
    proxyB: B2TreeNode | null;
}
export declare class B2BroadPhase {
    m_tree: B2DynamicTree;
    m_proxyCount: number;
    m_moveCount: number;
    m_moveBuffer: B2TreeNode[];
    m_pairCount: number;
    m_pairBuffer: B2Pair[];
    CreateProxy(aabb: B2AABB, userData: any): B2TreeNode;
    DestroyProxy(proxy: B2TreeNode): void;
    MoveProxy(proxy: B2TreeNode, aabb: B2AABB, displacement: B2Vec2): void;
    TouchProxy(proxy: B2TreeNode): void;
    GetFatAABB(proxy: B2TreeNode): B2AABB;
    GetUserData(proxy: B2TreeNode): any;
    TestOverlap(proxyA: B2TreeNode, proxyB: B2TreeNode): boolean;
    GetProxyCount(): number;
    UpdatePairs(contactManager: B2ContactManager): void;
    Query(callback: (node: B2TreeNode) => boolean, aabb: B2AABB): void;
    RayCast(callback: (input: B2RayCastInput, node: B2TreeNode) => number, input: B2RayCastInput): void;
    GetTreeHeight(): number;
    GetTreeBalance(): number;
    GetTreeQuality(): number;
    ShiftOrigin(newOrigin: B2Vec2): void;
    BufferMove(proxy: B2TreeNode): void;
    UnBufferMove(proxy: B2TreeNode): void;
}
export declare function B2PairLessThan(pair1: B2Pair, pair2: B2Pair): number;
