import { B2Vec2 } from "../Common/index";
export interface B2AppliedForce {
    force: B2Vec2;
    point: B2Vec2;
}
export declare abstract class AbstractB2AppliedForce implements B2AppliedForce {
    force: B2Vec2;
    point: B2Vec2;
}
