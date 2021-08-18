import { B2Body, B2BodyType, B2World } from "../Dynamics/index";
import { B2Vec2 } from "./b2Math";
export interface B2BoxDef {
    name: string;
    size: B2Vec2;
    position: B2Vec2;
    type?: B2BodyType;
    angle?: number;
    density?: number;
    friction?: number;
    restitution?: number;
}
export declare const B2Factory: {
    createBox(world: B2World, def: B2BoxDef): B2Body;
};
