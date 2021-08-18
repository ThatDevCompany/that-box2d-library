import { B2PolygonShape } from "../Collision/index";
import {
  B2Body,
  B2BodyDef,
  B2BodyType,
  B2FixtureDef,
  B2World,
} from "../Dynamics/index";
import { B2DegToRad, B2Vec2 } from "./b2Math";

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

export const B2Factory = {
  createBox(world: B2World, def: B2BoxDef): B2Body {
    // Create Body
    const body = world.CreateBody(new B2BodyDef());
    body.SetType(def.type || B2BodyType.B2_staticBody);
    body.SetAngle(B2DegToRad(def.angle || 0));
    body.SetPosition(def.position);
    body.SetUserData({ name: def.name });

    // Create Shape
    const shape = new B2PolygonShape();
    shape.Set([
      new B2Vec2(-def.size.x / 2, -def.size.y / 2),
      new B2Vec2(-def.size.x / 2, +def.size.y / 2),
      new B2Vec2(+def.size.x / 2, +def.size.y / 2),
      new B2Vec2(+def.size.x / 2, -def.size.y / 2),
      new B2Vec2(-def.size.x / 2, -def.size.y / 2),
    ]);

    // Create Fixture Def
    const fixtureDef = new B2FixtureDef();
    fixtureDef.shape = shape;
    fixtureDef.userData = { name: def.name };
    fixtureDef.density = def.density || 1;
    fixtureDef.friction = def.friction || 0.3;
    fixtureDef.restitution = def.restitution || 0.25;

    // Create Fixture
    body.CreateFixture(fixtureDef);

    return body;
  },
};
