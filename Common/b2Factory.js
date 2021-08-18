"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.B2Factory = void 0;
const index_1 = require("../Collision/index");
const index_2 = require("../Dynamics/index");
const b2Math_1 = require("./b2Math");
exports.B2Factory = {
    createBox(world, def) {
        // Create Body
        const body = world.CreateBody(new index_2.B2BodyDef());
        body.SetType(def.type || 0 /* B2_staticBody */);
        body.SetAngle(b2Math_1.B2DegToRad(def.angle || 0));
        body.SetPosition(def.position);
        body.SetUserData({ name: def.name });
        // Create Shape
        const shape = new index_1.B2PolygonShape();
        shape.Set([
            new b2Math_1.B2Vec2(-def.size.x / 2, -def.size.y / 2),
            new b2Math_1.B2Vec2(-def.size.x / 2, +def.size.y / 2),
            new b2Math_1.B2Vec2(+def.size.x / 2, +def.size.y / 2),
            new b2Math_1.B2Vec2(+def.size.x / 2, -def.size.y / 2),
            new b2Math_1.B2Vec2(-def.size.x / 2, -def.size.y / 2),
        ]);
        // Create Fixture Def
        const fixtureDef = new index_2.B2FixtureDef();
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
