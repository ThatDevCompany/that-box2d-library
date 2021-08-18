"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.B2JointFactory = void 0;
const b2AreaJoint_1 = require("./b2AreaJoint");
const b2DistanceJoint_1 = require("./b2DistanceJoint");
const b2FrictionJoint_1 = require("./b2FrictionJoint");
const b2GearJoint_1 = require("./b2GearJoint");
const b2MotorJoint_1 = require("./b2MotorJoint");
const b2MouseJoint_1 = require("./b2MouseJoint");
const b2PrismaticJoint_1 = require("./b2PrismaticJoint");
const b2PulleyJoint_1 = require("./b2PulleyJoint");
const b2RevoluteJoint_1 = require("./b2RevoluteJoint");
const b2RopeJoint_1 = require("./b2RopeJoint");
const b2WeldJoint_1 = require("./b2WeldJoint");
const b2WheelJoint_1 = require("./b2WheelJoint");
class B2JointFactory {
    static Create(def, allocator) {
        let joint = null;
        switch (def.type) {
            case 3 /* e_distanceJoint */:
                joint = new b2DistanceJoint_1.B2DistanceJoint(def);
                break;
            case 5 /* e_mouseJoint */:
                joint = new b2MouseJoint_1.B2MouseJoint(def);
                break;
            case 2 /* e_prismaticJoint */:
                joint = new b2PrismaticJoint_1.B2PrismaticJoint(def);
                break;
            case 1 /* e_revoluteJoint */:
                joint = new b2RevoluteJoint_1.B2RevoluteJoint(def);
                break;
            case 4 /* e_pulleyJoint */:
                joint = new b2PulleyJoint_1.B2PulleyJoint(def);
                break;
            case 6 /* e_gearJoint */:
                joint = new b2GearJoint_1.B2GearJoint(def);
                break;
            case 7 /* e_wheelJoint */:
                joint = new b2WheelJoint_1.B2WheelJoint(def);
                break;
            case 8 /* e_weldJoint */:
                joint = new b2WeldJoint_1.B2WeldJoint(def);
                break;
            case 9 /* e_frictionJoint */:
                joint = new b2FrictionJoint_1.B2FrictionJoint(def);
                break;
            case 10 /* e_ropeJoint */:
                joint = new b2RopeJoint_1.B2RopeJoint(def);
                break;
            case 11 /* e_motorJoint */:
                joint = new b2MotorJoint_1.B2MotorJoint(def);
                break;
            case 12 /* e_areaJoint */:
                joint = new b2AreaJoint_1.B2AreaJoint(def);
                break;
            default:
                /// b2Assert(false);
                break;
        }
        return joint;
    }
    static Destroy(joint, allocator) { }
}
exports.B2JointFactory = B2JointFactory;
