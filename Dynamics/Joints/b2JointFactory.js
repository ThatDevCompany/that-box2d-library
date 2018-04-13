import { B2AreaJoint } from './b2AreaJoint';
import { B2DistanceJoint } from './b2DistanceJoint';
import { B2FrictionJoint } from './b2FrictionJoint';
import { B2GearJoint } from './b2GearJoint';
import { B2MotorJoint } from './b2MotorJoint';
import { B2MouseJoint } from './b2MouseJoint';
import { B2PrismaticJoint } from './b2PrismaticJoint';
import { B2PulleyJoint } from './b2PulleyJoint';
import { B2RevoluteJoint } from './b2RevoluteJoint';
import { B2RopeJoint } from './b2RopeJoint';
import { B2WeldJoint } from './b2WeldJoint';
import { B2WheelJoint } from './b2WheelJoint';
export class B2JointFactory {
    static Create(def, allocator) {
        let joint = null;
        switch (def.type) {
            case 3 /* e_distanceJoint */:
                joint = new B2DistanceJoint(def);
                break;
            case 5 /* e_mouseJoint */:
                joint = new B2MouseJoint(def);
                break;
            case 2 /* e_prismaticJoint */:
                joint = new B2PrismaticJoint(def);
                break;
            case 1 /* e_revoluteJoint */:
                joint = new B2RevoluteJoint(def);
                break;
            case 4 /* e_pulleyJoint */:
                joint = new B2PulleyJoint(def);
                break;
            case 6 /* e_gearJoint */:
                joint = new B2GearJoint(def);
                break;
            case 7 /* e_wheelJoint */:
                joint = new B2WheelJoint(def);
                break;
            case 8 /* e_weldJoint */:
                joint = new B2WeldJoint(def);
                break;
            case 9 /* e_frictionJoint */:
                joint = new B2FrictionJoint(def);
                break;
            case 10 /* e_ropeJoint */:
                joint = new B2RopeJoint(def);
                break;
            case 11 /* e_motorJoint */:
                joint = new B2MotorJoint(def);
                break;
            case 12 /* e_areaJoint */:
                joint = new B2AreaJoint(def);
                break;
            default:
                /// b2Assert(false);
                break;
        }
        return joint;
    }
    static Destroy(joint, allocator) {
    }
}
