import { B2Joint, B2JointDef, B2JointType } from './b2Joint';
import { B2AreaJoint, B2AreaJointDef } from './b2AreaJoint';
import { B2DistanceJoint, B2DistanceJointDef } from './b2DistanceJoint';
import { B2FrictionJoint, B2FrictionJointDef } from './b2FrictionJoint';
import { B2GearJoint, B2GearJointDef } from './b2GearJoint';
import { B2MotorJoint, B2MotorJointDef } from './b2MotorJoint';
import { B2MouseJoint, B2MouseJointDef } from './b2MouseJoint';
import { B2PrismaticJoint, B2PrismaticJointDef } from './b2PrismaticJoint';
import { B2PulleyJoint, B2PulleyJointDef } from './b2PulleyJoint';
import { B2RevoluteJoint, B2RevoluteJointDef } from './b2RevoluteJoint';
import { B2RopeJoint, B2RopeJointDef } from './b2RopeJoint';
import { B2WeldJoint, B2WeldJointDef } from './b2WeldJoint';
import { B2WheelJoint, B2WheelJointDef } from './b2WheelJoint';

export class B2JointFactory {
  public static Create(def: B2JointDef, allocator: any): B2Joint {
    let joint: B2Joint = null;

    switch (def.type) {
    case B2JointType.e_distanceJoint:
      joint = new B2DistanceJoint(<B2DistanceJointDef> def);
      break;

    case B2JointType.e_mouseJoint:
      joint = new B2MouseJoint(<B2MouseJointDef> def);
      break;

    case B2JointType.e_prismaticJoint:
      joint = new B2PrismaticJoint(<B2PrismaticJointDef> def);
      break;

    case B2JointType.e_revoluteJoint:
      joint = new B2RevoluteJoint(<B2RevoluteJointDef> def);
      break;

    case B2JointType.e_pulleyJoint:
      joint = new B2PulleyJoint(<B2PulleyJointDef> def);
      break;

    case B2JointType.e_gearJoint:
      joint = new B2GearJoint(<B2GearJointDef> def);
      break;

    case B2JointType.e_wheelJoint:
      joint = new B2WheelJoint(<B2WheelJointDef> def);
      break;

    case B2JointType.e_weldJoint:
      joint = new B2WeldJoint(<B2WeldJointDef> def);
      break;

    case B2JointType.e_frictionJoint:
      joint = new B2FrictionJoint(<B2FrictionJointDef> def);
      break;

    case B2JointType.e_ropeJoint:
      joint = new B2RopeJoint(<B2RopeJointDef> def);
      break;

    case B2JointType.e_motorJoint:
      joint = new B2MotorJoint(<B2MotorJointDef> def);
      break;

    case B2JointType.e_areaJoint:
      joint = new B2AreaJoint(<B2AreaJointDef> def);
      break;

    default:
      /// b2Assert(false);
      break;
    }

    return joint;
  }

  public static Destroy(joint: B2Joint, allocator: any): void {
  }
}
