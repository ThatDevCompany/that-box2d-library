import { B2MakeArray } from '../../Common/b2Settings';
import { B2ShapeType } from '../../Collision/Shapes/b2Shape';
import { B2Contact } from './b2Contact';
import { B2CircleContact } from './b2CircleContact';
import { B2PolygonContact } from './b2PolygonContact';
import { B2PolygonAndCircleContact } from './b2PolygonAndCircleContact';
import { B2EdgeAndCircleContact } from './b2EdgeAndCircleContact';
import { B2EdgeAndPolygonContact } from './b2EdgeAndPolygonContact';
import { B2ChainAndCircleContact } from './b2ChainAndCircleContact';
import { B2ChainAndPolygonContact } from './b2ChainAndPolygonContact';
import { B2Fixture } from '../b2Fixture';

export class B2ContactRegister {
  public pool: B2Contact[] = null;
  public createFcn: { (allocator: any): B2Contact; } = null;
  public destroyFcn: { (contact: B2Contact, allocator: any): void; } = null;
  public primary: boolean = false;
}

export class B2ContactFactory {
  public m_allocator: any = null;
  public m_registers: B2ContactRegister[][];

  constructor(allocator: any) {
    this.m_allocator = allocator;
    this.InitializeRegisters();
  }

  private AddType(createFcn: (allocator: any) => B2Contact, destroyFcn: (contact: B2Contact, allocator: any) => void, type1: B2ShapeType, type2: B2ShapeType): void {
    const that: B2ContactFactory = this;

    const pool: B2Contact[] = B2MakeArray(256, function (i) { return createFcn(that.m_allocator); } ); // TODO: B2Settings

    function poolCreateFcn(allocator: any): B2Contact {
      if (pool.length > 0) {
        return pool.pop();
      }

      return createFcn(allocator);
    }

    function poolDestroyFcn(contact: B2Contact, allocator: any): void {
      pool.push(contact);
    }

    this.m_registers[type1][type2].pool = pool;
    this.m_registers[type1][type2].createFcn = poolCreateFcn;
    this.m_registers[type1][type2].destroyFcn = poolDestroyFcn;
    this.m_registers[type1][type2].primary = true;

    if (type1 !== type2) {
      this.m_registers[type2][type1].pool = pool;
      this.m_registers[type2][type1].createFcn = poolCreateFcn;
      this.m_registers[type2][type1].destroyFcn = poolDestroyFcn;
      this.m_registers[type2][type1].primary = false;
    }

    /*
    this.m_registers[type1][type2].createFcn = createFcn;
    this.m_registers[type1][type2].destroyFcn = destroyFcn;
    this.m_registers[type1][type2].primary = true;

    if (type1 !== type2) {
      this.m_registers[type2][type1].createFcn = createFcn;
      this.m_registers[type2][type1].destroyFcn = destroyFcn;
      this.m_registers[type2][type1].primary = false;
    }
    */
  }

  private InitializeRegisters(): void {
    this.m_registers = [/*b2ShapeType.e_shapeTypeCount*/];

    for (let i: number = 0; i < B2ShapeType.e_shapeTypeCount; i++) {
      this.m_registers[i] = [/*b2ShapeType.e_shapeTypeCount*/];

      for (let j: number = 0; j < B2ShapeType.e_shapeTypeCount; j++) {
        this.m_registers[i][j] = new B2ContactRegister();
      }
    }

    this.AddType(          B2CircleContact.Create,           B2CircleContact.Destroy, B2ShapeType.e_circleShape,  B2ShapeType.e_circleShape);
    this.AddType(B2PolygonAndCircleContact.Create, B2PolygonAndCircleContact.Destroy, B2ShapeType.e_polygonShape, B2ShapeType.e_circleShape);
    this.AddType(         B2PolygonContact.Create,          B2PolygonContact.Destroy, B2ShapeType.e_polygonShape, B2ShapeType.e_polygonShape);
    this.AddType(   B2EdgeAndCircleContact.Create,    B2EdgeAndCircleContact.Destroy, B2ShapeType.e_edgeShape,    B2ShapeType.e_circleShape);
    this.AddType(  B2EdgeAndPolygonContact.Create,   B2EdgeAndPolygonContact.Destroy, B2ShapeType.e_edgeShape,    B2ShapeType.e_polygonShape);
    this.AddType(  B2ChainAndCircleContact.Create,   B2ChainAndCircleContact.Destroy, B2ShapeType.e_chainShape,   B2ShapeType.e_circleShape);
    this.AddType( B2ChainAndPolygonContact.Create,  B2ChainAndPolygonContact.Destroy, B2ShapeType.e_chainShape,   B2ShapeType.e_polygonShape);
  }

  public Create(fixtureA: B2Fixture, indexA: number, fixtureB: B2Fixture, indexB: number): B2Contact {
    const type1: B2ShapeType = fixtureA.GetType();
    const type2: B2ShapeType = fixtureB.GetType();

    /// b2Assert(0 <= type1 && type1 < B2ShapeType.e_shapeTypeCount);
    /// b2Assert(0 <= type2 && type2 < B2ShapeType.e_shapeTypeCount);

    const reg: B2ContactRegister = this.m_registers[type1][type2];

    const c: B2Contact = reg.createFcn(this.m_allocator);
    if (reg.primary) {
      c.Reset(fixtureA, indexA, fixtureB, indexB);
    } else {
      c.Reset(fixtureB, indexB, fixtureA, indexA);
    }
    return c;
  }

  public Destroy(contact: B2Contact): void {
    const fixtureA: B2Fixture = contact.m_fixtureA;
    const fixtureB: B2Fixture = contact.m_fixtureB;

    if (contact.m_manifold.pointCount > 0 &&
      !fixtureA.IsSensor() &&
      !fixtureB.IsSensor()) {
      fixtureA.GetBody().SetAwake(true);
      fixtureB.GetBody().SetAwake(true);
    }

    const typeA: B2ShapeType = fixtureA.GetType();
    const typeB: B2ShapeType = fixtureB.GetType();

    /// b2Assert(0 <= typeA && typeB < B2ShapeType.e_shapeTypeCount);
    /// b2Assert(0 <= typeA && typeB < B2ShapeType.e_shapeTypeCount);

    const reg: B2ContactRegister = this.m_registers[typeA][typeB];

    reg.destroyFcn(contact, this.m_allocator);
  }
}
