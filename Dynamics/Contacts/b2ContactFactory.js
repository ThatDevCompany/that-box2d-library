import { B2MakeArray } from '../../Common/b2Settings';
import { B2CircleContact } from './b2CircleContact';
import { B2PolygonContact } from './b2PolygonContact';
import { B2PolygonAndCircleContact } from './b2PolygonAndCircleContact';
import { B2EdgeAndCircleContact } from './b2EdgeAndCircleContact';
import { B2EdgeAndPolygonContact } from './b2EdgeAndPolygonContact';
import { B2ChainAndCircleContact } from './b2ChainAndCircleContact';
import { B2ChainAndPolygonContact } from './b2ChainAndPolygonContact';
export class B2ContactRegister {
    constructor() {
        this.pool = null;
        this.createFcn = null;
        this.destroyFcn = null;
        this.primary = false;
    }
}
export class B2ContactFactory {
    constructor(allocator) {
        this.m_allocator = null;
        this.m_allocator = allocator;
        this.InitializeRegisters();
    }
    AddType(createFcn, destroyFcn, type1, type2) {
        const that = this;
        const pool = B2MakeArray(256, function (i) { return createFcn(that.m_allocator); }); // TODO: B2Settings
        function poolCreateFcn(allocator) {
            if (pool.length > 0) {
                return pool.pop();
            }
            return createFcn(allocator);
        }
        function poolDestroyFcn(contact, allocator) {
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
    InitializeRegisters() {
        this.m_registers = [];
        for (let i = 0; i < 4 /* e_shapeTypeCount */; i++) {
            this.m_registers[i] = [];
            for (let j = 0; j < 4 /* e_shapeTypeCount */; j++) {
                this.m_registers[i][j] = new B2ContactRegister();
            }
        }
        this.AddType(B2CircleContact.Create, B2CircleContact.Destroy, 0 /* e_circleShape */, 0 /* e_circleShape */);
        this.AddType(B2PolygonAndCircleContact.Create, B2PolygonAndCircleContact.Destroy, 2 /* e_polygonShape */, 0 /* e_circleShape */);
        this.AddType(B2PolygonContact.Create, B2PolygonContact.Destroy, 2 /* e_polygonShape */, 2 /* e_polygonShape */);
        this.AddType(B2EdgeAndCircleContact.Create, B2EdgeAndCircleContact.Destroy, 1 /* e_edgeShape */, 0 /* e_circleShape */);
        this.AddType(B2EdgeAndPolygonContact.Create, B2EdgeAndPolygonContact.Destroy, 1 /* e_edgeShape */, 2 /* e_polygonShape */);
        this.AddType(B2ChainAndCircleContact.Create, B2ChainAndCircleContact.Destroy, 3 /* e_chainShape */, 0 /* e_circleShape */);
        this.AddType(B2ChainAndPolygonContact.Create, B2ChainAndPolygonContact.Destroy, 3 /* e_chainShape */, 2 /* e_polygonShape */);
    }
    Create(fixtureA, indexA, fixtureB, indexB) {
        const type1 = fixtureA.GetType();
        const type2 = fixtureB.GetType();
        /// b2Assert(0 <= type1 && type1 < B2ShapeType.e_shapeTypeCount);
        /// b2Assert(0 <= type2 && type2 < B2ShapeType.e_shapeTypeCount);
        const reg = this.m_registers[type1][type2];
        const c = reg.createFcn(this.m_allocator);
        if (reg.primary) {
            c.Reset(fixtureA, indexA, fixtureB, indexB);
        }
        else {
            c.Reset(fixtureB, indexB, fixtureA, indexA);
        }
        return c;
    }
    Destroy(contact) {
        const fixtureA = contact.m_fixtureA;
        const fixtureB = contact.m_fixtureB;
        if (contact.m_manifold.pointCount > 0 &&
            !fixtureA.IsSensor() &&
            !fixtureB.IsSensor()) {
            fixtureA.GetBody().SetAwake(true);
            fixtureB.GetBody().SetAwake(true);
        }
        const typeA = fixtureA.GetType();
        const typeB = fixtureB.GetType();
        /// b2Assert(0 <= typeA && typeB < B2ShapeType.e_shapeTypeCount);
        /// b2Assert(0 <= typeA && typeB < B2ShapeType.e_shapeTypeCount);
        const reg = this.m_registers[typeA][typeB];
        reg.destroyFcn(contact, this.m_allocator);
    }
}
