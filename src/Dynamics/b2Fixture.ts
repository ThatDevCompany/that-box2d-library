/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

import { B2MakeArray } from '../Common/b2Settings';
import { B2Vec2, B2Transform } from '../Common/b2Math';
import { B2BroadPhase } from '../Collision/b2BroadPhase';
import { B2AABB, B2RayCastInput, B2RayCastOutput } from '../Collision/b2Collision';
import { B2TreeNode } from '../Collision/b2DynamicTree';
import { B2Shape, B2ShapeType, B2MassData } from '../Collision/Shapes/b2Shape';
import { B2Body } from './b2Body';

///  This holds contact filtering data.
export class B2Filter {
  ///  The collision category bits. Normally you would just set one bit.
  public categoryBits: number = 0x0001;

  ///  The collision mask bits. This states the categories that this
  ///  shape would accept for collision.
  public maskBits: number = 0xFFFF;

  ///  Collision groups allow a certain group of objects to never collide (negative)
  ///  or always collide (positive). Zero means no collision group. Non-zero group
  ///  filtering always wins against the mask bits.
  public groupIndex: number = 0;

  public Clone(): B2Filter {
    return new B2Filter().Copy(this);
  }

  public Copy(other: B2Filter): B2Filter {
    /// b2Assert(this !== other);
    this.categoryBits = other.categoryBits;
    this.maskBits = other.maskBits;
    this.groupIndex = other.groupIndex;
    return this;
  }
}

///  A fixture definition is used to create a fixture. This class defines an
///  abstract fixture definition. You can reuse fixture definitions safely.
export class B2FixtureDef {
  ///  The shape, this must be set. The shape will be cloned, so you
  ///  can create the shape on the stack.
  public shape: B2Shape = null;

  ///  Use this to store application specific fixture data.
  public userData: any = null;

  ///  The friction coefficient, usually in the range [0,1].
  public friction: number = 0.2;

  ///  The restitution (elasticity) usually in the range [0,1].
  public restitution: number = 0;

  ///  The density, usually in kg/m^2.
  public density: number = 0;

  ///  A sensor shape collects contact information but never generates a collision
  ///  response.
  public isSensor: boolean = false;

  ///  Contact filtering data.
  public filter: B2Filter = new B2Filter();
}

///  This proxy is used internally to connect fixtures to the broad-phase.
export class B2FixtureProxy {
  public aabb: B2AABB = new B2AABB();
  public fixture: B2Fixture = null;
  public childIndex: number = 0;
  public proxy: B2TreeNode = null;
  public static MakeArray(length: number): B2FixtureProxy[] {
    return B2MakeArray(length, function (i) { return new B2FixtureProxy(); });
  }
}

///  A fixture is used to attach a shape to a body for collision detection. A fixture
///  inherits its transform from its parent. Fixtures hold additional non-geometric data
///  such as friction, collision filters, etc.
///  Fixtures are created via B2Body::CreateFixture.
///  @warning you cannot reuse fixtures.
export class B2Fixture {
  public m_density: number = 0;

  public m_next: B2Fixture = null;
  public m_body: B2Body = null;

  public m_shape: B2Shape = null;

  public m_friction: number = 0;
  public m_restitution: number = 0;

  public m_proxies: B2FixtureProxy[] = null;
  public m_proxyCount: number = 0;

  public m_filter: B2Filter = new B2Filter();

  public m_isSensor: boolean = false;

  public m_userData: any = null;

  ///  Get the type of the child shape. You can use this to down cast to the concrete shape.
  ///  @return the shape type.
  public GetType(): B2ShapeType {
    return this.m_shape.GetType();
  }

  ///  Get the child shape. You can modify the child shape, however you should not change the
  ///  number of vertices because this will crash some collision caching mechanisms.
  ///  Manipulating the shape may lead to non-physical behavior.
  public GetShape(): B2Shape {
    return this.m_shape;
  }

  ///  Set if this fixture is a sensor.
  public SetSensor(sensor: boolean): void {
    if (sensor !== this.m_isSensor) {
      this.m_body.SetAwake(true);
      this.m_isSensor = sensor;
    }
  }

  ///  Is this fixture a sensor (non-solid)?
  ///  @return the true if the shape is a sensor.
  public IsSensor(): boolean {
    return this.m_isSensor;
  }

  ///  Set the contact filtering data. This will not update contacts until the next time
  ///  step when either parent body is active and awake.
  ///  This automatically calls Refilter.
  public SetFilterData(filter: B2Filter): void {
    this.m_filter.Copy(filter);

    this.Refilter();
  }

  ///  Get the contact filtering data.
  public GetFilterData(): B2Filter {
    return this.m_filter;
  }

  ///  Call this if you want to establish collision that was previously disabled by B2ContactFilter::ShouldCollide.
  public Refilter(): void {
    if (this.m_body) {
      return;
    }

    // Flag associated contacts for filtering.
    let edge = this.m_body.GetContactList();

    while (edge) {
      const contact = edge.contact;
      const fixtureA = contact.GetFixtureA();
      const fixtureB = contact.GetFixtureB();
      if (fixtureA === this || fixtureB === this) {
        contact.FlagForFiltering();
      }

      edge = edge.next;
    }

    const world = this.m_body.GetWorld();

    if (world === null) {
      return;
    }

    // Touch each proxy so that new pairs may be created
    const broadPhase = world.m_contactManager.m_broadPhase;
    for (let i: number = 0; i < this.m_proxyCount; ++i) {
      broadPhase.TouchProxy(this.m_proxies[i].proxy);
    }
  }

  ///  Get the parent body of this fixture. This is NULL if the fixture is not attached.
  ///  @return the parent body.
  public GetBody(): B2Body {
    return this.m_body;
  }

  ///  Get the next fixture in the parent body's fixture list.
  ///  @return the next shape.
  public GetNext(): B2Fixture {
    return this.m_next;
  }

  ///  Get the user data that was assigned in the fixture definition. Use this to
  ///  store your application specific data.
  public GetUserData(): any {
    return this.m_userData;
  }

  ///  Set the user data. Use this to store your application specific data.
  public SetUserData(data: any): void {
    this.m_userData = data;
  }

  ///  Test a point for containment in this fixture.
  ///  @param p a point in world coordinates.
  public TestPoint(p: B2Vec2): boolean {
    return this.m_shape.TestPoint(this.m_body.GetTransform(), p);
  }

  /// #if B2_ENABLE_PARTICLE
  public ComputeDistance(p: B2Vec2, normal: B2Vec2, childIndex: number): number {
    return this.m_shape.ComputeDistance(this.m_body.GetTransform(), p, normal, childIndex);
  }
  /// #endif

  ///  Cast a ray against this shape.
  ///  @param output the ray-cast results.
  ///  @param input the ray-cast input parameters.
  public RayCast(output: B2RayCastOutput, input: B2RayCastInput, childIndex: number): boolean {
    return this.m_shape.RayCast(output, input, this.m_body.GetTransform(), childIndex);
  }

  ///  Get the mass data for this fixture. The mass data is based on the density and
  ///  the shape. The rotational inertia is about the shape's origin. This operation
  ///  may be expensive.
  public GetMassData(massData: B2MassData = new B2MassData()): B2MassData {
    this.m_shape.ComputeMass(massData, this.m_density);

    return massData;
  }

  ///  Set the density of this fixture. This will _not_ automatically adjust the mass
  ///  of the body. You must call B2Body::ResetMassData to update the body's mass.
  public SetDensity(density: number): void {
    this.m_density = density;
  }

  ///  Get the density of this fixture.
  public GetDensity(): number {
    return this.m_density;
  }

  ///  Get the coefficient of friction.
  public GetFriction(): number {
    return this.m_friction;
  }

  ///  Set the coefficient of friction. This will _not_ change the friction of
  ///  existing contacts.
  public SetFriction(friction: number): void {
    this.m_friction = friction;
  }

  ///  Get the coefficient of restitution.
  public GetRestitution(): number {
    return this.m_restitution;
  }

  ///  Set the coefficient of restitution. This will _not_ change the restitution of
  ///  existing contacts.
  public SetRestitution(restitution: number): void {
    this.m_restitution = restitution;
  }

  ///  Get the fixture's AABB. This AABB may be enlarge and/or stale.
  ///  If you need a more accurate AABB, compute it using the shape and
  ///  the body transform.
  public GetAABB(childIndex: number): B2AABB {
    /// b2Assert(0 <= childIndex && childIndex < this.m_proxyCount);
    return this.m_proxies[childIndex].aabb;
  }

  ///  Dump this fixture to the log file.
  public Dump(log: (format: string, ...args: any[]) => void, bodyIndex: number): void {
    log('    const fd: B2FixtureDef = new B2FixtureDef();\n');
    log('    fd.friction = %.15f;\n', this.m_friction);
    log('    fd.restitution = %.15f;\n', this.m_restitution);
    log('    fd.density = %.15f;\n', this.m_density);
    log('    fd.isSensor = %s;\n', (this.m_isSensor) ? ('true') : ('false'));
    log('    fd.filter.categoryBits = %d;\n', this.m_filter.categoryBits);
    log('    fd.filter.maskBits = %d;\n', this.m_filter.maskBits);
    log('    fd.filter.groupIndex = %d;\n', this.m_filter.groupIndex);

    this.m_shape.Dump(log);

    log('\n');
    log('    fd.shape = shape;\n');
    log('\n');
    log('    bodies[%d].CreateFixture(fd);\n', bodyIndex);
  }

  // We need separation create/destroy functions from the constructor/destructor because
  // the destructor cannot access the allocator (no destructor arguments allowed by C++).
  public Create(body: B2Body, def: B2FixtureDef): void {
    this.m_userData = def.userData;
    this.m_friction = def.friction;
    this.m_restitution = def.restitution;

    this.m_body = body;
    this.m_next = null;

    this.m_filter.Copy(def.filter);

    this.m_isSensor = def.isSensor;

    this.m_shape = def.shape.Clone();

    // Reserve proxy space
    // const childCount = m_shape->GetChildCount();
    // m_proxies = (B2FixtureProxy*)allocator->Allocate(childCount * sizeof(B2FixtureProxy));
    // for (int32 i = 0; i < childCount; ++i)
    // {
    //   m_proxies[i].fixture = NULL;
    //   m_proxies[i].proxyId = B2BroadPhase::e_nullProxy;
    // }
    this.m_proxies = B2FixtureProxy.MakeArray(this.m_shape.GetChildCount());
    this.m_proxyCount = 0;

    this.m_density = def.density;
  }

  public Destroy(): void {
    // The proxies must be destroyed before calling this.
    /// b2Assert(this.m_proxyCount === 0);

    // Free the proxy array.
    // int32 childCount = m_shape->GetChildCount();
    // allocator->Free(m_proxies, childCount * sizeof(B2FixtureProxy));
    // m_proxies = NULL;

    this.m_shape = null;
  }

  // These support body activation/deactivation.
  public CreateProxies(broadPhase: B2BroadPhase, xf: B2Transform): void {
    /// b2Assert(this.m_proxyCount === 0);

    // Create proxies in the broad-phase.
    this.m_proxyCount = this.m_shape.GetChildCount();

    for (let i: number = 0; i < this.m_proxyCount; ++i) {
      const proxy = this.m_proxies[i];
      this.m_shape.ComputeAABB(proxy.aabb, xf, i);
      proxy.proxy = broadPhase.CreateProxy(proxy.aabb, proxy);
      proxy.fixture = this;
      proxy.childIndex = i;
    }
  }

  public DestroyProxies(broadPhase: B2BroadPhase): void {
    // Destroy proxies in the broad-phase.
    for (let i: number = 0; i < this.m_proxyCount; ++i) {
      const proxy = this.m_proxies[i];
      broadPhase.DestroyProxy(proxy.proxy);
      proxy.proxy = null;
    }

    this.m_proxyCount = 0;
  }

  private static Synchronize_s_aabb1 = new B2AABB();
  private static Synchronize_s_aabb2 = new B2AABB();
  private static Synchronize_s_displacement = new B2Vec2();
  public Synchronize(broadPhase: B2BroadPhase, transform1: B2Transform, transform2: B2Transform): void {
    if (this.m_proxyCount === 0) {
      return;
    }

    for (let i: number = 0; i < this.m_proxyCount; ++i) {
      const proxy = this.m_proxies[i];

      // Compute an AABB that covers the swept shape (may miss some rotation effect).
      const aabb1 = B2Fixture.Synchronize_s_aabb1;
      const aabb2 = B2Fixture.Synchronize_s_aabb2;
      this.m_shape.ComputeAABB(aabb1, transform1, i);
      this.m_shape.ComputeAABB(aabb2, transform2, i);

      proxy.aabb.Combine2(aabb1, aabb2);

      const displacement: B2Vec2 = B2Vec2.SubVV(transform2.p, transform1.p, B2Fixture.Synchronize_s_displacement);

      broadPhase.MoveProxy(proxy.proxy, proxy.aabb, displacement);
    }
  }
}
