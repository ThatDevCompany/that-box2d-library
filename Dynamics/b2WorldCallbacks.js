"use strict";
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
Object.defineProperty(exports, "__esModule", { value: true });
exports.B2RayCastCallback = exports.B2QueryCallback = exports.B2ContactListener = exports.B2ContactImpulse = exports.B2ContactFilter = exports.B2DestructionListener = void 0;
const b2Settings_1 = require("../Common/b2Settings");
/// #endif
///  Joints and fixtures are destroyed when their associated
///  body is destroyed. Implement this listener so that you
///  may nullify references to these joints and shapes.
class B2DestructionListener {
    ///  Called when any joint is about to be destroyed due
    ///  to the destruction of one of its attached bodies.
    SayGoodbyeJoint(joint) { }
    ///  Called when any fixture is about to be destroyed due
    ///  to the destruction of its parent body.
    SayGoodbyeFixture(fixture) { }
    /// #if B2_ENABLE_PARTICLE
    ///  Called when any particle group is about to be destroyed.
    SayGoodbyeParticleGroup(group) { }
    ///  Called when a particle is about to be destroyed.
    ///  The index can be used in conjunction with
    ///  B2ParticleSystem::GetUserDataBuffer() or
    ///  B2ParticleSystem::GetParticleHandleFromIndex() to determine which
    ///  particle has been destroyed.
    SayGoodbyeParticle(system, index) { }
}
exports.B2DestructionListener = B2DestructionListener;
///  Implement this class to provide collision filtering. In other words, you can implement
///  this class if you want finer control over contact creation.
class B2ContactFilter {
    ///  Return true if contact calculations should be performed between these two shapes.
    ///  @warning for performance reasons this is only called when the AABBs begin to overlap.
    ShouldCollide(fixtureA, fixtureB) {
        const bodyA = fixtureA.GetBody();
        const bodyB = fixtureB.GetBody();
        // At least one body should be dynamic or kinematic.
        if (bodyB.GetType() === 0 /* B2_staticBody */ &&
            bodyA.GetType() === 0 /* B2_staticBody */) {
            return false;
        }
        // Does a joint prevent collision?
        if (!bodyB.ShouldCollideConnected(bodyA)) {
            return false;
        }
        const filter1 = fixtureA.GetFilterData();
        const filter2 = fixtureB.GetFilterData();
        if (filter1.groupIndex === filter2.groupIndex && filter1.groupIndex !== 0) {
            return filter1.groupIndex > 0;
        }
        const collide = (filter1.maskBits & filter2.categoryBits) !== 0 &&
            (filter1.categoryBits & filter2.maskBits) !== 0;
        return collide;
    }
    /// #if B2_ENABLE_PARTICLE
    ShouldCollideFixtureParticle(fixture, system, index) {
        return true;
    }
    ShouldCollideParticleParticle(system, indexA, indexB) {
        return true;
    }
}
exports.B2ContactFilter = B2ContactFilter;
/// #endif
B2ContactFilter.B2_defaultFilter = new B2ContactFilter();
///  Contact impulses for reporting. Impulses are used instead of forces because
///  sub-step forces may approach infinity for rigid body collisions. These
///  match up one-to-one with the contact points in B2Manifold.
class B2ContactImpulse {
    constructor() {
        this.normalImpulses = b2Settings_1.B2MakeNumberArray(b2Settings_1.B2_maxManifoldPoints);
        this.tangentImpulses = b2Settings_1.B2MakeNumberArray(b2Settings_1.B2_maxManifoldPoints);
        this.count = 0;
    }
}
exports.B2ContactImpulse = B2ContactImpulse;
///  Implement this class to get contact information. You can use these results for
///  things like sounds and game logic. You can also get contact results by
///  traversing the contact lists after the time step. However, you might miss
///  some contacts because continuous physics leads to sub-stepping.
///  Additionally you may receive multiple callbacks for the same contact in a
///  single time step.
///  You should strive to make your callbacks efficient because there may be
///  many callbacks per time step.
///  @warning You cannot create/destroy Box2D entities inside these callbacks.
class B2ContactListener {
    ///  Called when two fixtures begin to touch.
    BeginContact(contact) { }
    ///  Called when two fixtures continue to touch
    ContinueContact(contact) { }
    ///  Called when two fixtures cease to touch.
    EndContact(contact) { }
    /// #if B2_ENABLE_PARTICLE
    BeginContactFixtureParticle(system, contact) { }
    EndContactFixtureParticle(system, contact) { }
    BeginContactParticleParticle(system, contact) { }
    EndContactParticleParticle(system, contact) { }
    /// #endif
    ///  This is called after a contact is updated. This allows you to inspect a
    ///  contact before it goes to the solver. If you are careful, you can modify the
    ///  contact manifold (e.g. disable contact).
    ///  A copy of the old manifold is provided so that you can detect changes.
    ///  Note: this is called only for awake bodies.
    ///  Note: this is called even when the number of contact points is zero.
    ///  Note: this is not called for sensors.
    ///  Note: if you set the number of contact points to zero, you will not
    ///  get an EndContact callback. However, you may get a BeginContact callback
    ///  the next step.
    PreSolve(contact, oldManifold) { }
    ///  This lets you inspect a contact after the solver is finished. This is useful
    ///  for inspecting impulses.
    ///  Note: the contact manifold does not include time of impact impulses, which can be
    ///  arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
    ///  in a separate data structure.
    ///  Note: this is only called for contacts that are touching, solid, and awake.
    PostSolve(contact, impulse) { }
}
exports.B2ContactListener = B2ContactListener;
B2ContactListener.B2_defaultListener = new B2ContactListener();
///  Callback class for AABB queries.
///  See B2World::Query
class B2QueryCallback {
    ///  Called for each fixture found in the query AABB.
    ///  @return false to terminate the query.
    ReportFixture(fixture) {
        return true;
    }
    /// #if B2_ENABLE_PARTICLE
    ReportParticle(system, index) {
        return false;
    }
    ShouldQueryParticleSystem(system) {
        return true;
    }
}
exports.B2QueryCallback = B2QueryCallback;
///  Callback class for ray casts.
///  See B2World::RayCast
class B2RayCastCallback {
    ///  Called for each fixture found in the query. You control how the ray cast
    ///  proceeds by returning a float:
    ///  return -1: ignore this fixture and continue
    ///  return 0: terminate the ray cast
    ///  return fraction: clip the ray to this point
    ///  return 1: don't clip the ray and continue
    ///  @param fixture the fixture hit by the ray
    ///  @param point the point of initial intersection
    ///  @param normal the normal vector at the point of intersection
    ///  @return -1 to filter, 0 to terminate, fraction to clip the ray for
    ///  closest hit, 1 to continue
    ReportFixture(fixture, point, normal, fraction) {
        return fraction;
    }
    /// #if B2_ENABLE_PARTICLE
    ReportParticle(system, index, point, normal, fraction) {
        return 0;
    }
    ShouldQueryParticleSystem(system) {
        return true;
    }
}
exports.B2RayCastCallback = B2RayCastCallback;
