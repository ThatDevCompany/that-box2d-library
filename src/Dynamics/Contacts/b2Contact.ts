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

import {B2_linearSlop} from '../../Common/b2Settings';
import {B2Sqrt, B2Transform, B2Sweep} from '../../Common/b2Math';
import {B2Manifold, B2WorldManifold, B2ManifoldPoint, B2ContactID} from '../../Collision/b2Collision';
import {B2TestOverlapShape} from '../../Collision/b2Collision';
import {B2TimeOfImpact, B2TOIInput, B2TOIOutput} from '../../Collision/b2TimeOfImpact';
import {B2Body} from '../b2Body';
import {B2Fixture} from '../b2Fixture';
import {B2Shape} from '../../Collision/Shapes/b2Shape';
import {B2ContactListener} from '../b2WorldCallbacks';

///  Friction mixing law. The idea is to allow either fixture to drive the restitution to zero.
///  For example, anything slides on ice.
export function B2MixFriction(friction1: number, friction2: number): number {
	return B2Sqrt(friction1 * friction2);
}

///  Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
///  For example, a superball bounces on anything.
export function B2MixRestitution(restitution1: number, restitution2: number): number {
	return restitution1 > restitution2 ? restitution1 : restitution2;
}

export class B2ContactEdge {
	public other: B2Body | null = null; /// < provides quick access to the other body attached.
	public contact: B2Contact | null = null; /// < the contact
	public prev: B2ContactEdge | null = null; /// < the previous contact edge in the body's contact list
	public next: B2ContactEdge | null = null; /// < the next contact edge in the body's contact list
}

export class B2Contact {
	public m_islandFlag: boolean = false; ///  Used when crawling contact graph when forming islands.
	public m_touchingFlag: boolean = false; ///  Set when the shapes are touching.
	public m_enabledFlag: boolean = false; ///  This contact can be disabled (by user)
	public m_filterFlag: boolean = false; ///  This contact needs filtering because a fixture filter was changed.
	public m_bulletHitFlag: boolean = false; ///  This bullet contact had a TOI event
	public m_toiFlag: boolean = false; ///  This contact has a valid TOI in m_toi

	public m_prev: B2Contact | null = null;
	public m_next: B2Contact | null = null;

	public m_nodeA: B2ContactEdge = new B2ContactEdge();
	public m_nodeB: B2ContactEdge = new B2ContactEdge();

	public m_fixtureA: B2Fixture | null = null;
	public m_fixtureB: B2Fixture | null = null;

	public m_indexA: number = 0;
	public m_indexB: number = 0;

	public m_manifold: B2Manifold = new B2Manifold();

	public m_toiCount: number = 0;
	public m_toi: number = 0;

	public m_friction: number = 0;
	public m_restitution: number = 0;

	public m_tangentSpeed: number = 0;

	public m_oldManifold: B2Manifold = new B2Manifold();

	public GetManifold() {
		return this.m_manifold;
	}

	public GetWorldManifold(worldManifold: B2WorldManifold): void {
		const bodyA: B2Body = this.m_fixtureA.GetBody();
		const bodyB: B2Body = this.m_fixtureB.GetBody();
		const shapeA: B2Shape = this.m_fixtureA.GetShape();
		const shapeB: B2Shape = this.m_fixtureB.GetShape();
		worldManifold.Initialize(this.m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
	}

	public IsTouching(): boolean {
		return this.m_touchingFlag;
	}

	public SetEnabled(flag: boolean): void {
		this.m_enabledFlag = flag;
	}

	public IsEnabled(): boolean {
		return this.m_enabledFlag;
	}

	public GetNext(): B2Contact | null {
		return this.m_next;
	}

	public GetFixtureA(): B2Fixture | null {
		return this.m_fixtureA;
	}

	public GetChildIndexA(): number {
		return this.m_indexA;
	}

	public GetFixtureB(): B2Fixture | null {
		return this.m_fixtureB;
	}

	public GetChildIndexB(): number {
		return this.m_indexB;
	}

	public Evaluate(manifold: B2Manifold, xfA: B2Transform, xfB: B2Transform): void {
	}

	public FlagForFiltering(): void {
		this.m_filterFlag = true;
	}

	public SetFriction(friction: number): void {
		this.m_friction = friction;
	}

	public GetFriction(): number {
		return this.m_friction;
	}

	public ResetFriction(): void {
		this.m_friction = B2MixFriction(this.m_fixtureA.m_friction, this.m_fixtureB.m_friction);
	}

	public SetRestitution(restitution: number): void {
		this.m_restitution = restitution;
	}

	public GetRestitution(): number {
		return this.m_restitution;
	}

	public ResetRestitution(): void {
		this.m_restitution = B2MixRestitution(this.m_fixtureA.m_restitution, this.m_fixtureB.m_restitution);
	}

	public SetTangentSpeed(speed: number): void {
		this.m_tangentSpeed = speed;
	}

	public GetTangentSpeed(): number {
		return this.m_tangentSpeed;
	}

	public Reset(fixtureA: B2Fixture, indexA: number, fixtureB: B2Fixture, indexB: number): void {
		this.m_islandFlag = false;
		this.m_touchingFlag = false;
		this.m_enabledFlag = true;
		this.m_filterFlag = false;
		this.m_bulletHitFlag = false;
		this.m_toiFlag = false;

		this.m_fixtureA = fixtureA;
		this.m_fixtureB = fixtureB;

		this.m_indexA = indexA;
		this.m_indexB = indexB;

		this.m_manifold.pointCount = 0;

		this.m_prev = null;
		this.m_next = null;

		this.m_nodeA.contact = null;
		this.m_nodeA.prev = null;
		this.m_nodeA.next = null;
		this.m_nodeA.other = null;

		this.m_nodeB.contact = null;
		this.m_nodeB.prev = null;
		this.m_nodeB.next = null;
		this.m_nodeB.other = null;

		this.m_toiCount = 0;

		this.m_friction = B2MixFriction(this.m_fixtureA.m_friction, this.m_fixtureB.m_friction);
		this.m_restitution = B2MixRestitution(this.m_fixtureA.m_restitution, this.m_fixtureB.m_restitution);
	}

	public Update(listener: B2ContactListener): void {
		const tManifold: B2Manifold = this.m_oldManifold;
		this.m_oldManifold = this.m_manifold;
		this.m_manifold = tManifold;

		// Re-enable this contact.
		this.m_enabledFlag = true;

		let touching: boolean = false;
		const wasTouching: boolean = this.m_touchingFlag;

		const sensorA: boolean = this.m_fixtureA.IsSensor();
		const sensorB: boolean = this.m_fixtureB.IsSensor();
		const sensor: boolean = sensorA || sensorB;

		const bodyA: B2Body = this.m_fixtureA.GetBody();
		const bodyB: B2Body = this.m_fixtureB.GetBody();
		const xfA: B2Transform = bodyA.GetTransform();
		const xfB: B2Transform = bodyB.GetTransform();

		/// const aabbOverlap = B2TestOverlapAABB(this.m_fixtureA.GetAABB(0), this.m_fixtureB.GetAABB(0));

		// Is this contact a sensor?
		if (sensor) {
			/// if (aabbOverlap)
			/// {
			const shapeA: B2Shape = this.m_fixtureA.GetShape();
			const shapeB: B2Shape = this.m_fixtureB.GetShape();
			touching = B2TestOverlapShape(shapeA, this.m_indexA, shapeB, this.m_indexB, xfA, xfB);
			/// }

			// Sensors DO generate manifolds.
			this.Evaluate(this.m_manifold, xfA, xfB);

		} else {
			/// if (aabbOverlap)
			/// {
			this.Evaluate(this.m_manifold, xfA, xfB);
			touching = this.m_manifold.pointCount > 0;

			// Match old contact ids to new contact ids and copy the
			// stored impulses to warm start the solver.
			for (let i: number = 0; i < this.m_manifold.pointCount; ++i) {
				const mp2: B2ManifoldPoint = this.m_manifold.points[i];
				mp2.normalImpulse = 0;
				mp2.tangentImpulse = 0;
				const id2: B2ContactID = mp2.id;

				for (let j: number = 0; j < this.m_oldManifold.pointCount; ++j) {
					const mp1: B2ManifoldPoint = this.m_oldManifold.points[j];

					if (mp1.id.key === id2.key) {
						mp2.normalImpulse = mp1.normalImpulse;
						mp2.tangentImpulse = mp1.tangentImpulse;
						break;
					}
				}
			}
			/// }
			/// else
			/// {
			///   this.m_manifold.pointCount = 0;
			/// }

			if (touching !== wasTouching) {
				bodyA.SetAwake(true);
				bodyB.SetAwake(true);
			}
		}

		this.m_touchingFlag = touching;

		if (!wasTouching && touching && listener) {
			listener.BeginContact(this);
		}

		if (wasTouching && !touching && listener) {
			listener.EndContact(this);
		}

		if (!sensor && touching && listener) {
			listener.PreSolve(this, this.m_oldManifold);
		}
	}

	private static ComputeTOI_s_input = new B2TOIInput();
	private static ComputeTOI_s_output = new B2TOIOutput();

	public ComputeTOI(sweepA: B2Sweep, sweepB: B2Sweep): number {
		const input: B2TOIInput = B2Contact.ComputeTOI_s_input;
		input.proxyA.SetShape(this.m_fixtureA.GetShape(), this.m_indexA);
		input.proxyB.SetShape(this.m_fixtureB.GetShape(), this.m_indexB);
		input.sweepA.Copy(sweepA);
		input.sweepB.Copy(sweepB);
		input.tMax = B2_linearSlop;

		const output: B2TOIOutput = B2Contact.ComputeTOI_s_output;

		B2TimeOfImpact(output, input);

		return output.t;
	}
}
