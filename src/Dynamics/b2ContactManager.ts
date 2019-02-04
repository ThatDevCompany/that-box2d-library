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

import { B2BroadPhase } from '../Collision/b2BroadPhase'
import { B2TreeNode } from '../Collision/b2DynamicTree'
import { B2Contact, B2ContactEdge } from './Contacts/b2Contact'
import { B2ContactFactory } from './Contacts/b2ContactFactory'
import { B2Body, B2BodyType } from './b2Body'
import { B2Fixture, B2FixtureProxy } from './b2Fixture'
import { B2ContactFilter, B2ContactListener } from './b2WorldCallbacks'

// Delegate of B2World.
export class B2ContactManager {
	public m_broadPhase: B2BroadPhase = new B2BroadPhase()
	public m_contactList: B2Contact = null
	public m_contactCount: number = 0
	public m_contactFilter: B2ContactFilter = B2ContactFilter.B2_defaultFilter
	public m_contactListener: B2ContactListener =
		B2ContactListener.B2_defaultListener
	public m_allocator: any = null

	public m_contactFactory: B2ContactFactory = null

	constructor() {
		this.m_contactFactory = new B2ContactFactory(this.m_allocator)
	}

	// Broad-phase callback.
	public AddPair(proxyUserDataA: any, proxyUserDataB: any): void {
		/// b2Assert(proxyUserDataA instanceof B2FixtureProxy);
		/// b2Assert(proxyUserDataB instanceof B2FixtureProxy);
		const proxyA: B2FixtureProxy = <B2FixtureProxy>proxyUserDataA // (proxyUserDataA instanceof B2FixtureProxy ? proxyUserDataA : null);
		const proxyB: B2FixtureProxy = <B2FixtureProxy>proxyUserDataB // (proxyUserDataB instanceof B2FixtureProxy ? proxyUserDataB : null);

		let fixtureA: B2Fixture = proxyA.fixture
		let fixtureB: B2Fixture = proxyB.fixture

		let indexA: number = proxyA.childIndex
		let indexB: number = proxyB.childIndex

		let bodyA: B2Body = fixtureA.GetBody()
		let bodyB: B2Body = fixtureB.GetBody()

		// Are the fixtures on the same body?
		if (bodyA === bodyB) {
			return
		}

		// TODO_ERIN use a hash table to remove a potential bottleneck when both
		// bodies have a lot of contacts.
		// Does a contact already exist?
		let edge: B2ContactEdge = bodyB.GetContactList()
		while (edge) {
			if (edge.other === bodyA) {
				const fA: B2Fixture = edge.contact.GetFixtureA()
				const fB: B2Fixture = edge.contact.GetFixtureB()
				const iA: number = edge.contact.GetChildIndexA()
				const iB: number = edge.contact.GetChildIndexB()

				if (
					fA === fixtureA &&
					fB === fixtureB &&
					iA === indexA &&
					iB === indexB
				) {
					// A contact already exists.
					return
				}

				if (
					fA === fixtureB &&
					fB === fixtureA &&
					iA === indexB &&
					iB === indexA
				) {
					// A contact already exists.
					return
				}
			}

			edge = edge.next
		}

		// Check user filtering.
		if (
			this.m_contactFilter &&
			!this.m_contactFilter.ShouldCollide(fixtureA, fixtureB)
		) {
			return
		}

		// Call the factory.
		const c: B2Contact = this.m_contactFactory.Create(
			fixtureA,
			indexA,
			fixtureB,
			indexB
		)
		if (c === null) {
			return
		}

		// Contact creation may swap fixtures.
		fixtureA = c.GetFixtureA()
		fixtureB = c.GetFixtureB()
		indexA = c.GetChildIndexA()
		indexB = c.GetChildIndexB()
		bodyA = fixtureA.m_body
		bodyB = fixtureB.m_body

		// Insert into the world.
		c.m_prev = null
		c.m_next = this.m_contactList
		if (this.m_contactList !== null) {
			this.m_contactList.m_prev = c
		}
		this.m_contactList = c

		// Connect to island graph.

		// Connect to body A
		c.m_nodeA.contact = c
		c.m_nodeA.other = bodyB

		c.m_nodeA.prev = null
		c.m_nodeA.next = bodyA.m_contactList
		if (bodyA.m_contactList !== null) {
			bodyA.m_contactList.prev = c.m_nodeA
		}
		bodyA.m_contactList = c.m_nodeA

		// Connect to body B
		c.m_nodeB.contact = c
		c.m_nodeB.other = bodyA

		c.m_nodeB.prev = null
		c.m_nodeB.next = bodyB.m_contactList
		if (bodyB.m_contactList !== null) {
			bodyB.m_contactList.prev = c.m_nodeB
		}
		bodyB.m_contactList = c.m_nodeB

		// Wake up the bodies
		if (!fixtureA.IsSensor() && !fixtureB.IsSensor()) {
			bodyA.SetAwake(true)
			bodyB.SetAwake(true)
		}

		++this.m_contactCount
	}

	public FindNewContacts(): void {
		this.m_broadPhase.UpdatePairs(this)
	}

	public Destroy(c: B2Contact): void {
		const fixtureA: B2Fixture = c.GetFixtureA()
		const fixtureB: B2Fixture = c.GetFixtureB()
		const bodyA: B2Body = fixtureA.GetBody()
		const bodyB: B2Body = fixtureB.GetBody()

		if (this.m_contactListener && c.IsTouching()) {
			this.m_contactListener.EndContact(c)
		}

		// Remove from the world.
		if (c.m_prev) {
			c.m_prev.m_next = c.m_next
		}

		if (c.m_next) {
			c.m_next.m_prev = c.m_prev
		}

		if (c === this.m_contactList) {
			this.m_contactList = c.m_next
		}

		// Remove from body 1
		if (c.m_nodeA.prev) {
			c.m_nodeA.prev.next = c.m_nodeA.next
		}

		if (c.m_nodeA.next) {
			c.m_nodeA.next.prev = c.m_nodeA.prev
		}

		if (c.m_nodeA === bodyA.m_contactList) {
			bodyA.m_contactList = c.m_nodeA.next
		}

		// Remove from body 2
		if (c.m_nodeB.prev) {
			c.m_nodeB.prev.next = c.m_nodeB.next
		}

		if (c.m_nodeB.next) {
			c.m_nodeB.next.prev = c.m_nodeB.prev
		}

		if (c.m_nodeB === bodyB.m_contactList) {
			bodyB.m_contactList = c.m_nodeB.next
		}

		// Call the factory.
		this.m_contactFactory.Destroy(c)
		--this.m_contactCount
	}

	// This is the top level collision call for the time step. Here
	// all the narrow phase collision is processed for the world
	// contact list.
	public Collide(): void {
		// Update awake contacts.
		let c: B2Contact = this.m_contactList
		while (c) {
			const fixtureA: B2Fixture = c.GetFixtureA()
			const fixtureB: B2Fixture = c.GetFixtureB()
			const indexA: number = c.GetChildIndexA()
			const indexB: number = c.GetChildIndexB()
			const bodyA: B2Body = fixtureA.GetBody()
			const bodyB: B2Body = fixtureB.GetBody()

			// Is this contact flagged for filtering?
			if (c.m_filterFlag) {
				// Check user filtering.
				if (
					this.m_contactFilter &&
					!this.m_contactFilter.ShouldCollide(fixtureA, fixtureB)
				) {
					const cNuke: B2Contact = c
					c = cNuke.m_next
					this.Destroy(cNuke)
					continue
				}

				// Clear the filtering flag.
				c.m_filterFlag = false
			}

			const activeA: boolean =
				bodyA.IsAwake() && bodyA.m_type !== B2BodyType.B2_staticBody
			const activeB: boolean =
				bodyB.IsAwake() && bodyB.m_type !== B2BodyType.B2_staticBody

			// At least one body must be awake and it must be dynamic or kinematic.
			if (!activeA && !activeB) {
				c = c.m_next
				continue
			}

			const proxyA: B2TreeNode = fixtureA.m_proxies[indexA].proxy
			const proxyB: B2TreeNode = fixtureB.m_proxies[indexB].proxy
			const overlap: boolean = this.m_broadPhase.TestOverlap(proxyA, proxyB)

			// Here we destroy contacts that cease to overlap in the broad-phase.
			if (!overlap) {
				const cNuke: B2Contact = c
				c = cNuke.m_next
				this.Destroy(cNuke)
				continue
			}

			// The contact persists.
			c.Update(this.m_contactListener)
			c = c.m_next
		}
	}
}
