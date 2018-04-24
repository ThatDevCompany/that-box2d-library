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

import {B2Transform} from '../../Common/b2Math';
import {B2CollidePolygons} from '../../Collision/b2CollidePolygon';
import {B2Manifold} from '../../Collision/b2Collision';
import {B2Shape} from '../../Collision/Shapes/b2Shape';
import {B2PolygonShape} from '../../Collision/Shapes/b2PolygonShape';
import {B2Contact} from './b2Contact';
import {B2Fixture} from '../b2Fixture';

export class B2PolygonContact extends B2Contact {
	constructor() {
		super();
	}

	public static Create(allocator: any): B2Contact {
		return new B2PolygonContact();
	}

	public static Destroy(contact: B2Contact, allocator: any): void {
	}

	public Reset(fixtureA: B2Fixture, indexA: number, fixtureB: B2Fixture, indexB: number): void {
		super.Reset(fixtureA, indexA, fixtureB, indexB);
	}

	public Evaluate(manifold: B2Manifold, xfA: B2Transform, xfB: B2Transform): void {
		const shapeA: B2Shape = this.m_fixtureA.GetShape();
		const shapeB: B2Shape = this.m_fixtureB.GetShape();
		/// b2Assert(shapeA instanceof B2PolygonShape);
		/// b2Assert(shapeB instanceof B2PolygonShape);
		B2CollidePolygons(
			manifold,
			<B2PolygonShape> shapeA, xfA,
			<B2PolygonShape> shapeB, xfB);
	}
}
