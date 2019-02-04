/*
 * Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

import { B2MakeArray } from '../Common/b2Settings'
import { B2Vec2 } from '../Common/b2Math'

///  Profiling data. Times are in milliseconds.
export class B2Profile {
	public step: number = 0
	public collide: number = 0
	public solve: number = 0
	public solveInit: number = 0
	public solveVelocity: number = 0
	public solvePosition: number = 0
	public broadphase: number = 0
	public solveTOI: number = 0

	public Reset() {
		this.step = 0
		this.collide = 0
		this.solve = 0
		this.solveInit = 0
		this.solveVelocity = 0
		this.solvePosition = 0
		this.broadphase = 0
		this.solveTOI = 0
		return this
	}
}

///  This is an internal structure.
export class B2TimeStep {
	public dt: number = 0 // time step
	public inv_dt: number = 0 // inverse time step (0 if dt == 0).
	public dtRatio: number = 0 // dt * inv_dt0
	public velocityIterations: number = 0
	public positionIterations: number = 0
	/// #if B2_ENABLE_PARTICLE
	public particleIterations: number = 0
	/// #endif
	public warmStarting: boolean = false

	public Copy(step: B2TimeStep): B2TimeStep {
		this.dt = step.dt
		this.inv_dt = step.inv_dt
		this.dtRatio = step.dtRatio
		this.positionIterations = step.positionIterations
		this.velocityIterations = step.velocityIterations
		/// #if B2_ENABLE_PARTICLE
		this.particleIterations = step.particleIterations
		/// #endif
		this.warmStarting = step.warmStarting
		return this
	}
}

export class B2Position {
	public c: B2Vec2 = new B2Vec2()
	public a: number = 0

	public static MakeArray(length: number): B2Position[] {
		return B2MakeArray(length, function(i: number): B2Position {
			return new B2Position()
		})
	}
}

export class B2Velocity {
	public v: B2Vec2 = new B2Vec2()
	public w: number = 0

	public static MakeArray(length: number): B2Velocity[] {
		return B2MakeArray(length, function(i: number): B2Velocity {
			return new B2Velocity()
		})
	}
}

export class B2SolverData {
	public step: B2TimeStep = new B2TimeStep()
	public positions: B2Position[] = null
	public velocities: B2Velocity[] = null
}
