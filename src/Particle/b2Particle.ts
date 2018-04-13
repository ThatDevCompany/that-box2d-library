/*
 * Copyright (c) 2013 Google, Inc.
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

/// #if B2_ENABLE_PARTICLE

import { B2_invalidParticleIndex } from '../Common/b2Settings';
import { B2Clamp, B2Vec2 } from '../Common/b2Math';
import { B2Color } from '../Common/b2Draw';
import { B2ParticleGroup } from './b2ParticleGroup';

/**
 * The particle type. Can be combined with the | operator.
 */
export const enum B2ParticleFlag {
  ///  Water particle.
  B2_waterParticle = 0,
  ///  Removed after next simulation step.
  B2_zombieParticle = 1 << 1,
  ///  Zero velocity.
  B2_wallParticle = 1 << 2,
  ///  With restitution from stretching.
  B2_springParticle = 1 << 3,
  ///  With restitution from deformation.
  B2_elasticParticle = 1 << 4,
  ///  With viscosity.
  B2_viscousParticle = 1 << 5,
  ///  Without isotropic pressure.
  B2_powderParticle = 1 << 6,
  ///  With surface tension.
  B2_tensileParticle = 1 << 7,
  ///  Mix color between contacting particles.
  B2_colorMixingParticle = 1 << 8,
  ///  Call B2DestructionListener on destruction.
  B2_destructionListenerParticle = 1 << 9,
  ///  Prevents other particles from leaking.
  B2_barrierParticle = 1 << 10,
  ///  Less compressibility.
  B2_staticPressureParticle = 1 << 11,
  ///  Makes pairs or triads with other particles.
  B2_reactiveParticle = 1 << 12,
  ///  With high repulsive force.
  B2_repulsiveParticle = 1 << 13,
  ///  Call B2ContactListener when this particle is about to interact with
  ///  a rigid body or stops interacting with a rigid body.
  ///  This results in an expensive operation compared to using
  ///  B2_fixtureContactFilterParticle to detect collisions between
  ///  particles.
  B2_fixtureContactListenerParticle = 1 << 14,
  ///  Call B2ContactListener when this particle is about to interact with
  ///  another particle or stops interacting with another particle.
  ///  This results in an expensive operation compared to using
  ///  B2_particleContactFilterParticle to detect collisions between
  ///  particles.
  B2_particleContactListenerParticle = 1 << 15,
  ///  Call B2ContactFilter when this particle interacts with rigid bodies.
  B2_fixtureContactFilterParticle = 1 << 16,
  ///  Call B2ContactFilter when this particle interacts with other
  ///  particles.
  B2_particleContactFilterParticle = 1 << 17
}

export class B2ParticleDef {
  flags: B2ParticleFlag = 0;
  position: B2Vec2 = new B2Vec2();
  velocity: B2Vec2 = new B2Vec2();
  color: B2Color = new B2Color();
  lifetime: number = 0.0;
  userData: any = null;
  group: B2ParticleGroup = null;
}

export function B2CalculateParticleIterations(gravity: number, radius: number, timeStep: number): number {
  // In some situations you may want more particle iterations than this,
  // but to avoid excessive cycle cost, don't recommend more than this.
  const B2_MAX_RECOMMENDED_PARTICLE_ITERATIONS = 8;
  const B2_RADIUS_THRESHOLD = 0.01;
  const iterations = Math.ceil(Math.sqrt(gravity / (B2_RADIUS_THRESHOLD * radius)) * timeStep);
  return B2Clamp(iterations, 1, B2_MAX_RECOMMENDED_PARTICLE_ITERATIONS);
}

export class B2ParticleHandle {
  public m_index: number = B2_invalidParticleIndex;
  public GetIndex(): number { return this.m_index; }
  public SetIndex(index: number): void { this.m_index = index; }
}

/// #endif
