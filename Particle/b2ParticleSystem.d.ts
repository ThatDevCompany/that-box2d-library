import { B2Vec2, B2Rot, B2Transform } from '../Common/b2Math';
import { B2Color } from '../Common/b2Draw';
import { B2AABB, B2RayCastInput, B2RayCastOutput } from '../Collision/b2Collision';
import { B2Shape, B2MassData } from '../Collision/Shapes/b2Shape';
import { B2EdgeShape } from '../Collision/Shapes/b2EdgeShape';
import { B2TimeStep } from '../Dynamics/b2TimeStep';
import { B2Fixture } from '../Dynamics/b2Fixture';
import { B2Body } from '../Dynamics/b2Body';
import { B2World } from '../Dynamics/b2World';
import { B2ContactFilter, B2ContactListener, B2QueryCallback, B2RayCastCallback } from '../Dynamics/b2WorldCallbacks';
import { B2ParticleFlag, B2ParticleDef, B2ParticleHandle } from './b2Particle';
import { B2ParticleGroupFlag, B2ParticleGroupDef, B2ParticleGroup } from './b2ParticleGroup';
import { B2DistanceProxy } from '../Collision/b2Distance';
export declare class B2GrowableBuffer<T> {
    data: T[];
    count: number;
    capacity: number;
    allocator: () => T;
    constructor(allocator: () => T);
    Append(): number;
    Reserve(newCapacity: number): void;
    Grow(): void;
    Free(): void;
    Shorten(newEnd: number): void;
    Data(): T[];
    GetCount(): number;
    SetCount(newCount: number): void;
    GetCapacity(): number;
    RemoveIf(pred: (t: T) => boolean): void;
    Unique(pred: (a: T, b: T) => boolean): void;
}
export declare type B2ParticleIndex = number;
export declare class B2FixtureParticleQueryCallback extends B2QueryCallback {
    m_system: B2ParticleSystem;
    constructor(system: B2ParticleSystem);
    ShouldQueryParticleSystem(system: B2ParticleSystem): boolean;
    ReportFixture(fixture: B2Fixture): boolean;
    ReportParticle(system: B2ParticleSystem, index: number): boolean;
    ReportFixtureAndParticle(fixture: B2Fixture, childIndex: number, index: number): void;
}
export declare class B2ParticleContact {
    indexA: number;
    indexB: number;
    weight: number;
    normal: B2Vec2;
    flags: B2ParticleFlag;
    SetIndices(a: number, b: number): void;
    SetWeight(w: number): void;
    SetNormal(n: B2Vec2): void;
    SetFlags(f: B2ParticleFlag): void;
    GetIndexA(): number;
    GetIndexB(): number;
    GetWeight(): number;
    GetNormal(): B2Vec2;
    GetFlags(): B2ParticleFlag;
    IsEqual(rhs: B2ParticleContact): boolean;
    IsNotEqual(rhs: B2ParticleContact): boolean;
    ApproximatelyEqual(rhs: B2ParticleContact): boolean;
}
export declare class B2ParticleBodyContact {
    index: number;
    body: B2Body;
    fixture: B2Fixture;
    weight: number;
    normal: B2Vec2;
    mass: number;
}
export declare class B2ParticlePair {
    indexA: number;
    indexB: number;
    flags: B2ParticleFlag;
    strength: number;
    distance: number;
}
export declare class B2ParticleTriad {
    indexA: number;
    indexB: number;
    indexC: number;
    flags: B2ParticleFlag;
    strength: number;
    pa: B2Vec2;
    pb: B2Vec2;
    pc: B2Vec2;
    ka: number;
    kb: number;
    kc: number;
    s: number;
}
export declare class B2ParticleSystemDef {
    /**
     * Enable strict Particle/Body contact check.
     * See SetStrictContactCheck for details.
     */
    strictContactCheck: boolean;
    /**
     * Set the particle density.
     * See SetDensity for details.
     */
    density: number;
    /**
     * Change the particle gravity scale. Adjusts the effect of the
     * global gravity vector on particles. Default value is 1.0f.
     */
    gravityScale: number;
    /**
     * Particles behave as circles with this radius. In Box2D units.
     */
    radius: number;
    /**
     * Set the maximum number of particles.
     * By default, there is no maximum. The particle buffers can
     * continue to grow while B2World's block allocator still has
     * memory.
     * See SetMaxParticleCount for details.
     */
    maxCount: number;
    /**
     * Increases pressure in response to compression
     * Smaller values allow more compression
     */
    pressureStrength: number;
    /**
     * Reduces velocity along the collision normal
     * Smaller value reduces less
     */
    dampingStrength: number;
    /**
     * Restores shape of elastic particle groups
     * Larger values increase elastic particle velocity
     */
    elasticStrength: number;
    /**
     * Restores length of spring particle groups
     * Larger values increase spring particle velocity
     */
    springStrength: number;
    /**
     * Reduces relative velocity of viscous particles
     * Larger values slow down viscous particles more
     */
    viscousStrength: number;
    /**
     * Produces pressure on tensile particles
     * 0~0.2. Larger values increase the amount of surface tension.
     */
    surfaceTensionPressureStrength: number;
    /**
     * Smoothes outline of tensile particles
     * 0~0.2. Larger values result in rounder, smoother,
     * water-drop-like clusters of particles.
     */
    surfaceTensionNormalStrength: number;
    /**
     * Produces additional pressure on repulsive particles
     * Larger values repulse more
     * Negative values mean attraction. The range where particles
     * behave stably is about -0.2 to 2.0.
     */
    repulsiveStrength: number;
    /**
     * Produces repulsion between powder particles
     * Larger values repulse more
     */
    powderStrength: number;
    /**
     * Pushes particles out of solid particle group
     * Larger values repulse more
     */
    ejectionStrength: number;
    /**
     * Produces static pressure
     * Larger values increase the pressure on neighboring partilces
     * For a description of static pressure, see
     * http://en.wikipedia.org/wiki/Static_pressure#Static_pressure_in_fluid_dynamics
     */
    staticPressureStrength: number;
    /**
     * Reduces instability in static pressure calculation
     * Larger values make stabilize static pressure with fewer
     * iterations
     */
    staticPressureRelaxation: number;
    /**
     * Computes static pressure more precisely
     * See SetStaticPressureIterations for details
     */
    staticPressureIterations: number;
    /**
     * Determines how fast colors are mixed
     * 1.0f ==> mixed immediately
     * 0.5f ==> mixed half way each simulation step (see
     * B2World::Step())
     */
    colorMixingStrength: number;
    /**
     * Whether to destroy particles by age when no more particles
     * can be created.  See #b2ParticleSystem::SetDestructionByAge()
     * for more information.
     */
    destroyByAge: boolean;
    /**
     * Granularity of particle lifetimes in seconds.  By default
     * this is set to (1.0f / 60.0f) seconds.  B2ParticleSystem uses
     * a 32-bit signed value to track particle lifetimes so the
     * maximum lifetime of a particle is (2^32 - 1) / (1.0f /
     * lifetimeGranularity) seconds. With the value set to 1/60 the
     * maximum lifetime or age of a particle is 2.27 years.
     */
    lifetimeGranularity: number;
    Copy(def: B2ParticleSystemDef): B2ParticleSystemDef;
    Clone(): B2ParticleSystemDef;
}
export declare class B2ParticleSystem {
    m_paused: boolean;
    m_timestamp: number;
    m_allParticleFlags: B2ParticleFlag;
    m_needsUpdateAllParticleFlags: boolean;
    m_allGroupFlags: B2ParticleGroupFlag;
    m_needsUpdateAllGroupFlags: boolean;
    m_hasForce: boolean;
    m_iterationIndex: number;
    m_inverseDensity: number;
    m_particleDiameter: number;
    m_inverseDiameter: number;
    m_squaredDiameter: number;
    m_count: number;
    m_internalAllocatedCapacity: number;
    /**
     * Allocator for B2ParticleHandle instances.
     */
    /**
     * Maps particle indicies to handles.
     */
    m_handleIndexBuffer: B2ParticleSystem.UserOverridableBuffer<B2ParticleHandle>;
    m_flagsBuffer: B2ParticleSystem.UserOverridableBuffer<B2ParticleFlag>;
    m_positionBuffer: B2ParticleSystem.UserOverridableBuffer<B2Vec2>;
    m_velocityBuffer: B2ParticleSystem.UserOverridableBuffer<B2Vec2>;
    m_forceBuffer: B2Vec2[];
    /**
     * this.m_weightBuffer is populated in ComputeWeight and used in
     * ComputeDepth(), SolveStaticPressure() and SolvePressure().
     */
    m_weightBuffer: number[];
    /**
     * When any particles have the flag B2_staticPressureParticle,
     * this.m_staticPressureBuffer is first allocated and used in
     * SolveStaticPressure() and SolvePressure().  It will be
     * reallocated on subsequent CreateParticle() calls.
     */
    m_staticPressureBuffer: number[];
    /**
     * this.m_accumulationBuffer is used in many functions as a temporary
     * buffer for scalar values.
     */
    m_accumulationBuffer: number[];
    /**
     * When any particles have the flag B2_tensileParticle,
     * this.m_accumulation2Buffer is first allocated and used in
     * SolveTensile() as a temporary buffer for vector values.  It
     * will be reallocated on subsequent CreateParticle() calls.
     */
    m_accumulation2Buffer: B2Vec2[];
    /**
     * When any particle groups have the flag B2_solidParticleGroup,
     * this.m_depthBuffer is first allocated and populated in
     * ComputeDepth() and used in SolveSolid(). It will be
     * reallocated on subsequent CreateParticle() calls.
     */
    m_depthBuffer: number[];
    m_colorBuffer: B2ParticleSystem.UserOverridableBuffer<B2Color>;
    m_groupBuffer: B2ParticleGroup[];
    m_userDataBuffer: B2ParticleSystem.UserOverridableBuffer<any>;
    /**
     * Stuck particle detection parameters and record keeping
     */
    m_stuckThreshold: number;
    m_lastBodyContactStepBuffer: B2ParticleSystem.UserOverridableBuffer<number>;
    m_bodyContactCountBuffer: B2ParticleSystem.UserOverridableBuffer<number>;
    m_consecutiveContactStepsBuffer: B2ParticleSystem.UserOverridableBuffer<number>;
    m_stuckParticleBuffer: B2GrowableBuffer<number>;
    m_proxyBuffer: B2GrowableBuffer<B2ParticleSystem.Proxy>;
    m_contactBuffer: B2GrowableBuffer<B2ParticleContact>;
    m_bodyContactBuffer: B2GrowableBuffer<B2ParticleBodyContact>;
    m_pairBuffer: B2GrowableBuffer<B2ParticlePair>;
    m_triadBuffer: B2GrowableBuffer<B2ParticleTriad>;
    /**
     * Time each particle should be destroyed relative to the last
     * time this.m_timeElapsed was initialized.  Each unit of time
     * corresponds to B2ParticleSystemDef::lifetimeGranularity
     * seconds.
     */
    m_expirationTimeBuffer: B2ParticleSystem.UserOverridableBuffer<number>;
    /**
     * List of particle indices sorted by expiration time.
     */
    m_indexByExpirationTimeBuffer: B2ParticleSystem.UserOverridableBuffer<number>;
    /**
     * Time elapsed in 32:32 fixed point.  Each non-fractional unit
     * of time corresponds to
     * B2ParticleSystemDef::lifetimeGranularity seconds.
     */
    m_timeElapsed: number;
    /**
     * Whether the expiration time buffer has been modified and
     * needs to be resorted.
     */
    m_expirationTimeBufferRequiresSorting: boolean;
    m_groupCount: number;
    m_groupList: B2ParticleGroup;
    m_def: B2ParticleSystemDef;
    m_world: B2World;
    m_prev: B2ParticleSystem;
    m_next: B2ParticleSystem;
    static xTruncBits: number;
    static yTruncBits: number;
    static tagBits: number;
    static yOffset: number;
    static yShift: number;
    static xShift: number;
    static xScale: number;
    static xOffset: number;
    static yMask: number;
    static xMask: number;
    static computeTag(x: number, y: number): number;
    static computeRelativeTag(tag: number, x: number, y: number): number;
    constructor(def: B2ParticleSystemDef, world: B2World);
    Drop(): void;
    /**
     * Create a particle whose properties have been defined.
     *
     * No reference to the definition is retained.
     *
     * A simulation step must occur before it's possible to interact
     * with a newly created particle.  For example,
     * DestroyParticleInShape() will not destroy a particle until
     * B2World::Step() has been called.
     *
     * warning: This function is locked during callbacks.
     */
    CreateParticle(def: B2ParticleDef): number;
    /**
     * Retrieve a handle to the particle at the specified index.
     *
     * Please see #b2ParticleHandle for why you might want a handle.
     */
    GetParticleHandleFromIndex(index: number): B2ParticleHandle;
    /**
     * Destroy a particle.
     *
     * The particle is removed after the next simulation step (see
     * B2World::Step()).
     *
     * @param index Index of the particle to destroy.
     * @param callDestructionListener Whether to call the
     *      destruction listener just before the particle is
     *      destroyed.
     */
    DestroyParticle(index: number, callDestructionListener?: boolean): void;
    /**
     * Destroy the Nth oldest particle in the system.
     *
     * The particle is removed after the next B2World::Step().
     *
     * @param index Index of the Nth oldest particle to
     *      destroy, 0 will destroy the oldest particle in the
     *      system, 1 will destroy the next oldest particle etc.
     * @param callDestructionListener Whether to call the
     *      destruction listener just before the particle is
     *      destroyed.
     */
    DestroyOldestParticle(index: number, callDestructionListener?: boolean): void;
    /**
     * Destroy particles inside a shape.
     *
     * warning: This function is locked during callbacks.
     *
     * In addition, this function immediately destroys particles in
     * the shape in constrast to DestroyParticle() which defers the
     * destruction until the next simulation step.
     *
     * @return Number of particles destroyed.
     * @param shape Shape which encloses particles
     *      that should be destroyed.
     * @param xf Transform applied to the shape.
     * @param callDestructionListener Whether to call the
     *      world B2DestructionListener for each particle
     *      destroyed.
     */
    DestroyParticlesInShape(shape: B2Shape, xf: B2Transform, callDestructionListener?: boolean): number;
    static DestroyParticlesInShape_s_aabb: B2AABB;
    /**
     * Create a particle group whose properties have been defined.
     *
     * No reference to the definition is retained.
     *
     * warning: This function is locked during callbacks.
     */
    CreateParticleGroup(groupDef: B2ParticleGroupDef): B2ParticleGroup;
    static CreateParticleGroup_s_transform: B2Transform;
    /**
     * Join two particle groups.
     *
     * warning: This function is locked during callbacks.
     *
     * @param groupA the first group. Expands to encompass the second group.
     * @param groupB the second group. It is destroyed.
     */
    JoinParticleGroups(groupA: B2ParticleGroup, groupB: B2ParticleGroup): void;
    /**
     * Split particle group into multiple disconnected groups.
     *
     * warning: This function is locked during callbacks.
     *
     * @param group the group to be split.
     */
    SplitParticleGroup(group: B2ParticleGroup): void;
    /**
     * Get the world particle group list. With the returned group,
     * use B2ParticleGroup::GetNext to get the next group in the
     * world list.
     *
     * A null group indicates the end of the list.
     *
     * @return the head of the world particle group list.
     */
    GetParticleGroupList(): B2ParticleGroup;
    /**
     * Get the number of particle groups.
     */
    GetParticleGroupCount(): number;
    /**
     * Get the number of particles.
     */
    GetParticleCount(): number;
    /**
     * Get the maximum number of particles.
     */
    GetMaxParticleCount(): number;
    /**
     * Set the maximum number of particles.
     *
     * A value of 0 means there is no maximum. The particle buffers
     * can continue to grow while B2World's block allocator still
     * has memory.
     *
     * Note: If you try to CreateParticle() with more than this
     * count, B2_invalidParticleIndex is returned unless
     * SetDestructionByAge() is used to enable the destruction of
     * the oldest particles in the system.
     */
    SetMaxParticleCount(count: number): void;
    /**
     * Get all existing particle flags.
     */
    GetAllParticleFlags(): B2ParticleFlag;
    /**
     * Get all existing particle group flags.
     */
    GetAllGroupFlags(): B2ParticleGroupFlag;
    /**
     * Pause or unpause the particle system. When paused,
     * B2World::Step() skips over this particle system. All
     * B2ParticleSystem function calls still work.
     *
     * @param paused paused is true to pause, false to un-pause.
     */
    SetPaused(paused: boolean): void;
    /**
     * Initially, true, then, the last value passed into
     * SetPaused().
     *
     * @return true if the particle system is being updated in B2World::Step().
     */
    GetPaused(): boolean;
    /**
     * Change the particle density.
     *
     * Particle density affects the mass of the particles, which in
     * turn affects how the particles interact with B2Bodies. Note
     * that the density does not affect how the particles interact
     * with each other.
     */
    SetDensity(density: number): void;
    /**
     * Get the particle density.
     */
    GetDensity(): number;
    /**
     * Change the particle gravity scale. Adjusts the effect of the
     * global gravity vector on particles.
     */
    SetGravityScale(gravityScale: number): void;
    /**
     * Get the particle gravity scale.
     */
    GetGravityScale(): number;
    /**
     * Damping is used to reduce the velocity of particles. The
     * damping parameter can be larger than 1.0f but the damping
     * effect becomes sensitive to the time step when the damping
     * parameter is large.
     */
    SetDamping(damping: number): void;
    /**
     * Get damping for particles
     */
    GetDamping(): number;
    /**
     * Change the number of iterations when calculating the static
     * pressure of particles. By default, 8 iterations. You can
     * reduce the number of iterations down to 1 in some situations,
     * but this may cause instabilities when many particles come
     * together. If you see particles popping away from each other
     * like popcorn, you may have to increase the number of
     * iterations.
     *
     * For a description of static pressure, see
     * http://en.wikipedia.org/wiki/Static_pressure#Static_pressure_in_fluid_dynamics
     */
    SetStaticPressureIterations(iterations: number): void;
    /**
     * Get the number of iterations for static pressure of
     * particles.
     */
    GetStaticPressureIterations(): number;
    /**
     * Change the particle radius.
     *
     * You should set this only once, on world start.
     * If you change the radius during execution, existing particles
     * may explode, shrink, or behave unexpectedly.
     */
    SetRadius(radius: number): void;
    /**
     * Get the particle radius.
     */
    GetRadius(): number;
    /**
     * Get the position of each particle
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle positions array.
     */
    GetPositionBuffer(): B2Vec2[];
    /**
     * Get the velocity of each particle
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle velocities array.
     */
    GetVelocityBuffer(): B2Vec2[];
    /**
     * Get the color of each particle
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle colors array.
     */
    GetColorBuffer(): B2Color[];
    /**
     * Get the particle-group of each particle.
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle group array.
     */
    GetGroupBuffer(): B2ParticleGroup[];
    /**
     * Get the weight of each particle
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle positions array.
     */
    GetWeightBuffer(): number[];
    /**
     * Get the user-specified data of each particle.
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle user-data array.
     */
    GetUserDataBuffer(): any[];
    /**
     * Get the flags for each particle. See the B2ParticleFlag enum.
     *
     * Array is length GetParticleCount()
     *
     * @return the pointer to the head of the particle-flags array.
     */
    GetFlagsBuffer(): B2ParticleFlag[];
    /**
     * Set flags for a particle. See the B2ParticleFlag enum.
     */
    SetParticleFlags(index: number, newFlags: B2ParticleFlag): void;
    /**
     * Get flags for a particle. See the B2ParticleFlag enum.
     */
    GetParticleFlags(index: number): B2ParticleFlag;
    /**
     * Set an external buffer for particle data.
     *
     * Normally, the B2World's block allocator is used for particle
     * data. However, sometimes you may have an OpenGL or Java
     * buffer for particle data. To avoid data duplication, you may
     * supply this external buffer.
     *
     * Note that, when B2World's block allocator is used, the
     * particle data buffers can grow as required. However, when
     * external buffers are used, the maximum number of particles is
     * clamped to the size of the smallest external buffer.
     *
     * @param buffer a pointer to a block of memory.
     * @param capacity the number of values in the block.
     */
    SetFlagsBuffer(buffer: B2ParticleFlag[], capacity: number): void;
    SetPositionBuffer(buffer: B2Vec2[], capacity: number): void;
    SetVelocityBuffer(buffer: B2Vec2[], capacity: number): void;
    SetColorBuffer(buffer: B2Color[], capacity: number): void;
    SetUserDataBuffer(buffer: any[], capacity: number): void;
    /**
     * Get contacts between particles
     * Contact data can be used for many reasons, for example to
     * trigger rendering or audio effects.
     */
    GetContacts(): B2ParticleContact[];
    GetContactCount(): number;
    /**
     * Get contacts between particles and bodies
     *
     * Contact data can be used for many reasons, for example to
     * trigger rendering or audio effects.
     */
    GetBodyContacts(): B2ParticleBodyContact[];
    GetBodyContactCount(): number;
    /**
     * Get array of particle pairs. The particles in a pair:
     *   (1) are contacting,
     *   (2) are in the same particle group,
     *   (3) are part of a rigid particle group, or are spring, elastic,
     *       or wall particles.
     *   (4) have at least one particle that is a spring or barrier
     *       particle (i.e. one of the types in k_pairFlags),
     *   (5) have at least one particle that returns true for
     *       ConnectionFilter::IsNecessary,
     *   (6) are not zombie particles.
     *
     * Essentially, this is an array of spring or barrier particles
     * that are interacting. The array is sorted by B2ParticlePair's
     * indexA, and then indexB. There are no duplicate entries.
     */
    GetPairs(): B2ParticlePair[];
    GetPairCount(): number;
    /**
     * Get array of particle triads. The particles in a triad:
     *   (1) are in the same particle group,
     *   (2) are in a Voronoi triangle together,
     *   (3) are within B2_maxTriadDistance particle diameters of each
     *       other,
     *   (4) return true for ConnectionFilter::ShouldCreateTriad
     *   (5) have at least one particle of type elastic (i.e. one of the
     *       types in k_triadFlags),
     *   (6) are part of a rigid particle group, or are spring, elastic,
     *       or wall particles.
     *   (7) are not zombie particles.
     *
     * Essentially, this is an array of elastic particles that are
     * interacting. The array is sorted by B2ParticleTriad's indexA,
     * then indexB, then indexC. There are no duplicate entries.
     */
    GetTriads(): B2ParticleTriad[];
    GetTriadCount(): number;
    /**
     * Set an optional threshold for the maximum number of
     * consecutive particle iterations that a particle may contact
     * multiple bodies before it is considered a candidate for being
     * 'stuck'. Setting to zero or less disables.
     */
    SetStuckThreshold(steps: number): void;
    /**
     * Get potentially stuck particles from the last step; the user
     * must decide if they are stuck or not, and if so, delete or
     * move them
     */
    GetStuckCandidates(): number[];
    /**
     * Get the number of stuck particle candidates from the last
     * step.
     */
    GetStuckCandidateCount(): number;
    /**
     * Compute the kinetic energy that can be lost by damping force
     */
    ComputeCollisionEnergy(): number;
    static ComputeCollisionEnergy_s_v: B2Vec2;
    /**
     * Set strict Particle/Body contact check.
     *
     * This is an option that will help ensure correct behavior if
     * there are corners in the world model where Particle/Body
     * contact is ambiguous. This option scales at n*log(n) of the
     * number of Particle/Body contacts, so it is best to only
     * enable if it is necessary for your geometry. Enable if you
     * see strange particle behavior around B2Body intersections.
     */
    SetStrictContactCheck(enabled: boolean): void;
    /**
     * Get the status of the strict contact check.
     */
    GetStrictContactCheck(): boolean;
    /**
     * Set the lifetime (in seconds) of a particle relative to the
     * current time.  A lifetime of less than or equal to 0.0f
     * results in the particle living forever until it's manually
     * destroyed by the application.
     */
    SetParticleLifetime(index: number, lifetime: number): void;
    /**
     * Get the lifetime (in seconds) of a particle relative to the
     * current time.  A value > 0.0f is returned if the particle is
     * scheduled to be destroyed in the future, values <= 0.0f
     * indicate the particle has an infinite lifetime.
     */
    GetParticleLifetime(index: number): number;
    /**
     * Enable / disable destruction of particles in CreateParticle()
     * when no more particles can be created due to a prior call to
     * SetMaxParticleCount().  When this is enabled, the oldest
     * particle is destroyed in CreateParticle() favoring the
     * destruction of particles with a finite lifetime over
     * particles with infinite lifetimes. This feature is enabled by
     * default when particle lifetimes are tracked.  Explicitly
     * enabling this feature using this function enables particle
     * lifetime tracking.
     */
    SetDestructionByAge(enable: boolean): void;
    /**
     * Get whether the oldest particle will be destroyed in
     * CreateParticle() when the maximum number of particles are
     * present in the system.
     */
    GetDestructionByAge(): boolean;
    /**
     * Get the array of particle expiration times indexed by
     * particle index.
     *
     * GetParticleCount() items are in the returned array.
     */
    GetExpirationTimeBuffer(): number[];
    /**
     * Convert a expiration time value in returned by
     * GetExpirationTimeBuffer() to a time in seconds relative to
     * the current simulation time.
     */
    ExpirationTimeToLifetime(expirationTime: number): number;
    /**
     * Get the array of particle indices ordered by reverse
     * lifetime. The oldest particle indexes are at the end of the
     * array with the newest at the start.  Particles with infinite
     * lifetimes (i.e expiration times less than or equal to 0) are
     * placed at the start of the array.
     * ExpirationTimeToLifetime(GetExpirationTimeBuffer()[index]) is
     * equivalent to GetParticleLifetime(index).
     *
     * GetParticleCount() items are in the returned array.
     */
    GetIndexByExpirationTimeBuffer(): number[];
    /**
     * Apply an impulse to one particle. This immediately modifies
     * the velocity. Similar to B2Body::ApplyLinearImpulse.
     *
     * @param index the particle that will be modified.
     * @param impulse impulse the world impulse vector, usually in N-seconds or kg-m/s.
     */
    ParticleApplyLinearImpulse(index: number, impulse: B2Vec2): void;
    /**
     * Apply an impulse to all particles between 'firstIndex' and
     * 'lastIndex'. This immediately modifies the velocity. Note
     * that the impulse is applied to the total mass of all
     * particles. So, calling ParticleApplyLinearImpulse(0, impulse)
     * and ParticleApplyLinearImpulse(1, impulse) will impart twice
     * as much velocity as calling just ApplyLinearImpulse(0, 1,
     * impulse).
     *
     * @param firstIndex the first particle to be modified.
     * @param lastIndex the last particle to be modified.
     * @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
     */
    ApplyLinearImpulse(firstIndex: number, lastIndex: number, impulse: B2Vec2): void;
    static IsSignificantForce(force: B2Vec2): boolean;
    /**
     * Apply a force to the center of a particle.
     *
     * @param index the particle that will be modified.
     * @param force the world force vector, usually in Newtons (N).
     */
    ParticleApplyForce(index: number, force: B2Vec2): void;
    /**
     * Distribute a force across several particles. The particles
     * must not be wall particles. Note that the force is
     * distributed across all the particles, so calling this
     * function for indices 0..N is not the same as calling
     * ParticleApplyForce(i, force) for i in 0..N.
     *
     * @param firstIndex the first particle to be modified.
     * @param lastIndex the last particle to be modified.
     * @param force the world force vector, usually in Newtons (N).
     */
    ApplyForce(firstIndex: number, lastIndex: number, force: B2Vec2): void;
    /**
     * Get the next particle-system in the world's particle-system
     * list.
     */
    GetNext(): B2ParticleSystem;
    /**
     * Query the particle system for all particles that potentially
     * overlap the provided AABB.
     * B2QueryCallback::ShouldQueryParticleSystem is ignored.
     *
     * @param callback a user implemented callback class.
     * @param aabb the query box.
     */
    QueryAABB(callback: B2QueryCallback, aabb: B2AABB): void;
    /**
     * Query the particle system for all particles that potentially
     * overlap the provided shape's AABB. Calls QueryAABB
     * internally. B2QueryCallback::ShouldQueryParticleSystem is
     * ignored.
     *
     * @param callback a user implemented callback class.
     * @param shape the query shape
     * @param xf the transform of the AABB
     * @param childIndex
     */
    QueryShapeAABB(callback: B2QueryCallback, shape: B2Shape, xf: B2Transform, childIndex?: number): void;
    static QueryShapeAABB_s_aabb: B2AABB;
    QueryPointAABB(callback: B2QueryCallback, point: B2Vec2, slop?: number): void;
    static QueryPointAABB_s_aabb: B2AABB;
    /**
     * Ray-cast the particle system for all particles in the path of
     * the ray. Your callback controls whether you get the closest
     * point, any point, or n-points. The ray-cast ignores particles
     * that contain the starting point.
     * B2RayCastCallback::ShouldQueryParticleSystem is ignored.
     *
     * @export
     * @return {void}
     * @param {b2RayCastCallback} callback a user implemented
     *      callback class.
     * @param {b2Vec2} point1 the ray starting point
     * @param {b2Vec2} point2 the ray ending point
     */
    RayCast(callback: B2RayCastCallback, point1: B2Vec2, point2: B2Vec2): void;
    static RayCast_s_aabb: B2AABB;
    static RayCast_s_p: B2Vec2;
    static RayCast_s_v: B2Vec2;
    static RayCast_s_n: B2Vec2;
    static RayCast_s_point: B2Vec2;
    /**
     * Compute the axis-aligned bounding box for all particles
     * contained within this particle system.
     *
     * @export
     * @return {void}
     * @param {b2AABB} aabb Returns the axis-aligned bounding
     *      box of the system.
     */
    ComputeAABB(aabb: B2AABB): void;
    /**
     * All particle types that require creating pairs
     */
    static k_pairFlags: number;
    /**
     * All particle types that require creating triads
     *
     * @type {number}
     */
    static k_triadFlags: B2ParticleFlag;
    /**
     * All particle types that do not produce dynamic pressure
     *
     * @type {number}
     */
    static k_noPressureFlags: number;
    /**
     * All particle types that apply extra damping force with bodies
     *
     * @type {number}
     */
    static k_extraDampingFlags: B2ParticleFlag;
    /**
     * @type {number}
     */
    static k_barrierWallFlags: number;
    FreeBuffer(b: any, capacity: number): void;
    FreeUserOverridableBuffer(b: B2ParticleSystem.UserOverridableBuffer<any>): void;
    /**
     * Reallocate a buffer
     */
    ReallocateBuffer3(oldBuffer: any[], oldCapacity: number, newCapacity: number): any[];
    /**
     * Reallocate a buffer
     */
    ReallocateBuffer5(buffer: any[], userSuppliedCapacity: number, oldCapacity: number, newCapacity: number, deferred: boolean): any[];
    /**
     * Reallocate a buffer
     */
    ReallocateBuffer4(buffer: B2ParticleSystem.UserOverridableBuffer<any>, oldCapacity: number, newCapacity: number, deferred: boolean): any[];
    RequestBuffer(buffer: any[]): any[];
    /**
     * Reallocate the handle / index map and schedule the allocation
     * of a new pool for handle allocation.
     */
    ReallocateHandleBuffers(newCapacity: number): void;
    ReallocateInternalAllocatedBuffers(capacity: number): void;
    CreateParticleForGroup(groupDef: B2ParticleGroupDef, xf: B2Transform, p: B2Vec2): void;
    CreateParticlesStrokeShapeForGroup(shape: B2Shape, groupDef: B2ParticleGroupDef, xf: B2Transform): void;
    static CreateParticlesStrokeShapeForGroup_s_edge: B2EdgeShape;
    static CreateParticlesStrokeShapeForGroup_s_d: B2Vec2;
    static CreateParticlesStrokeShapeForGroup_s_p: B2Vec2;
    CreateParticlesFillShapeForGroup(shape: B2Shape, groupDef: B2ParticleGroupDef, xf: B2Transform): void;
    static CreateParticlesFillShapeForGroup_s_aabb: B2AABB;
    static CreateParticlesFillShapeForGroup_s_p: B2Vec2;
    CreateParticlesWithShapeForGroup(shape: B2Shape, groupDef: B2ParticleGroupDef, xf: B2Transform): void;
    CreateParticlesWithShapesForGroup(shapes: B2Shape[], shapeCount: number, groupDef: B2ParticleGroupDef, xf: B2Transform): void;
    CloneParticle(oldIndex: number, group: B2ParticleGroup): number;
    DestroyParticlesInGroup(group: B2ParticleGroup, callDestructionListener?: boolean): void;
    DestroyParticleGroup(group: B2ParticleGroup): void;
    static ParticleCanBeConnected(flags: B2ParticleFlag, group: B2ParticleGroup): boolean;
    UpdatePairsAndTriads(firstIndex: number, lastIndex: number, filter: B2ParticleSystem.ConnectionFilter): void;
    private static UpdatePairsAndTriads_s_dab;
    private static UpdatePairsAndTriads_s_dbc;
    private static UpdatePairsAndTriads_s_dca;
    UpdatePairsAndTriadsWithReactiveParticles(): void;
    static ComparePairIndices(a: B2ParticlePair, b: B2ParticlePair): boolean;
    static MatchPairIndices(a: B2ParticlePair, b: B2ParticlePair): boolean;
    static CompareTriadIndices(a: B2ParticleTriad, b: B2ParticleTriad): boolean;
    static MatchTriadIndices(a: B2ParticleTriad, b: B2ParticleTriad): boolean;
    static InitializeParticleLists(group: B2ParticleGroup, nodeBuffer: B2ParticleSystem.ParticleListNode[]): void;
    MergeParticleListsInContact(group: B2ParticleGroup, nodeBuffer: B2ParticleSystem.ParticleListNode[]): void;
    static MergeParticleLists(listA: B2ParticleSystem.ParticleListNode, listB: B2ParticleSystem.ParticleListNode): void;
    static FindLongestParticleList(group: B2ParticleGroup, nodeBuffer: B2ParticleSystem.ParticleListNode[]): B2ParticleSystem.ParticleListNode;
    MergeZombieParticleListNodes(group: B2ParticleGroup, nodeBuffer: B2ParticleSystem.ParticleListNode[], survivingList: B2ParticleSystem.ParticleListNode): void;
    static MergeParticleListAndNode(list: B2ParticleSystem.ParticleListNode, node: B2ParticleSystem.ParticleListNode): void;
    CreateParticleGroupsFromParticleList(group: B2ParticleGroup, nodeBuffer: B2ParticleSystem.ParticleListNode[], survivingList: B2ParticleSystem.ParticleListNode): void;
    UpdatePairsAndTriadsWithParticleList(group: B2ParticleGroup, nodeBuffer: B2ParticleSystem.ParticleListNode[]): void;
    ComputeDepth(): void;
    GetInsideBoundsEnumerator(aabb: B2AABB): B2ParticleSystem.InsideBoundsEnumerator;
    UpdateAllParticleFlags(): void;
    UpdateAllGroupFlags(): void;
    AddContact(a: number, b: number, contacts: B2GrowableBuffer<B2ParticleContact>): void;
    static AddContact_s_d: B2Vec2;
    FindContacts_Reference(contacts: B2GrowableBuffer<B2ParticleContact>): void;
    FindContacts(contacts: B2GrowableBuffer<B2ParticleContact>): void;
    UpdateProxies_Reference(proxies: B2GrowableBuffer<B2ParticleSystem.Proxy>): void;
    UpdateProxies(proxies: B2GrowableBuffer<B2ParticleSystem.Proxy>): void;
    SortProxies(proxies: B2GrowableBuffer<B2ParticleSystem.Proxy>): void;
    FilterContacts(contacts: B2GrowableBuffer<B2ParticleContact>): void;
    NotifyContactListenerPreContact(particlePairs: B2ParticleSystem.B2ParticlePairSet): void;
    NotifyContactListenerPostContact(particlePairs: B2ParticleSystem.B2ParticlePairSet): void;
    static B2ParticleContactIsZombie(contact: B2ParticleContact): boolean;
    UpdateContacts(exceptZombie: boolean): void;
    NotifyBodyContactListenerPreContact(fixtureSet: B2ParticleSystem.FixtureParticleSet): void;
    NotifyBodyContactListenerPostContact(fixtureSet: B2ParticleSystem.FixtureParticleSet): void;
    UpdateBodyContacts(): void;
    static UpdateBodyContacts_s_aabb: B2AABB;
    Solve(step: B2TimeStep): void;
    static Solve_s_subStep: B2TimeStep;
    SolveCollision(step: B2TimeStep): void;
    static SolveCollision_s_aabb: B2AABB;
    LimitVelocity(step: B2TimeStep): void;
    SolveGravity(step: B2TimeStep): void;
    static SolveGravity_s_gravity: B2Vec2;
    SolveBarrier(step: B2TimeStep): void;
    static SolveBarrier_s_aabb: B2AABB;
    static SolveBarrier_s_va: B2Vec2;
    static SolveBarrier_s_vb: B2Vec2;
    static SolveBarrier_s_pba: B2Vec2;
    static SolveBarrier_s_vba: B2Vec2;
    static SolveBarrier_s_vc: B2Vec2;
    static SolveBarrier_s_pca: B2Vec2;
    static SolveBarrier_s_vca: B2Vec2;
    static SolveBarrier_s_qba: B2Vec2;
    static SolveBarrier_s_qca: B2Vec2;
    static SolveBarrier_s_dv: B2Vec2;
    static SolveBarrier_s_f: B2Vec2;
    SolveStaticPressure(step: B2TimeStep): void;
    ComputeWeight(): void;
    SolvePressure(step: B2TimeStep): void;
    static SolvePressure_s_f: B2Vec2;
    SolveDamping(step: B2TimeStep): void;
    static SolveDamping_s_v: B2Vec2;
    static SolveDamping_s_f: B2Vec2;
    SolveRigidDamping(): void;
    static SolveRigidDamping_s_t0: B2Vec2;
    static SolveRigidDamping_s_t1: B2Vec2;
    static SolveRigidDamping_s_p: B2Vec2;
    static SolveRigidDamping_s_v: B2Vec2;
    SolveExtraDamping(): void;
    static SolveExtraDamping_s_v: B2Vec2;
    static SolveExtraDamping_s_f: B2Vec2;
    SolveWall(): void;
    SolveRigid(step: B2TimeStep): void;
    static SolveRigid_s_position: B2Vec2;
    static SolveRigid_s_rotation: B2Rot;
    static SolveRigid_s_transform: B2Transform;
    static SolveRigid_s_velocityTransform: B2Transform;
    SolveElastic(step: B2TimeStep): void;
    static SolveElastic_s_pa: B2Vec2;
    static SolveElastic_s_pb: B2Vec2;
    static SolveElastic_s_pc: B2Vec2;
    static SolveElastic_s_r: B2Rot;
    static SolveElastic_s_t0: B2Vec2;
    SolveSpring(step: B2TimeStep): void;
    static SolveSpring_s_pa: B2Vec2;
    static SolveSpring_s_pb: B2Vec2;
    static SolveSpring_s_d: B2Vec2;
    static SolveSpring_s_f: B2Vec2;
    SolveTensile(step: B2TimeStep): void;
    static SolveTensile_s_weightedNormal: B2Vec2;
    static SolveTensile_s_s: B2Vec2;
    static SolveTensile_s_f: B2Vec2;
    SolveViscous(): void;
    static SolveViscous_s_v: B2Vec2;
    static SolveViscous_s_f: B2Vec2;
    SolveRepulsive(step: B2TimeStep): void;
    static SolveRepulsive_s_f: B2Vec2;
    SolvePowder(step: B2TimeStep): void;
    static SolvePowder_s_f: B2Vec2;
    SolveSolid(step: B2TimeStep): void;
    static SolveSolid_s_f: B2Vec2;
    SolveForce(step: B2TimeStep): void;
    SolveColorMixing(): void;
    SolveZombie(): void;
    /**
     * Destroy all particles which have outlived their lifetimes set
     * by SetParticleLifetime().
     */
    SolveLifetimes(step: B2TimeStep): void;
    RotateBuffer(start: number, mid: number, end: number): void;
    GetCriticalVelocity(step: B2TimeStep): number;
    GetCriticalVelocitySquared(step: B2TimeStep): number;
    GetCriticalPressure(step: B2TimeStep): number;
    GetParticleStride(): number;
    GetParticleMass(): number;
    GetParticleInvMass(): number;
    /**
     * Get the world's contact filter if any particles with the
     * B2_contactFilterParticle flag are present in the system.
     */
    GetFixtureContactFilter(): B2ContactFilter;
    /**
     * Get the world's contact filter if any particles with the
     * B2_particleContactFilterParticle flag are present in the
     * system.
     */
    GetParticleContactFilter(): B2ContactFilter;
    /**
     * Get the world's contact listener if any particles with the
     * B2_fixtureContactListenerParticle flag are present in the
     * system.
     */
    GetFixtureContactListener(): B2ContactListener;
    /**
     * Get the world's contact listener if any particles with the
     * B2_particleContactListenerParticle flag are present in the
     * system.
     */
    GetParticleContactListener(): B2ContactListener;
    SetUserOverridableBuffer(buffer: B2ParticleSystem.UserOverridableBuffer<any>, newData: any[], newCapacity: number): void;
    SetGroupFlags(group: B2ParticleGroup, newFlags: B2ParticleGroupFlag): void;
    static BodyContactCompare(lhs: B2ParticleBodyContact, rhs: B2ParticleBodyContact): boolean;
    RemoveSpuriousBodyContacts(): void;
    private static RemoveSpuriousBodyContacts_s_n;
    private static RemoveSpuriousBodyContacts_s_pos;
    private static RemoveSpuriousBodyContacts_s_normal;
    DetectStuckParticle(particle: number): void;
    /**
     * Determine whether a particle index is valid.
     */
    ValidateParticleIndex(index: number): boolean;
    /**
     * Get the time elapsed in
     * B2ParticleSystemDef::lifetimeGranularity.
     */
    GetQuantizedTimeElapsed(): number;
    /**
     * Convert a lifetime in seconds to an expiration time.
     */
    LifetimeToExpirationTime(lifetime: number): number;
    ForceCanBeApplied(flags: B2ParticleFlag): boolean;
    PrepareForceBuffer(): void;
    IsRigidGroup(group: B2ParticleGroup): boolean;
    GetLinearVelocity(group: B2ParticleGroup, particleIndex: number, point: B2Vec2, out: B2Vec2): B2Vec2;
    InitDampingParameter(invMass: number[], invInertia: number[], tangentDistance: number[], mass: number, inertia: number, center: B2Vec2, point: B2Vec2, normal: B2Vec2): void;
    InitDampingParameterWithRigidGroupOrParticle(invMass: number[], invInertia: number[], tangentDistance: number[], isRigidGroup: boolean, group: B2ParticleGroup, particleIndex: number, point: B2Vec2, normal: B2Vec2): void;
    ComputeDampingImpulse(invMassA: number, invInertiaA: number, tangentDistanceA: number, invMassB: number, invInertiaB: number, tangentDistanceB: number, normalVelocity: number): number;
    ApplyDamping(invMass: number, invInertia: number, tangentDistance: number, isRigidGroup: boolean, group: B2ParticleGroup, particleIndex: number, impulse: number, normal: B2Vec2): void;
}
export declare namespace B2ParticleSystem {
    class UserOverridableBuffer<T> {
        data: T[];
        userSuppliedCapacity: number;
    }
    class Proxy {
        index: number;
        tag: number;
        static CompareProxyProxy(a: Proxy, b: Proxy): boolean;
        static CompareTagProxy(a: number, b: Proxy): boolean;
        static CompareProxyTag(a: Proxy, b: number): boolean;
    }
    class InsideBoundsEnumerator {
        m_system: B2ParticleSystem;
        m_xLower: number;
        m_xUpper: number;
        m_yLower: number;
        m_yUpper: number;
        m_first: number;
        m_last: number;
        /**
         * InsideBoundsEnumerator enumerates all particles inside the
         * given bounds.
         *
         * Construct an enumerator with bounds of tags and a range of
         * proxies.
         */
        constructor(system: B2ParticleSystem, lower: number, upper: number, first: number, last: number);
        /**
         * Get index of the next particle. Returns
         * B2_invalidParticleIndex if there are no more particles.
         */
        GetNext(): number;
    }
    class ParticleListNode {
        /**
         * The head of the list.
         */
        list: B2ParticleSystem.ParticleListNode;
        /**
         * The next node in the list.
         */
        next: B2ParticleSystem.ParticleListNode;
        /**
         * Number of entries in the list. Valid only for the node at the
         * head of the list.
         */
        count: number;
        /**
         * Particle index.
         */
        index: number;
    }
    /**
     * @constructor
     */
    class FixedSetAllocator {
        Allocate(itemSize: number, count: number): number;
        Clear(): void;
        GetCount(): number;
        Invalidate(itemIndex: number): void;
        GetValidBuffer(): boolean[];
        GetBuffer(): any[];
        SetCount(count: number): void;
    }
    class FixtureParticle {
        first: B2Fixture;
        second: number;
        constructor(fixture: B2Fixture, particle: number);
    }
    class FixtureParticleSet extends B2ParticleSystem.FixedSetAllocator {
        Initialize(bodyContactBuffer: B2GrowableBuffer<B2ParticleBodyContact>, flagsBuffer: B2ParticleSystem.UserOverridableBuffer<B2ParticleFlag>): void;
        Find(pair: B2ParticleSystem.FixtureParticle): number;
    }
    class ParticlePair {
        first: number;
        second: number;
        constructor(particleA: number, particleB: number);
    }
    class B2ParticlePairSet extends B2ParticleSystem.FixedSetAllocator {
        Initialize(contactBuffer: B2GrowableBuffer<B2ParticleContact>, flagsBuffer: UserOverridableBuffer<B2ParticleFlag>): void;
        /**
         * @return {number}
         * @param {b2ParticleSystem.ParticlePair} pair
         */
        Find(pair: B2ParticleSystem.ParticlePair): number;
    }
    class ConnectionFilter {
        /**
         * Is the particle necessary for connection?
         * A pair or a triad should contain at least one 'necessary'
         * particle.
         */
        IsNecessary(index: number): boolean;
        /**
         * An additional condition for creating a pair.
         */
        ShouldCreatePair(a: number, b: number): boolean;
        /**
         * An additional condition for creating a triad.
         */
        ShouldCreateTriad(a: number, b: number, c: number): boolean;
    }
    class DestroyParticlesInShapeCallback extends B2QueryCallback {
        m_system: B2ParticleSystem;
        m_shape: B2Shape;
        m_xf: B2Transform;
        m_callDestructionListener: boolean;
        m_destroyed: number;
        constructor(system: B2ParticleSystem, shape: B2Shape, xf: B2Transform, callDestructionListener: boolean);
        ReportFixture(fixture: B2Fixture): boolean;
        ReportParticle(particleSystem: B2ParticleSystem, index: number): boolean;
        Destroyed(): number;
    }
    class JoinParticleGroupsFilter extends B2ParticleSystem.ConnectionFilter {
        m_threshold: number;
        constructor(threshold: number);
        /**
         * An additional condition for creating a pair.
         */
        ShouldCreatePair(a: number, b: number): boolean;
        /**
         * An additional condition for creating a triad.
         */
        ShouldCreateTriad(a: number, b: number, c: number): boolean;
    }
    class CompositeShape extends B2Shape {
        constructor(shapes: B2Shape[], shapeCount: number);
        m_shapes: B2Shape[];
        m_shapeCount: number;
        Clone(): B2Shape;
        GetChildCount(): number;
        /**
         * @see B2Shape::TestPoint
         */
        TestPoint(xf: B2Transform, p: B2Vec2): boolean;
        /**
         * @see B2Shape::ComputeDistance
         */
        ComputeDistance(xf: B2Transform, p: B2Vec2, normal: B2Vec2, childIndex: number): number;
        /**
         * Implement B2Shape.
         */
        RayCast(output: B2RayCastOutput, input: B2RayCastInput, xf: B2Transform, childIndex: number): boolean;
        /**
         * @see B2Shape::ComputeAABB
         */
        ComputeAABB(aabb: B2AABB, xf: B2Transform, childIndex: number): void;
        /**
         * @see B2Shape::ComputeMass
         */
        ComputeMass(massData: B2MassData, density: number): void;
        SetupDistanceProxy(proxy: B2DistanceProxy, index: number): void;
        ComputeSubmergedArea(normal: B2Vec2, offset: number, xf: B2Transform, c: B2Vec2): number;
        Dump(log: (format: string, ...args: any[]) => void): void;
    }
    class ReactiveFilter extends B2ParticleSystem.ConnectionFilter {
        m_flagsBuffer: B2ParticleSystem.UserOverridableBuffer<B2ParticleFlag>;
        constructor(flagsBuffer: B2ParticleSystem.UserOverridableBuffer<B2ParticleFlag>);
        IsNecessary(index: number): boolean;
    }
    class UpdateBodyContactsCallback extends B2FixtureParticleQueryCallback {
        m_contactFilter: B2ContactFilter;
        constructor(system: B2ParticleSystem, contactFilter: B2ContactFilter);
        ShouldCollideFixtureParticle(fixture: B2Fixture, particleSystem: B2ParticleSystem, particleIndex: number): boolean;
        ReportFixtureAndParticle(fixture: B2Fixture, childIndex: number, a: number): void;
        static ReportFixtureAndParticle_s_n: B2Vec2;
        static ReportFixtureAndParticle_s_rp: B2Vec2;
    }
    class SolveCollisionCallback extends B2FixtureParticleQueryCallback {
        m_step: B2TimeStep;
        constructor(system: B2ParticleSystem, step: B2TimeStep);
        ReportFixtureAndParticle(fixture: B2Fixture, childIndex: number, a: number): void;
        static ReportFixtureAndParticle_s_p1: B2Vec2;
        static ReportFixtureAndParticle_s_output: B2RayCastOutput;
        static ReportFixtureAndParticle_s_input: B2RayCastInput;
        static ReportFixtureAndParticle_s_p: B2Vec2;
        static ReportFixtureAndParticle_s_v: B2Vec2;
        static ReportFixtureAndParticle_s_f: B2Vec2;
        /**
         * @export
         * @return {boolean}
         * @param {b2ParticleSystem} system
         * @param {number} index
         */
        ReportParticle(system: B2ParticleSystem, index: number): boolean;
    }
}
