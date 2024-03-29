"use strict";
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
Object.defineProperty(exports, "__esModule", { value: true });
exports.B2VoronoiDiagram = void 0;
/// #if B2_ENABLE_PARTICLE
const b2Settings_1 = require("../Common/b2Settings");
const b2Math_1 = require("../Common/b2Math");
const b2StackQueue_1 = require("./b2StackQueue");
function B2Assert(condition) { }
/**
 * A field representing the nearest generator from each point.
 */
class B2VoronoiDiagram {
    constructor(generatorCapacity) {
        this.m_generatorBuffer = null;
        this.m_generatorCapacity = 0;
        this.m_generatorCount = 0;
        this.m_countX = 0;
        this.m_countY = 0;
        this.m_diagram = null;
        this.m_generatorBuffer = b2Settings_1.B2MakeArray(generatorCapacity, function (index) {
            return new B2VoronoiDiagram.Generator();
        });
        this.m_generatorCapacity = generatorCapacity;
    }
    /**
     * Add a generator.
     *
     * @param center the position of the generator.
     * @param tag a tag used to identify the generator in callback functions.
     * @param necessary whether to callback for nodes associated with the generator.
     */
    AddGenerator(center, tag, necessary) {
        B2Assert(this.m_generatorCount < this.m_generatorCapacity);
        let g = this.m_generatorBuffer[this.m_generatorCount++];
        g.center.Copy(center);
        g.tag = tag;
        g.necessary = necessary;
    }
    /**
     * Generate the Voronoi diagram. It is rasterized with a given
     * interval in the same range as the necessary generators exist.
     *
     * @param radius the interval of the diagram.
     * @param margin margin for which the range of the diagram is extended.
     */
    Generate(radius, margin) {
        B2Assert(this.m_diagram === null);
        let inverseRadius = 1 / radius;
        let lower = new b2Math_1.B2Vec2(+b2Settings_1.B2_maxFloat, +b2Settings_1.B2_maxFloat);
        let upper = new b2Math_1.B2Vec2(-b2Settings_1.B2_maxFloat, -b2Settings_1.B2_maxFloat);
        let necessary_count = 0;
        for (let k = 0; k < this.m_generatorCount; k++) {
            let g = this.m_generatorBuffer[k];
            if (g.necessary) {
                b2Math_1.B2Vec2.MinV(lower, g.center, lower);
                b2Math_1.B2Vec2.MaxV(upper, g.center, upper);
                ++necessary_count;
            }
        }
        if (necessary_count === 0) {
            /// debugger;
            this.m_countX = 0;
            this.m_countY = 0;
            return;
        }
        lower.x -= margin;
        lower.y -= margin;
        upper.x += margin;
        upper.y += margin;
        this.m_countX = 1 + Math.floor(inverseRadius * (upper.x - lower.x));
        this.m_countY = 1 + Math.floor(inverseRadius * (upper.y - lower.y));
        ///   m_diagram = (Generator**) m_allocator->Allocate(sizeof(Generator*) * m_countX * m_countY);
        ///   for (int32 i = 0; i < m_countX * m_countY; i++)
        ///   {
        ///     m_diagram[i] = NULL;
        ///   }
        this.m_diagram = b2Settings_1.B2MakeArray(this.m_countX * this.m_countY, function (index) {
            return null;
        });
        // (4 * m_countX * m_countY) is the queue capacity that is experimentally
        // known to be necessary and sufficient for general particle distributions.
        let queue = new b2StackQueue_1.B2StackQueue(4 * this.m_countX * this.m_countY);
        for (let k = 0; k < this.m_generatorCount; k++) {
            let g = this.m_generatorBuffer[k];
            ///   g.center = inverseRadius * (g.center - lower);
            g.center.SelfSub(lower).SelfMul(inverseRadius);
            let x = Math.floor(g.center.x);
            let y = Math.floor(g.center.y);
            if (x >= 0 && y >= 0 && x < this.m_countX && y < this.m_countY) {
                queue.Push(new B2VoronoiDiagram.Task(x, y, x + y * this.m_countX, g));
            }
        }
        while (!queue.Empty()) {
            let task = queue.Front();
            let x = task.m_x;
            let y = task.m_y;
            let i = task.m_i;
            let g = task.m_generator;
            queue.Pop();
            if (!this.m_diagram[i]) {
                this.m_diagram[i] = g;
                if (x > 0) {
                    queue.Push(new B2VoronoiDiagram.Task(x - 1, y, i - 1, g));
                }
                if (y > 0) {
                    queue.Push(new B2VoronoiDiagram.Task(x, y - 1, i - this.m_countX, g));
                }
                if (x < this.m_countX - 1) {
                    queue.Push(new B2VoronoiDiagram.Task(x + 1, y, i + 1, g));
                }
                if (y < this.m_countY - 1) {
                    queue.Push(new B2VoronoiDiagram.Task(x, y + 1, i + this.m_countX, g));
                }
            }
        }
        for (let y = 0; y < this.m_countY; y++) {
            for (let x = 0; x < this.m_countX - 1; x++) {
                let i = x + y * this.m_countX;
                let a = this.m_diagram[i];
                let b = this.m_diagram[i + 1];
                if (a !== b) {
                    queue.Push(new B2VoronoiDiagram.Task(x, y, i, b));
                    queue.Push(new B2VoronoiDiagram.Task(x + 1, y, i + 1, a));
                }
            }
        }
        for (let y = 0; y < this.m_countY - 1; y++) {
            for (let x = 0; x < this.m_countX; x++) {
                let i = x + y * this.m_countX;
                let a = this.m_diagram[i];
                let b = this.m_diagram[i + this.m_countX];
                if (a !== b) {
                    queue.Push(new B2VoronoiDiagram.Task(x, y, i, b));
                    queue.Push(new B2VoronoiDiagram.Task(x, y + 1, i + this.m_countX, a));
                }
            }
        }
        while (!queue.Empty()) {
            let task = queue.Front();
            let x = task.m_x;
            let y = task.m_y;
            let i = task.m_i;
            let k = task.m_generator;
            queue.Pop();
            let a = this.m_diagram[i];
            let b = k;
            if (a !== b) {
                let ax = a.center.x - x;
                let ay = a.center.y - y;
                let bx = b.center.x - x;
                let by = b.center.y - y;
                let a2 = ax * ax + ay * ay;
                let B2 = bx * bx + by * by;
                if (a2 > B2) {
                    this.m_diagram[i] = b;
                    if (x > 0) {
                        queue.Push(new B2VoronoiDiagram.Task(x - 1, y, i - 1, b));
                    }
                    if (y > 0) {
                        queue.Push(new B2VoronoiDiagram.Task(x, y - 1, i - this.m_countX, b));
                    }
                    if (x < this.m_countX - 1) {
                        queue.Push(new B2VoronoiDiagram.Task(x + 1, y, i + 1, b));
                    }
                    if (y < this.m_countY - 1) {
                        queue.Push(new B2VoronoiDiagram.Task(x, y + 1, i + this.m_countX, b));
                    }
                }
            }
        }
    }
    /**
     * Enumerate all nodes that contain at least one necessary
     * generator.
     */
    GetNodes(callback) {
        for (let y = 0; y < this.m_countY - 1; y++) {
            for (let x = 0; x < this.m_countX - 1; x++) {
                let i = x + y * this.m_countX;
                let a = this.m_diagram[i];
                let b = this.m_diagram[i + 1];
                let c = this.m_diagram[i + this.m_countX];
                let d = this.m_diagram[i + 1 + this.m_countX];
                if (b !== c) {
                    if (a !== b &&
                        a !== c &&
                        (a.necessary || b.necessary || c.necessary)) {
                        callback(a.tag, b.tag, c.tag);
                    }
                    if (d !== b &&
                        d !== c &&
                        (a.necessary || b.necessary || c.necessary)) {
                        callback(b.tag, d.tag, c.tag);
                    }
                }
            }
        }
    }
}
exports.B2VoronoiDiagram = B2VoronoiDiagram;
(function (B2VoronoiDiagram) {
    class Generator {
        constructor() {
            this.center = new b2Math_1.B2Vec2();
            this.tag = 0;
            this.necessary = false;
        }
    }
    B2VoronoiDiagram.Generator = Generator;
    class Task {
        constructor(x, y, i, g) {
            this.m_x = 0;
            this.m_y = 0;
            this.m_i = 0;
            this.m_generator = null;
            this.m_x = x;
            this.m_y = y;
            this.m_i = i;
            this.m_generator = g;
        }
    }
    B2VoronoiDiagram.Task = Task;
})(B2VoronoiDiagram = exports.B2VoronoiDiagram || (exports.B2VoronoiDiagram = {})); // namespace B2VoronoiDiagram
/// #endif
